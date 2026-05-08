#!/usr/bin/env python3
"""
Empirical bench harness for Hiwonder RRC Lite motor stop behavior.

Bypasses the adapter framework and system-manager — opens the serial
port directly so you can try each candidate stop variant and observe
which one actually halts a free-spinning wheel. Prints the exact bytes
put on the wire so they can be cross-referenced with the protocol PDF.

Usage:
    # On the robot, after stopping the consumer of /dev/ttyACM0:
    sudo systemctl stop polyflow-system-manager
    python3 -m hiwonder_rrc_adapter.scripts.test_stop --motor 1
    # or, with the package on PYTHONPATH:
    python3 scripts/test_stop.py --port /dev/ttyACM0 --motor 1
"""

import argparse
import importlib.util
import struct
import sys
import time
from pathlib import Path

# Load rrc.py as a standalone module (skipping hiwonder_rrc_adapter/__init__.py,
# which imports the SDK-dependent adapter classes we don't need here).
_RRC_PATH = Path(__file__).resolve().parents[1] / "src" / "hiwonder_rrc_adapter" / "rrc.py"
_spec = importlib.util.spec_from_file_location("rrc", _RRC_PATH)
rrc_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rrc_mod)
HiwonderRRC = rrc_mod.HiwonderRRC
Func = rrc_mod.Func
_crc8 = rrc_mod._crc8


# Sub-command bytes from the 2026-05 RRC Lite "host-side motor protocol" spec.
SUB_SPEED_SINGLE = 0x00   # one motor, signed float r/s (PID — needs working encoders)
SUB_SPEED_MULTI  = 0x01   # N motors                    (PID — needs working encoders)
SUB_STOP_SINGLE  = 0x02   # halt one motor
SUB_STOP_MASK    = 0x03   # halt N motors (bit i = motor i)
SUB_RAW_PWM      = 0x04   # one motor, signed i16 PWM (±1000) — open-loop

SPINUP_RPS = 5.0
SPINUP_PWM = 500          # 50% duty for raw-PWM spin-up


def hexbytes(buf):
    return " ".join(f"{b:02X}" for b in buf)


def predict_frame(func, data):
    buf = bytearray([0xAA, 0x55, func, len(data)])
    buf.extend(data)
    buf.append(_crc8(bytes(buf[2:])))
    return buf


def send(rrc, data, label):
    print(f"  {label}")
    print(f"  → {hexbytes(predict_frame(Func.MOTOR, data))}")
    rrc._send(Func.MOTOR, list(data))


def listen(rrc, seconds=10.0):
    """Passively capture incoming frames for `seconds`. Parses anything that
    starts with AA 55 as a protocol frame; everything else is shown as raw."""
    print(f"  listening for {seconds}s — emit any input the board produces...")
    deadline = time.monotonic() + seconds
    buf = bytearray()
    frames_seen = 0
    raw_seen = 0

    def try_parse_frame(b):
        # b starts at AA. Returns (frame_bytes, consumed) or (None, 0) if incomplete
        if len(b) < 5:
            return None, 0
        if b[0] != 0xAA or b[1] != 0x55:
            return None, 1  # consume one byte, resync
        func = b[2]
        dlen = b[3]
        total = 4 + dlen + 1
        if len(b) < total:
            return None, 0  # need more
        data = bytes(b[4:4 + dlen])
        crc_recv = b[4 + dlen]
        crc_calc = _crc8(bytes([func, dlen]) + data)
        ok = (crc_recv == crc_calc)
        return (func, dlen, data, crc_recv, ok), total

    try:
        while time.monotonic() < deadline:
            n = rrc._serial.in_waiting
            if n:
                buf.extend(rrc._serial.read(n))
                # Try to parse as many frames as possible
                while buf:
                    parsed, consumed = try_parse_frame(buf)
                    if parsed is None and consumed == 0:
                        break  # need more bytes
                    if parsed is None and consumed > 0:
                        raw_seen += 1
                        del buf[:consumed]
                        continue
                    func, dlen, data, crc_recv, ok = parsed
                    frames_seen += 1
                    func_name = {0:"SYS", 1:"LED", 2:"BUZZER", 3:"MOTOR", 4:"PWM_SERVO",
                                 5:"BUS_SERVO", 6:"KEY", 7:"IMU", 8:"GAMEPAD",
                                 9:"SBUS", 10:"OLED", 11:"RGB"}.get(func, f"?({func:#04x})")
                    sub = data[0] if dlen >= 1 else None
                    sub_str = f"sub=0x{sub:02X}" if sub is not None else ""
                    crc_str = "OK" if ok else f"BAD (got 0x{crc_recv:02X})"
                    print(f"  [{func_name:9s}] {sub_str:10s} dlen={dlen:3d}  "
                          f"data={hexbytes(data) or '(none)'}  crc={crc_str}")
                    del buf[:consumed]
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("  (interrupted)")

    print(f"  done. parsed {frames_seen} frames, {raw_seen} resync byte(s) discarded.")
    if buf:
        print(f"  trailing buffer ({len(buf)} bytes): {hexbytes(buf[:64])}")


def probe(rrc, func, payload, wait=0.3):
    """Send a packet on `func` with `payload` and print whatever bytes come back.
    Read-only style — does not write rrc state. Used to discover undocumented
    sub-commands (e.g., a hidden 'get version' under SYS=0x00)."""
    frame = predict_frame(func, payload)
    print(f"  → {hexbytes(frame)}")
    # Drain any stale bytes first
    if rrc._serial.in_waiting:
        stale = rrc._serial.read(rrc._serial.in_waiting)
        print(f"  (drained {len(stale)} stale bytes)")
    rrc._send(func, list(payload))
    rrc._serial.flush()
    deadline = time.monotonic() + wait
    chunks = []
    while time.monotonic() < deadline:
        n = rrc._serial.in_waiting
        if n:
            chunks.append(rrc._serial.read(n))
        else:
            time.sleep(0.02)
    resp = b"".join(chunks)
    if resp:
        print(f"  ← {hexbytes(resp)}")
    else:
        print(f"  ← (no response within {wait}s)")


def speed_single(motor_byte, speed_rps):
    """Sub-cmd 0x00 — single-motor form. May be limited to one motor at a time."""
    return [SUB_SPEED_SINGLE, motor_byte] + list(struct.pack("<f", float(speed_rps)))


def speed_multi(motors):
    """Sub-cmd 0x01 — multi-motor form. This is what the production adapter uses
    (with count=1). Pass a list of (motor_byte, speed_rps) tuples."""
    data = [SUB_SPEED_MULTI, len(motors)]
    for motor_byte, speed in motors:
        data.append(motor_byte)
        data.extend(struct.pack("<f", float(speed)))
    return data


def stop_single(motor_byte):
    return [SUB_STOP_SINGLE, motor_byte]


def stop_mask(mask):
    return [SUB_STOP_MASK, mask & 0xFF]


def pwm_single(motor_byte, pwm):
    """Sub-cmd 0x04 — raw PWM, single motor. Payload: motor_id:u8, pwm:i16 LE."""
    pwm = max(-1000, min(1000, int(pwm)))
    return list(struct.pack("<BBh", SUB_RAW_PWM, motor_byte, pwm))


MENU = """
========= RRC Lite Motor Stop Tests =========
Motor port: {motor}    (doc byte={doc_idx},  our byte={our_idx})

Spin-up (sub-cmd 0x00, single-motor form — may have firmware quirks):
  1)  spin at +{spinup} r/s, doc-indexed (byte=0x{doc_idx:02X})
  2)  spin at +{spinup} r/s, our-indexed (byte=0x{our_idx:02X})

Spin-up (sub-cmd 0x01, multi-motor form — what the production adapter uses):
  3)  spin at +{spinup} r/s, doc-indexed
  4)  spin at +{spinup} r/s, our-indexed
  r)  repeat last spin-up

Stop variants — each sends ONE packet, watch the wheel:
  a)  speed=0,  sub-cmd 0x00, doc-indexed
  b)  speed=0,  sub-cmd 0x00, our-indexed
  c)  stop,     sub-cmd 0x02, doc-indexed (byte=0x{doc_idx:02X})
  d)  stop,     sub-cmd 0x02, our-indexed (byte=0x{our_idx:02X})
  e)  stop mask sub-cmd 0x03, mask=0x{mask_doc:02X}  (doc: bit {doc_bit} = motor {motor})
  f)  stop mask sub-cmd 0x03, mask=0x{mask_alt:02X}  (off-by-one alternative)
  g)  pwm=0,    sub-cmd 0x04 raw PWM (open-loop)
  j)  speed=0.0001, sub-cmd 0x00, doc-indexed (epsilon trick)

Active brake (combo):
  h)  speed=-3 r/s, then sub-cmd 0x02 stop after 200ms
  i)  speed=-1 r/s, then sub-cmd 0x03 mask  after 200ms

Multi:
  k)  ALL motors stop, mask=0x0F
  n)  speed=0 via sub-cmd 0x01 (multi count=1) doc-indexed
  o)  speed=0 via sub-cmd 0x01 (multi count=1) our-indexed
  s)  sustained speed=-1 via sub-cmd 0x01 our-indexed (single packet)
  s5) loop: speed=-1 every 200ms for 5 sec (simulate gamepad-rate brake)
  s2) loop: speed=-0.5 every 200ms for 5 sec (gentler brake)

Deadband search — find the smallest target the firmware honors:
  v1) speed=+0.01 via sub-cmd 0x01 our-indexed (very slow — should hold near 0)
  v2) speed=+0.05
  v3) speed=+0.1
  v4) speed=+0.5
  v5) speed=+1.0

Firmware probes (read-only — won't move the motor):
  ?sys)  scan SYS func 0x00 sub-cmds 0x00..0x0F looking for a version reply
  ?<HH SS>) send func 0xHH sub-cmd 0xSS once and print response
            example: `?03 0A` probes MOTOR func with sub-cmd 0x0A
  ?listen [SEC])  passively capture and parse all incoming frames for SEC
                  seconds (default 10). Shows func/sub/data for every valid
                  frame the firmware spontaneously emits.

Other:
  m N) switch to motor port N (1-4)
  q)   quit
=============================================
"""


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115_200)
    p.add_argument("--motor", type=int, default=1, choices=[1, 2, 3, 4])
    p.add_argument("--spinup", type=float, default=SPINUP_RPS,
                   help=f"spin-up speed in r/s (default {SPINUP_RPS})")
    args = p.parse_args()

    rrc = HiwonderRRC(port=args.port, baudrate=args.baud)
    print(f"Opening {args.port} at {args.baud} baud...")
    rrc.open()
    print("Port open. Stop polyflow-system-manager first if it's running.\n")

    motor = args.motor
    spinup = args.spinup
    last_spinup = None

    try:
        while True:
            doc_idx = motor              # 1-indexed per PDF
            our_idx = motor - 1          # 0-indexed (current adapter convention)
            doc_bit = motor - 1
            mask_doc = 1 << doc_bit
            mask_alt = 1 << motor        # off-by-one variant

            print(MENU.format(
                motor=motor, doc_idx=doc_idx, our_idx=our_idx,
                mask_doc=mask_doc, mask_alt=mask_alt, doc_bit=doc_bit,
                spinup=spinup,
            ))
            choice = input(f"motor {motor} > ").strip().lower()

            if not choice:
                continue
            if choice == "q":
                break

            if choice.startswith("m "):
                try:
                    motor = int(choice.split()[1])
                    assert 1 <= motor <= 4
                except (IndexError, ValueError, AssertionError):
                    print("Usage: m <1-4>")
                continue

            if choice == "1":
                last_spinup = speed_single(doc_idx, spinup)
                send(rrc, last_spinup, f"spin-up sub-cmd 0x00 doc-indexed, +{spinup} r/s")
            elif choice == "2":
                last_spinup = speed_single(our_idx, spinup)
                send(rrc, last_spinup, f"spin-up sub-cmd 0x00 our-indexed, +{spinup} r/s")
            elif choice == "3":
                last_spinup = speed_multi([(doc_idx, spinup)])
                send(rrc, last_spinup, f"spin-up sub-cmd 0x01 doc-indexed, +{spinup} r/s")
            elif choice == "4":
                last_spinup = speed_multi([(our_idx, spinup)])
                send(rrc, last_spinup, f"spin-up sub-cmd 0x01 our-indexed, +{spinup} r/s")
            elif choice == "r":
                if last_spinup is None:
                    print("No spin-up to repeat. Try [1] or [2] first.")
                else:
                    send(rrc, last_spinup, "(repeat spin-up)")
            elif choice == "a":
                send(rrc, speed_single(doc_idx, 0.0), "speed=0, doc-indexed")
            elif choice == "b":
                send(rrc, speed_single(our_idx, 0.0), "speed=0, our-indexed")
            elif choice == "c":
                send(rrc, stop_single(doc_idx), f"stop sub-cmd 0x02, byte=0x{doc_idx:02X} (doc)")
            elif choice == "d":
                send(rrc, stop_single(our_idx), f"stop sub-cmd 0x02, byte=0x{our_idx:02X} (our)")
            elif choice == "e":
                send(rrc, stop_mask(mask_doc), f"stop mask 0x{mask_doc:02X} (doc bit)")
            elif choice == "f":
                send(rrc, stop_mask(mask_alt), f"stop mask 0x{mask_alt:02X} (off-by-one)")
            elif choice == "g":
                send(rrc, pwm_single(our_idx, 0), "pwm=0, sub-cmd 0x04 raw PWM (our-indexed)")
            elif choice == "j":
                send(rrc, speed_single(doc_idx, 0.0001), "speed=0.0001 (epsilon)")
            elif choice == "h":
                send(rrc, speed_single(doc_idx, -3.0), "step 1: speed=-3 r/s (PID brake)")
                time.sleep(0.2)
                send(rrc, stop_single(doc_idx), "step 2: sub-cmd 0x02 stop")
            elif choice == "i":
                send(rrc, speed_single(doc_idx, -1.0), "step 1: speed=-1 r/s (PID brake)")
                time.sleep(0.2)
                send(rrc, stop_mask(mask_doc), f"step 2: stop mask 0x{mask_doc:02X}")
            elif choice == "k":
                send(rrc, stop_mask(0x0F), "stop mask 0x0F (all motors)")
            elif choice == "n":
                send(rrc, speed_multi([(doc_idx, 0.0)]),
                     "speed=0, sub-cmd 0x01 doc-indexed (production form)")
            elif choice == "o":
                send(rrc, speed_multi([(our_idx, 0.0)]),
                     "speed=0, sub-cmd 0x01 our-indexed (production form)")
            elif choice == "s":
                send(rrc, speed_multi([(our_idx, -1.0)]),
                     "speed=-1, sub-cmd 0x01 our-indexed (PID brake, single packet)")
            elif choice in ("s5", "s2"):
                rate = -1.0 if choice == "s5" else -0.5
                print(f"  looping speed={rate} every 200ms for 5 sec...")
                for _ in range(25):
                    send(rrc, speed_multi([(our_idx, rate)]),
                         f"  loop tick speed={rate}")
                    time.sleep(0.2)
                print("  loop done.")
            elif choice == "?sys":
                print("  scanning SYS (func=0x00) sub-cmds 0x00..0x0F:")
                for sub in range(0x10):
                    print(f"  -- sub-cmd 0x{sub:02X} --")
                    probe(rrc, 0x00, [sub])
            elif choice.startswith("?listen"):
                parts = choice.split()
                seconds = float(parts[1]) if len(parts) > 1 else 10.0
                listen(rrc, seconds=seconds)
            elif choice.startswith("?"):
                # ?HH SS form
                try:
                    parts = choice[1:].strip().split()
                    func = int(parts[0], 16)
                    sub = int(parts[1], 16)
                except (IndexError, ValueError):
                    print("Usage: ?<HH> <SS>  (e.g. '?00 04' probes SYS battery)")
                    continue
                probe(rrc, func, [sub])
            elif choice in ("v1", "v2", "v3", "v4", "v5"):
                target = {"v1": 0.01, "v2": 0.05, "v3": 0.1, "v4": 0.5, "v5": 1.0}[choice]
                send(rrc, speed_multi([(our_idx, target)]),
                     f"deadband search: speed={target} via sub-cmd 0x01 our-indexed")
            else:
                print("?")
    finally:
        print("\nClosing port.")
        rrc.close()


if __name__ == "__main__":
    main()
