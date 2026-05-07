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


# Sub-command bytes from the official RRCLite protocol PDF.
SUB_SPEED_SINGLE = 0x00   # one motor, signed float r/s
SUB_SPEED_MULTI  = 0x01   # N motors
SUB_STOP_SINGLE  = 0x02   # halt one motor
SUB_STOP_MASK    = 0x03   # halt N motors (bit N of mask = motor N+1)
SUB_DUTY         = 0x05   # NOT in official spec; came from fast-hiwonder

SPINUP_RPS = 5.0


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


def duty_single(motor_byte, duty_pct):
    return [SUB_DUTY, 1, motor_byte] + list(struct.pack("<f", float(duty_pct)))


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
  g)  duty=0,   sub-cmd 0x05  (undocumented)
  j)  speed=0.0001, sub-cmd 0x00, doc-indexed (epsilon trick)

Active brake (combo):
  h)  speed=-3 r/s, then sub-cmd 0x02 stop after 200ms
  i)  speed=-1 r/s, then sub-cmd 0x03 mask  after 200ms

Multi:
  k)  ALL motors stop, mask=0x0F

Other:
  m N) switch to motor port N (1-4)
  q)   quit
=============================================
"""


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=1_000_000)
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
                send(rrc, duty_single(doc_idx, 0.0), "duty=0, undocumented sub-cmd 0x05")
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
            else:
                print("?")
    finally:
        print("\nClosing port.")
        rrc.close()


if __name__ == "__main__":
    main()
