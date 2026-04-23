"""
adapter.json manifest parser.

Each adapter package ships an `adapter.json` at its root declaring the
drivers it provides. system-manager scans these at startup to build a
`driver_name -> class_path` registry.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Union


@dataclass
class DriverEntry:
    """A single driver declared by an adapter package."""

    name: str
    """Driver identifier referenced by hardware.yaml (e.g., "odrive_can")."""

    class_path: str
    """Python class path in the form "module.submodule:ClassName"."""


@dataclass
class Manifest:
    """Parsed contents of an adapter.json file."""

    name: str
    """Package name (informational; does not have to match the Python package)."""

    drivers: List[DriverEntry] = field(default_factory=list)
    """Drivers this package provides."""

    schema_version: int = 1

    @classmethod
    def from_file(cls, path: Union[str, Path]) -> "Manifest":
        """Load and parse an adapter.json file from disk."""
        return cls.from_dict(json.loads(Path(path).read_text()))

    @classmethod
    def from_dict(cls, data: dict) -> "Manifest":
        """Construct a Manifest from a decoded dict."""
        if "name" not in data:
            raise ValueError("adapter.json: 'name' is required")
        return cls(
            schema_version=int(data.get("schema_version", 1)),
            name=str(data["name"]),
            drivers=[
                DriverEntry(name=str(d["name"]), class_path=str(d["class"]))
                for d in data.get("drivers", [])
            ],
        )

    def to_dict(self) -> dict:
        """Serialize back to a plain dict (inverse of from_dict)."""
        return {
            "schema_version": self.schema_version,
            "name": self.name,
            "drivers": [
                {"name": d.name, "class": d.class_path} for d in self.drivers
            ],
        }
