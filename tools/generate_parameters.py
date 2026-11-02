#!/usr/bin/env python3
"""Generate committed Python/C parameter catalogs from the sole JSON schema.

Run normally to regenerate, or with --check for a read-only stale-file check.
Only Python's standard library is required; firmware builds use generated C.
An optional host_max widens Python encoding/editor metadata only. The firmware
catalog uses max, or an explicit IQ_LIMIT firmware_max_macro whose profile
selects the board request ceiling. Defaults always retain the schema default.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = Path("config/parameters.json")
FIELDS = {"id", "symbol", "name", "label", "group", "field", "default_macro",
          "min", "max", "default", "step", "unit", "description", "invalidates_zero"}
OPTIONAL_FIELDS = {"host_max", "firmware_max_macro"}


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: %s" % key)
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError("non-finite JSON number: %s" % value)


def validate_schema(document):
    if not isinstance(document, dict) or set(document) != {"schema_version", "parameters"}:
        raise ValueError("root must contain exactly schema_version and parameters")
    if type(document["schema_version"]) is not int or document["schema_version"] != 1:
        raise ValueError("unsupported schema_version (expected integer 1)")
    records = document["parameters"]
    if not isinstance(records, list) or not records:
        raise ValueError("parameters must be a non-empty array")
    seen = {key: set() for key in ("id", "symbol", "name", "field", "default_macro")}
    for index, record in enumerate(records, 1):
        prefix = "parameter %d: " % index
        if not isinstance(record, dict) or not FIELDS <= set(record) <= FIELDS | OPTIONAL_FIELDS:
            raise ValueError(prefix + "missing or unknown fields")
        if type(record["id"]) is not int or not 1 <= record["id"] <= 255:
            raise ValueError(prefix + "id must be an integer in 1..255")
        for key in ("symbol", "name", "label", "group", "field", "default_macro", "unit", "description"):
            text = record[key]
            if not isinstance(text, str) or not text.strip() or text != text.strip() or any(ord(c) < 32 for c in text):
                raise ValueError(prefix + key + " must be a non-empty, trimmed, single-line string")
        for key, pattern in (("symbol", r"[A-Z][A-Z0-9_]*"),
                             ("name", r"[a-z][a-z0-9_]*"),
                             ("field", r"[a-z][a-z0-9_]*"),
                             ("default_macro", r"MOTOR_[A-Z0-9_]+")):
            if not re.fullmatch(pattern, record[key]):
                raise ValueError(prefix + "invalid " + key)
        if record["group"] not in ("current", "speed", "system"):
            raise ValueError(prefix + "unknown group")
        if type(record["invalidates_zero"]) is not bool:
            raise ValueError(prefix + "invalidates_zero must be boolean")
        if "firmware_max_macro" in record:
            supported_firmware_maxima = {
                (3, "IQ_LIMIT"): "MOTOR_IQ_REQUEST_MAX_A",
            }
            if supported_firmware_maxima.get((record["id"], record["symbol"])) != record["firmware_max_macro"]:
                raise ValueError(prefix + "unsupported firmware_max_macro")
        if record["invalidates_zero"] != (record["name"] == "bus_voltage"):
            raise ValueError(prefix + "only bus_voltage must invalidate electrical zero")
        numeric_fields = ("min", "max", "default", "step")
        if "host_max" in record:
            numeric_fields += ("host_max",)
        for key in numeric_fields:
            value = record[key]
            if type(value) not in (int, float):
                raise ValueError(prefix + key + " must be a finite number")
            try:
                if not math.isfinite(value):
                    raise ValueError(prefix + key + " must be a finite number")
                binary32 = struct.unpack("<f", struct.pack("<f", value))[0]
            except (OverflowError, struct.error) as error:
                raise ValueError(prefix + key + " must fit float32") from error
            if not math.isfinite(binary32) or (value != 0 and binary32 == 0):
                raise ValueError(prefix + key + " must fit non-underflowing float32")
        if not record["min"] < record["max"]:
            raise ValueError(prefix + "min must be less than max")
        if "host_max" in record and record["host_max"] < record["max"]:
            raise ValueError(prefix + "host_max must be at least max")
        if not struct.unpack("<f", struct.pack("<f", record["min"]))[0] < struct.unpack("<f", struct.pack("<f", record["max"]))[0]:
            raise ValueError(prefix + "min and max must remain distinct in float32")
        if not record["min"] <= record["default"] <= record["max"]:
            raise ValueError(prefix + "default must be inside the permitted range")
        if not 0 < record["step"] <= record["max"] - record["min"]:
            raise ValueError(prefix + "step must be positive and no larger than the range")
        for key, values in seen.items():
            if record[key] in values:
                raise ValueError(prefix + "duplicate " + key)
            values.add(record[key])
    if [record["id"] for record in records] != list(range(1, len(records) + 1)):
        raise ValueError("IDs must be contiguous and ordered from 1 for firmware table indexing")
    return records


def load_schema(path):
    with Path(path).open(encoding="utf-8") as stream:
        document = json.load(stream, object_pairs_hook=_object, parse_constant=_reject_constant)
    return validate_schema(document)


def _c_float(value):
    text = repr(float(value))
    return text + "f"


def render_outputs(records):
    banner = "Generated from config/parameters.json by tools/generate_parameters.py; do not edit."
    python = ['"""' + banner + '"""', "from dataclasses import dataclass", "from enum import IntEnum", "", "",
              "class Param(IntEnum):"]
    python += ["    %s = %d" % (record["symbol"], record["id"]) for record in records]
    python += ["", "", "@dataclass(frozen=True)", "class ParameterSpec:",
               "    id: int", "    name: str", "    label: str", "    group: str", "    unit: str",
               "    minimum: float", "    maximum: float", "    default: float", "    step: float",
               "    description: str", "    invalidates_zero: bool", "", "", "PARAMETER_SPECS = ("]
    for record in records:
        host_record = dict(record, max=record.get("host_max", record["max"]))
        values = [host_record[key] for key in ("id", "name", "label", "group", "unit", "min", "max", "default", "step", "description", "invalidates_zero")]
        python.append("    ParameterSpec(" + ", ".join(repr(value) for value in values) + "),")
    python += [")", "", "PARAMETERS = {", "    spec.id: {", "        'id': spec.id, 'name': spec.name, 'label': spec.label, 'group': spec.group,",
               "        'unit': spec.unit, 'min': spec.minimum, 'max': spec.maximum,",
               "        'default': spec.default, 'step': spec.step, 'description': spec.description,",
               "        'invalidates_zero': spec.invalidates_zero,", "    }", "    for spec in PARAMETER_SPECS", "}",
               "DEFAULT_PARAMETERS = {spec.id: spec.default for spec in PARAMETER_SPECS}", ""]
    catalog = ["/* " + banner + " */", "/* Intentionally no include guard: this is a repeatable X-macro list. */"]
    for record in records:
        catalog.append("FOC_PARAMETER(%s, %d, %s, %s, %s, %s, %d)" % (
            record["symbol"], record["id"], record["field"], record["default_macro"],
            _c_float(record["min"]), record.get("firmware_max_macro", _c_float(record["max"])),
            int(record["invalidates_zero"])))
    defaults = ["/* " + banner + " */", "#ifndef GIMBAL_MOTOR_TUNING_DEFAULTS_H",
                "#define GIMBAL_MOTOR_TUNING_DEFAULTS_H", ""]
    for record in records:
        defaults += ["#ifndef " + record["default_macro"],
                     "#define %s %s" % (record["default_macro"], _c_float(record["default"])), "#endif", ""]
    defaults += ["#endif", ""]
    return {Path("examples/parameter_catalog.py"): "\n".join(python),
            Path("software/Config/parameter_catalog.def"): "\n".join(catalog) + "\n",
            Path("software/Config/motor_tuning_defaults.h"): "\n".join(defaults)}


def generate(root=ROOT, *, check=False):
    root = Path(root)
    outputs = render_outputs(load_schema(root / SCHEMA_PATH))
    stale = []
    for relative, source in outputs.items():
        destination = root / relative
        expected = source.encode("utf-8")
        if check:
            if not destination.exists() or destination.read_bytes() != expected:
                stale.append(relative)
        else:
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(expected)
    return stale


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="report missing/stale generated files without writing")
    options = parser.parse_args(argv)
    try:
        stale = generate(ROOT, check=options.check)
    except (OSError, UnicodeError, ValueError) as error:
        print("parameter schema error: %s" % error, file=sys.stderr)
        return 2
    for path in stale:
        print("stale generated file: %s" % path.as_posix(), file=sys.stderr)
    return 1 if stale else 0


if __name__ == "__main__":
    raise SystemExit(main())
