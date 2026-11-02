"""The JSON catalog, generated bindings and all host consumers stay consistent."""
import contextlib
import copy
from dataclasses import FrozenInstanceError
import io
import json
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from desktop.transports import SimulatorTransport
from examples import gimbal_can as codec
from examples import parameter_catalog as catalog
from tools import generate_parameters as generator


ROOT = Path(__file__).resolve().parents[1]


class ParameterCatalogTests(unittest.TestCase):
    def setUp(self):
        self.document = json.loads((ROOT / "config/parameters.json").read_text(encoding="utf-8"))
        self.records = self.document["parameters"]

    def test_generated_outputs_are_current(self):
        self.assertEqual(generator.generate(ROOT, check=True), [])

    def test_public_python_metadata_and_firmware_bindings_match_schema(self):
        self.assertEqual(len(self.records), 9)
        self.assertIsInstance(catalog.PARAMETER_SPECS, tuple)
        self.assertIs(codec.Param, catalog.Param)
        self.assertIs(codec.PARAMETERS, catalog.PARAMETERS)
        self.assertEqual(SimulatorTransport.DEFAULT_PARAMETERS, catalog.DEFAULT_PARAMETERS)
        c_catalog = (ROOT / "software/Config/parameter_catalog.def").read_text(encoding="utf-8")
        c_records = re.findall(r"^FOC_PARAMETER\((\w+), (\d+), (\w+), (\w+), ([^,]+), ([^,]+), ([01])\)$",
                               c_catalog, re.MULTILINE)
        self.assertEqual(len(c_records), len(self.records))
        c_defaults = dict(re.findall(r"^#define (MOTOR_\w+) (\S+)$",
            (ROOT / "software/Config/motor_tuning_defaults.h").read_text(encoding="utf-8"), re.MULTILINE))
        for row, spec, c_row in zip(self.records, catalog.PARAMETER_SPECS, c_records):
            with self.subTest(parameter=row["name"]):
                self.assertEqual(catalog.Param[row["symbol"]], row["id"])
                for key in ("id", "name", "label", "group", "unit", "default", "step", "description", "invalidates_zero"):
                    self.assertEqual(getattr(spec, key), row[key])
                    self.assertEqual(catalog.PARAMETERS[row["id"]][key], row[key])
                host_maximum = row.get("host_max", row["max"])
                self.assertEqual((spec.minimum, spec.maximum), (row["min"], host_maximum))
                self.assertEqual((catalog.PARAMETERS[row["id"]]["min"], catalog.PARAMETERS[row["id"]]["max"]),
                                 (row["min"], host_maximum))
                self.assertEqual(c_row[:4], (row["symbol"], str(row["id"]), row["field"], row["default_macro"]))
                self.assertEqual(float(c_row[4][:-1]), row["min"])
                if "firmware_max_macro" in row:
                    self.assertEqual(c_row[5], row["firmware_max_macro"])
                else:
                    self.assertEqual(float(c_row[5][:-1]), row["max"])
                self.assertEqual(int(c_row[6]), int(row["invalidates_zero"]))
                self.assertEqual(float(c_defaults[row["default_macro"]][:-1]), row["default"])
        self.assertEqual([spec.id for spec in catalog.PARAMETER_SPECS if spec.invalidates_zero], [9])
        with self.assertRaises(FrozenInstanceError):
            catalog.PARAMETER_SPECS[0].default = 2.0

    def test_wire_ids_defaults_and_other_ranges_remain_compatible(self):
        expected = ((1, "current_kp", 0, 5, 1), (2, "current_ki", 0, 2000, 1000),
                    (3, "iq_limit", 0.01, 3.3, 1), (4, "voltage_limit", 0.1, 5, 5),
                    (5, "iq_slew", 0.01, 10, 1), (6, "speed_kp", 0, 1, 0.03),
                    (7, "speed_ki", 0, 10, 0.1), (8, "speed_limit", 0.1, 30, 30),
                    (9, "bus_voltage", 8, 30, 14))
        actual = tuple((p.id, p.name, p.minimum, p.maximum, p.default) for p in catalog.PARAMETER_SPECS)
        self.assertEqual(actual, expected)
        simulator = SimulatorTransport()
        try:
            for p in catalog.PARAMETER_SPECS:
                self.assertEqual(simulator.parameters[p.id], struct.unpack("<f", struct.pack("<f", p.default))[0])
                for value in (p.minimum, p.default, p.maximum):
                    identifier, payload = codec.pack_param_write(1, 7, p.id, value)
                    self.assertEqual(identifier, 0x201)
                    self.assertEqual(payload[:4], bytes((0xA1, 0x11, 7, p.id)))
                with self.assertRaises(ValueError):
                    codec.pack_param_write(1, 7, p.id, p.maximum + p.step)
        finally:
            simulator.shutdown()

    def test_generation_is_deterministic_and_check_never_rewrites(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "config").mkdir()
            (root / "config/parameters.json").write_text(json.dumps(self.document, ensure_ascii=False), encoding="utf-8")
            expected_paths = set(generator.render_outputs(self.records))
            self.assertEqual(set(generator.generate(root, check=True)), expected_paths)
            self.assertFalse((root / "examples").exists())
            generator.generate(root)
            first = {path: (root / path).read_bytes() for path in expected_paths}
            generator.generate(root)
            self.assertEqual(first, {path: (root / path).read_bytes() for path in expected_paths})
            self.assertEqual(generator.generate(root, check=True), [])
            changed = root / "examples/parameter_catalog.py"
            changed.write_text("stale\n", encoding="utf-8")
            with patch.object(generator, "ROOT", root), contextlib.redirect_stderr(io.StringIO()) as errors:
                self.assertEqual(generator.main(["--check"]), 1)
            self.assertIn("examples/parameter_catalog.py", errors.getvalue())
            self.assertEqual(changed.read_text(encoding="utf-8"), "stale\n")

    def test_host_max_only_changes_python_range_and_keeps_profile_cap_and_defaults(self):
        iq = self.records[2]
        self.assertEqual((iq["id"], iq["max"], iq["host_max"], iq["default"]), (3, 3.3, 3.3, 1))
        widened_host = copy.deepcopy(self.records)
        widened_host[2]["host_max"] = 4.0
        widened = generator.render_outputs(widened_host)
        baseline = generator.render_outputs(self.records)
        for path in (Path("software/Config/parameter_catalog.def"),
                     Path("software/Config/motor_tuning_defaults.h")):
            with self.subTest(path=path):
                self.assertEqual(widened[path], baseline[path])
                self.assertEqual(widened[path].encode("utf-8"), (ROOT / path).read_bytes())
        self.assertNotEqual(widened[Path("examples/parameter_catalog.py")],
                            baseline[Path("examples/parameter_catalog.py")])
        self.assertIn("FOC_PARAMETER(IQ_LIMIT, 3, iq_limit_a, MOTOR_CURRENT_LIMIT_A, 0.01f, MOTOR_IQ_REQUEST_MAX_A, 0)",
                      widened[Path("software/Config/parameter_catalog.def")])
        self.assertEqual(catalog.DEFAULT_PARAMETERS[3], 1.0)

    def test_host_max_is_optional_validated_and_does_not_widen_default(self):
        for host_maximum in (3.3, 4.0):
            document = copy.deepcopy(self.document)
            document["parameters"][2]["host_max"] = host_maximum
            self.assertEqual(generator.validate_schema(document)[2]["default"], 1.0)
        document = copy.deepcopy(self.document)
        document["parameters"][2].pop("host_max")
        self.assertEqual(generator.validate_schema(document)[2]["max"], 3.3)
        for invalid in (True, None, "3.3", -1, 0, 0.9999, float("nan"),
                        float("inf"), float("-inf"), 10 ** 1000, 1e-100):
            document = copy.deepcopy(self.document)
            document["parameters"][2]["host_max"] = invalid
            with self.subTest(host_max=str(invalid)[:40]), self.assertRaises(ValueError):
                generator.validate_schema(document)
        document = copy.deepcopy(self.document)
        document["parameters"][2]["default"] = 3.31
        with self.assertRaisesRegex(ValueError, "default must be inside"):
            generator.validate_schema(document)

    def test_only_iq_limit_can_select_the_profile_request_ceiling_macro(self):
        for index, macro in ((0, "MOTOR_IQ_REQUEST_MAX_A"), (2, "MOTOR_OVERCURRENT_TRIP_A"),
                             (2, True), (2, None), (2, "MOTOR_IQ_REQUEST_MAX_A ")):
            document = copy.deepcopy(self.document)
            document["parameters"][index]["firmware_max_macro"] = macro
            with self.subTest(index=index, macro=macro), self.assertRaises(ValueError):
                generator.validate_schema(document)

    def test_schema_rejects_duplicate_ids_names_fields_and_bad_ranges(self):
        changes = [
            (1, "id", 1), (1, "name", "current_kp"), (1, "symbol", "CURRENT_KP"),
            (1, "field", "current_kp_v_per_a"), (1, "default_macro", "MOTOR_CURRENT_KP"),
            (0, "id", True), (0, "id", 0), (0, "default", True),
            (0, "max", float("nan")), (0, "min", float("-inf")),
            (0, "max", 10**1000), (0, "step", 1e-100),
            (0, "max", 0), (0, "default", 6), (0, "step", 0), (0, "step", -1),
            (0, "step", 6), (0, "group", "other"), (0, "symbol", "BAD-SYMBOL"),
            (0, "name", "not a name"), (0, "default_macro", "BAD_MACRO"),
            (0, "label", ""), (0, "description", "two\nlines"),
            (0, "invalidates_zero", 1), (0, "invalidates_zero", True),
            (8, "invalidates_zero", False),
        ]
        for index, key, value in changes:
            document = copy.deepcopy(self.document)
            document["parameters"][index][key] = value
            with self.subTest(index=index, key=key, value=str(value)[:30]), self.assertRaises(ValueError):
                generator.validate_schema(document)
        for mutation in (lambda d: d["parameters"][0].pop("unit"),
                         lambda d: d["parameters"][0].update(extra=True),
                         lambda d: d["parameters"].pop(2),
                         lambda d: d.update(schema_version=True)):
            document = copy.deepcopy(self.document)
            mutation(document)
            with self.assertRaises(ValueError):
                generator.validate_schema(document)

    def test_json_duplicate_keys_and_nonfinite_numbers_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "parameters.json"
            for text in ('{"schema_version":1,"schema_version":1,"parameters":[]}',
                         '{"schema_version":1,"parameters":[NaN]}',
                         '{"schema_version":1,"parameters":[Infinity]}'):
                source.write_text(text, encoding="utf-8")
                with self.assertRaises(ValueError):
                    generator.load_schema(source)

    def test_standalone_codec_runs_outside_repository(self):
        with tempfile.TemporaryDirectory() as folder:
            completed = subprocess.run([sys.executable, "-B", str(ROOT / "examples/gimbal_can.py"),
                "--node", "1", "--seq", "7", "stop"], cwd=folder, text=True,
                encoding="utf-8", capture_output=True, timeout=5)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("201#A100070000000000", completed.stdout)


if __name__ == "__main__":
    unittest.main()
