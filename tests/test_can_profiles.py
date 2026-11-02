"""PC plans validate fully before they can update an editor or controller."""
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from desktop.profiles import ProfileError, float32_equal, load_profile, save_profile, validate_profile
from examples.parameter_catalog import PARAMETER_SPECS


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "motor.json"
        self.spec = PARAMETER_SPECS[0]

    def document(self, parameters=None):
        return {"schema_version": 1, "parameters": parameters or {self.spec.name: self.spec.default}}

    def write(self, text):
        self.path.write_text(text, encoding="utf-8")
        return self.path

    def test_default_catalog_round_trip_labels_editor_source(self):
        defaults = {spec.name: spec.default for spec in PARAMETER_SPECS}
        saved = save_profile(self.path, defaults, motor="4015 / 14 V",
                             metadata={"note": "仅保存编辑器数值", "source": "board_actual"})
        loaded = load_profile(self.path)
        self.assertEqual(loaded, saved)
        self.assertEqual(loaded.motor, "4015 / 14 V")
        self.assertEqual(loaded.metadata["source"], "editor_values")
        self.assertTrue(loaded.metadata["saved_at"])
        self.assertEqual(set(loaded.to_mapping()), {spec.id for spec in PARAMETER_SPECS})
        self.assertEqual(json.loads(self.path.read_text(encoding="utf-8"))["schema_version"], 1)

    def test_partial_profile_does_not_add_or_write_other_parameters(self):
        profile = validate_profile(self.document())
        self.assertEqual(profile.to_mapping(), {self.spec.id: float(self.spec.default)})
        self.assertEqual(len(profile.parameters), 1)
        self.assertIsNone(profile.motor)

    def test_all_parameter_boundaries(self):
        for spec in PARAMETER_SPECS:
            for value in (spec.minimum, spec.maximum):
                with self.subTest(name=spec.name, value=value):
                    profile = validate_profile(self.document({spec.name: value}))
                    self.assertEqual(profile.to_mapping()[spec.id], value)
            for value in (spec.minimum - max(1e-6, abs(spec.minimum) * 1e-6),
                          spec.maximum + max(1e-6, abs(spec.maximum) * 1e-6)):
                with self.subTest(name=spec.name, rejected=value):
                    with self.assertRaises(ProfileError):
                        validate_profile(self.document({spec.name: value}))

    def test_iq_limit_profile_supports_host_capacity_without_changing_default(self):
        profile = validate_profile(self.document({"iq_limit": 3.3}))
        self.assertEqual(profile.to_mapping(), {3: 3.3})
        self.assertEqual(next(spec.default for spec in PARAMETER_SPECS if spec.id == 3), 1.0)
        with self.assertRaises(ProfileError):
            validate_profile(self.document({"iq_limit": 3.3001}))

    def test_missing_required_fields_unknown_fields_and_versions(self):
        cases = ({}, {"schema_version": 1}, {"parameters": {self.spec.name: 1}},
                 dict(self.document(), extra=0), dict(self.document(), schema_version=2),
                 dict(self.document(), schema_version=True), dict(self.document(), schema_version=1.0),
                 dict(self.document(), parameters={}), dict(self.document(), parameters=[]),
                 [], "text")
        for document in cases:
            with self.subTest(document=document), self.assertRaises(ProfileError):
                validate_profile(document)

    def test_unknown_parameters_and_non_numeric_values(self):
        cases = ({"unrecognized_gain": 1}, {1: 1}, {self.spec.name: True},
                 {self.spec.name: "1"}, {self.spec.name: None},
                 {self.spec.name: math.nan}, {self.spec.name: math.inf},
                 {self.spec.name: -math.inf}, {self.spec.name: 10 ** 1000})
        for parameters in cases:
            with self.subTest(parameters=parameters), self.assertRaises(ProfileError):
                validate_profile(self.document(parameters))

    def test_duplicate_keys_at_each_level_are_rejected(self):
        name = self.spec.name
        cases = ('{"schema_version":1,"schema_version":1,"parameters":{"%s":1}}' % name,
                 '{"schema_version":1,"parameters":{"%s":1,"%s":2}}' % (name, name),
                 '{"schema_version":1,"parameters":{"%s":1},"metadata":{"note":"a","note":"b"}}' % name)
        for text in cases:
            with self.subTest(text=text), self.assertRaises(ProfileError):
                load_profile(self.write(text))

    def test_nonfinite_json_constants_and_overflow_rejected(self):
        for value in ("NaN", "Infinity", "-Infinity", "1e999"):
            with self.subTest(value=value), self.assertRaises(ProfileError):
                load_profile(self.write('{"schema_version":1,"parameters":{"%s":%s}}' % (self.spec.name, value)))
        document = dict(self.document(), metadata={"nested": [0, {"bad": math.nan}]})
        with self.assertRaises(ProfileError):
            validate_profile(document)

    def test_optional_labels_and_metadata_types(self):
        for values in ({"motor": ""}, {"motor": None}, {"motor": 4015},
                       {"metadata": []}, {"metadata": {"bad": object()}}):
            with self.subTest(values=values), self.assertRaises(ProfileError):
                validate_profile(dict(self.document(), **values))
        for motor in ("", 4015):
            with self.subTest(motor=motor), self.assertRaises(ProfileError):
                save_profile(self.path, self.document()["parameters"], motor=motor)

    def test_failed_validation_preserves_existing_file(self):
        self.path.write_text("existing plan", encoding="utf-8")
        with self.assertRaises(ProfileError):
            save_profile(self.path, {self.spec.name: math.nan})
        self.assertEqual(self.path.read_text(encoding="utf-8"), "existing plan")
        self.assertEqual(list(Path(self.directory.name).glob(".foc-plan-*")), [])

    def test_malformed_or_missing_file_does_not_return_partial_plan(self):
        with self.assertRaises(ProfileError):
            load_profile(self.path)
        for text in ("{", "null", "[1,2]", '{"schema_version":1,"parameters":{"%s":1,"unknown":2}}' % self.spec.name):
            with self.subTest(text=text), self.assertRaises(ProfileError):
                load_profile(self.write(text))
        self.path.write_bytes(b"\xff\xfe\xff")
        with self.assertRaises(ProfileError):
            load_profile(self.path)

    def test_utf8_bom_and_unicode_label_are_supported(self):
        self.path.write_text(json.dumps(dict(self.document(), motor="云台4015"), ensure_ascii=False), encoding="utf-8-sig")
        self.assertEqual(load_profile(self.path).motor, "云台4015")

    def test_float32_comparison_avoids_false_modified_state(self):
        rounded = struct.unpack("<f", struct.pack("<f", 0.03))[0]
        self.assertTrue(float32_equal(0.03, rounded))
        self.assertTrue(float32_equal(0.0, -0.0))
        self.assertFalse(float32_equal(0.03, 0.031))
        for bad in (math.nan, math.inf, True, None, 1e300):
            self.assertFalse(float32_equal(bad, 0))


if __name__ == "__main__":
    unittest.main()
