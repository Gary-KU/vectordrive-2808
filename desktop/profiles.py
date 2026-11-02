"""Validated PC-only parameter plans; loading never talks to a controller."""
from __future__ import annotations

import json
import math
import os
import struct
import tempfile
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Mapping

from examples.parameter_catalog import PARAMETER_SPECS


SCHEMA_VERSION = 1
_BY_NAME = {spec.name: spec for spec in PARAMETER_SPECS}


class ProfileError(ValueError):
    """A plan is malformed, unsupported, or outside the parameter contract."""


def float32_equal(left, right):
    """Compare values as firmware float32, including equivalent +/- zero."""
    try:
        if isinstance(left, bool) or isinstance(right, bool):
            return False
        a, b = float(left), float(right)
        if not math.isfinite(a) or not math.isfinite(b):
            return False
        return struct.unpack("<f", struct.pack("<f", a))[0] == struct.unpack("<f", struct.pack("<f", b))[0]
    except (TypeError, ValueError, OverflowError, struct.error):
        return False


def _validate_json_value(value, location):
    if value is None or isinstance(value, (str, bool, int)):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ProfileError(f"{location}不能包含 NaN 或 Infinity。")
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            _validate_json_value(child, f"{location}[{index}]")
        return
    if isinstance(value, dict):
        for key, child in value.items():
            if not isinstance(key, str):
                raise ProfileError(f"{location}的键必须是字符串。")
            _validate_json_value(child, f"{location}.{key}")
        return
    raise ProfileError(f"{location}包含不支持的 JSON 值。")


def validate_parameters(parameters):
    if not isinstance(parameters, Mapping) or not parameters:
        raise ProfileError("parameters 必须是至少包含一项的参数对象；允许只列出部分参数。")
    result = {}
    for name, value in parameters.items():
        if not isinstance(name, str) or name not in _BY_NAME:
            raise ProfileError(f"未知参数名称：{name!r}。")
        spec = _BY_NAME[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProfileError(f"{spec.label}必须是数值。")
        try:
            number = float(value)
        except (ValueError, OverflowError) as error:
            raise ProfileError(f"{spec.label}数值无效。") from error
        if not math.isfinite(number) or not spec.minimum <= number <= spec.maximum:
            raise ProfileError(f"{spec.label}必须在 {spec.minimum:g} … {spec.maximum:g} {spec.unit} 范围内。")
        result[name] = number
    return result


@dataclass(frozen=True)
class Profile:
    parameters: dict[str, float]
    motor: str | None = None
    metadata: dict = field(default_factory=dict)

    def to_mapping(self):
        return {_BY_NAME[name].id: value for name, value in self.parameters.items()}

    def to_dict(self):
        result = {"schema_version": SCHEMA_VERSION, "parameters": dict(self.parameters)}
        if self.motor is not None:
            result["motor"] = self.motor
        if self.metadata:
            result["metadata"] = dict(self.metadata)
        return result


def validate_profile(document):
    if not isinstance(document, dict):
        raise ProfileError("方案顶层必须是 JSON 对象。")
    unknown = set(document) - {"schema_version", "motor", "parameters", "metadata"}
    if unknown:
        raise ProfileError("方案包含未知字段：" + ", ".join(sorted(map(str, unknown))))
    if "schema_version" not in document or "parameters" not in document:
        raise ProfileError("方案必须包含 schema_version 和 parameters。")
    if type(document["schema_version"]) is not int or document["schema_version"] != SCHEMA_VERSION:
        raise ProfileError(f"只支持 schema_version={SCHEMA_VERSION}。")
    motor = document.get("motor")
    if "motor" in document and (not isinstance(motor, str) or not motor.strip()):
        raise ProfileError("motor 如提供，必须是非空电机标签字符串。")
    metadata = document.get("metadata", {})
    if not isinstance(metadata, dict):
        raise ProfileError("metadata 必须是 JSON 对象。")
    _validate_json_value(metadata, "metadata")
    return Profile(validate_parameters(document["parameters"]), motor, dict(metadata))


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProfileError(f"JSON 存在重复字段：{key}。")
        result[key] = value
    return result


def _nonfinite_constant(value):
    raise ProfileError(f"JSON 不能包含 {value}。")


def load_profile(path):
    try:
        document = json.loads(Path(path).read_text(encoding="utf-8-sig"),
                              object_pairs_hook=_unique_object,
                              parse_constant=_nonfinite_constant)
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ProfileError(f"读取方案失败：{error}") from error
    return validate_profile(document)


def save_profile(path, parameters, motor=None, metadata=None):
    """Atomically save editor values. It never claims to be a board readback."""
    if metadata is not None and not isinstance(metadata, dict):
        raise ProfileError("metadata 必须是 JSON 对象。")
    details = dict(metadata or {})
    details["source"] = "editor_values"
    details["saved_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
    document = {"schema_version": SCHEMA_VERSION, "parameters": parameters, "metadata": details}
    if motor is not None:
        document["motor"] = motor
    profile = validate_profile(document)
    destination = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="\n",
                                         dir=destination.parent, prefix=".foc-plan-", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(profile.to_dict(), stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.write("\n")
        os.replace(temporary, destination)
    except OSError as error:
        raise ProfileError(f"保存方案失败：{error}") from error
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return profile
