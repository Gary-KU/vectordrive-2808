"""Explicit, stopped-only PCAN recovery. Never called by normal CAN reception."""
from __future__ import annotations

import json
import os
import re
import subprocess
import time


class RecoveryError(RuntimeError):
    pass


def _run(arguments, cancel, timeout=12.0):
    """Keep cancellation responsive while Windows restarts the USB device."""
    process = subprocess.Popen(arguments, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               encoding="utf-8", errors="replace",
                               creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    deadline = time.monotonic() + timeout
    try:
        while True:
            cancel()
            if time.monotonic() >= deadline:
                raise RecoveryError("PCAN 设备恢复超时；请重插 USB-CAN 后重新连接")
            try:
                stdout, stderr = process.communicate(timeout=.1)
                break
            except subprocess.TimeoutExpired:
                continue
        if process.returncode:
            raise RecoveryError("Windows 未能重启 PCAN 设备：" + (stderr or stdout).strip()[:500] +
                                "；请重插 USB-CAN 后重新连接")
        return stdout
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.communicate(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()


def restart_pcan_device(config, cancel, progress):
    """Restart precisely one classic PCAN-USB, never a hub or CMSIS-DAP.

    A second device/channel cannot be mapped reliably with the installed older
    PCAN API. Refuse ambiguity instead of resetting another motor's adapter.
    The controller must close its CAN handle and discard commands first.
    """
    if config.get("interface") != "pcan" or config.get("channel") != "PCAN_USBBUS1":
        raise RecoveryError("设备恢复仅支持 PCAN_USBBUS1；其他通道请重插对应适配器后连接")
    if os.name != "nt":
        raise RecoveryError("PCAN 设备恢复目前仅支持 Windows")
    progress("查找 PCAN-USB")
    # Fixed read-only command; no user text is interpolated into shell code.
    script = r"""
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
$devices = @(Get-PnpDevice -PresentOnly -ErrorAction Stop | Where-Object {
    $_.Class -eq 'CAN-Hardware' -and $_.InstanceId -like 'USB\VID_0C72*'
} | Select-Object -ExpandProperty InstanceId)
ConvertTo-Json -InputObject $devices -Compress
"""
    text = _run(["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script], cancel)
    try:
        devices = json.loads(text.lstrip("\ufeff"))
    except (ValueError, TypeError) as error:
        raise RecoveryError("无法读取 PCAN 设备列表；请重插 USB-CAN 后重新连接") from error
    if not isinstance(devices, list) or len(devices) != 1:
        raise RecoveryError("需要恰好一台 PCAN-USB 才能自动恢复；请检查 USB 连接或手动重插目标适配器")
    device = devices[0]
    if not isinstance(device, str) or not re.fullmatch(r"USB\\VID_0C72&PID_000C\\[A-Za-z0-9&_-]+", device, re.I):
        raise RecoveryError("当前设备不是支持自动恢复的单通道 PCAN-USB；请手动重插目标适配器")
    cancel()
    progress("重启 PCAN-USB")
    _run(["pnputil.exe", "/restart-device", device], cancel)
    cancel()
    return device
