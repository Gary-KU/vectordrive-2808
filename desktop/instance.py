"""Keep one Windows workbench instance so repeated launches cannot steal CAN."""
from __future__ import annotations

import ctypes
import os


class InstanceGuard:
    def __init__(self, kernel=None, handle=None):
        self.kernel, self.handle = kernel, handle

    def close(self):
        if self.handle is not None:
            self.kernel.CloseHandle(self.handle)
            self.handle = None


def acquire_gui_instance(name=r'Local\GimbalCANStudio.Workbench'):
    if os.name != 'nt':
        return InstanceGuard()
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.CreateMutexW.argtypes = (ctypes.c_void_p, ctypes.c_int, ctypes.c_wchar_p)
    kernel.CreateMutexW.restype = ctypes.c_void_p
    kernel.CloseHandle.argtypes = (ctypes.c_void_p,)
    kernel.CloseHandle.restype = ctypes.c_int
    ctypes.set_last_error(0)
    handle = kernel.CreateMutexW(None, False, name)
    if not handle:
        raise OSError(ctypes.get_last_error(), 'Cannot create workbench instance mutex')
    if ctypes.get_last_error() == 183:  # ERROR_ALREADY_EXISTS
        kernel.CloseHandle(handle)
        return None
    return InstanceGuard(kernel, handle)


def restore_existing_gui():
    """Duplicate-launch UI behavior; acquiring/testing a lock never changes focus."""
    if os.name != 'nt':
        return
    user = ctypes.WinDLL('user32', use_last_error=True)
    user.FindWindowW.argtypes = (ctypes.c_wchar_p, ctypes.c_wchar_p)
    user.FindWindowW.restype = ctypes.c_void_p
    user.ShowWindow.argtypes = (ctypes.c_void_p, ctypes.c_int)
    user.SetForegroundWindow.argtypes = (ctypes.c_void_p,)
    window = user.FindWindowW(None, '电机调试 · CAN')
    if window:
        user.ShowWindow(window, 9)
        user.SetForegroundWindow(window)
