@echo off
setlocal
cd /d "%~dp0"
if exist "dist\GimbalCAN-Studio-v2\GimbalCAN-Studio-v2.exe" (
  start "" "dist\GimbalCAN-Studio-v2\GimbalCAN-Studio-v2.exe"
  exit /b 0
)
if exist "dist\GimbalCAN-Studio\GimbalCAN-Studio.exe" (
  start "" "dist\GimbalCAN-Studio\GimbalCAN-Studio.exe"
  exit /b 0
)
if exist "dist\GimbalCAN\GimbalCAN.exe" (
  start "" "dist\GimbalCAN\GimbalCAN.exe"
  exit /b 0
)
if exist ".venv-can\Scripts\pythonw.exe" (
  start "" ".venv-can\Scripts\pythonw.exe" -m desktop
  exit /b 0
)
python -m desktop
if errorlevel 1 (
  echo Install Python with Tcl/Tk, then: python -m pip install -r desktop/requirements.txt
  pause
)
