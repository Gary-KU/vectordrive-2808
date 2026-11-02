param([string]$Python = (Join-Path $PSScriptRoot '..\.venv-can\Scripts\python.exe'),
      [string]$OutputName = 'GimbalCAN-Studio')
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
Push-Location $root
try {
    & $Python tools/generate_parameters.py --check
    if ($LASTEXITCODE -ne 0) { throw 'Run .\configure.ps1 before packaging.' }
    & $Python -m PyInstaller --noconfirm --windowed --onedir --name $OutputName `
        --distpath dist --workpath build/desktop --specpath build/desktop `
        --paths $root --hidden-import can.interfaces.pcan --hidden-import can.interfaces.slcan `
        --hidden-import can.interfaces.socketcan --hidden-import can.interfaces.usb2can `
        --hidden-import can.interfaces.canalystii --collect-data can desktop/launch.py
    if ($LASTEXITCODE -ne 0) { throw "Desktop packaging failed: $LASTEXITCODE" }
} finally { Pop-Location }
