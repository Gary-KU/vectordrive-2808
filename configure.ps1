[CmdletBinding()]
param(
    [string]$Python = '',
    [switch]$Check
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
if (-not $Python) {
    $localPython = Join-Path $PSScriptRoot '.venv-can\Scripts\python.exe'
    if (Test-Path -LiteralPath $localPython -PathType Leaf) {
        $Python = $localPython
    } else {
        $Python = (Get-Command python -ErrorAction Stop).Source
    }
}
$generatorArguments = @((Join-Path $PSScriptRoot 'tools\generate_parameters.py'))
if ($Check) { $generatorArguments += '--check' }
& $Python @generatorArguments
if ($LASTEXITCODE -ne 0) {
    throw 'Parameter catalog generation/check failed; see the error above.'
}
if ($Check) { Write-Host 'Parameter catalog: up to date.' }
else { Write-Host 'Parameter catalog: generated C/Python definitions.' }
