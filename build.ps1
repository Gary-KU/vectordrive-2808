[CmdletBinding()]
param(
    [string]$Configuration = 'Release',
    [string]$Toolchain = '',
    [ValidateSet('2808', '4015')]
    [string]$Motor = '2808',
    [switch]$Test
)

# Keep the software/ entry point usable while offering one command at repo root.
# Hashtable splatting preserves named parameters, switches, and paths with spaces.
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot 'configure.ps1')
& (Join-Path $PSScriptRoot 'software\build.ps1') @PSBoundParameters
