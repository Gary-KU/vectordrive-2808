param([string]$ManifestPath = (Join-Path (Split-Path -Parent $PSScriptRoot) 'software/project.psd1'))

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$manifest = Import-PowerShellDataFile -LiteralPath $ManifestPath
$softwareRoot = Split-Path -Parent (Get-Item -LiteralPath $ManifestPath).FullName
$workspaceRoot = Split-Path -Parent $softwareRoot

$includeArguments = @()
foreach ($relativePath in $manifest.IncludeDirectories) {
    $includePath = Join-Path $softwareRoot $relativePath
    if (-not (Test-Path -LiteralPath $includePath -PathType Container)) {
        throw "Manifest include directory is missing: $includePath"
    }
    $includeArguments += '-I' + $includePath
}

# Role metadata belongs to the same manifest used by GCC and Keil. Tests pick
# the production translation units they exercise without copying their layout.
$sourceRoles = @{}
foreach ($group in $manifest.SourceGroups) {
    if (-not $group.ContainsKey('Roles')) { continue }
    foreach ($role in $group.Roles.GetEnumerator()) {
        if ($sourceRoles.ContainsKey($role.Key)) {
            throw "Duplicate source role in manifest: $($role.Key)"
        }
        $sourcePath = Join-Path (Join-Path $softwareRoot $group.Path) $role.Value
        if (-not (Test-Path -LiteralPath $sourcePath -PathType Leaf)) {
            throw "Manifest source role '$($role.Key)' is missing: $sourcePath"
        }
        $sourceRoles[$role.Key] = $sourcePath
    }
}
foreach ($requiredRole in @('App', 'Console', 'CanProtocol', 'CanService', 'Foc', 'FocParameters')) {
    if (-not $sourceRoles.ContainsKey($requiredRole)) {
        throw "Required host-test source role is absent from manifest: $requiredRole"
    }
}

[pscustomobject]@{
    WorkspaceRoot = $workspaceRoot
    BuildDirectory = Join-Path $workspaceRoot 'build/host-tests'
    IncludeArguments = $includeArguments
    CompilerArguments = @('-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-pedantic')
    Sources = $sourceRoles
}
