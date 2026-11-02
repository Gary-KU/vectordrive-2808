[CmdletBinding()]
param()

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectFile = Join-Path $PSScriptRoot 'GimbalFOC.uvprojx'
$firmwareRoot = Split-Path -Parent $PSScriptRoot
$project = Import-PowerShellDataFile -LiteralPath (Join-Path $firmwareRoot 'project.psd1')
[xml]$projectXml = Get-Content -LiteralPath $projectFile -Raw

function Add-SourceGroup {
    param([string]$Name, [System.IO.FileInfo[]]$Sources)
    if ($Sources.Count -eq 0) { return }
    $group = $projectXml.CreateElement('Group')
    $groupName = $projectXml.CreateElement('GroupName')
    $groupName.InnerText = $Name
    [void]$group.AppendChild($groupName)
    $files = $projectXml.CreateElement('Files')
    foreach ($source in $Sources) {
        $fileNode = $projectXml.CreateElement('File')
        $relative = '..\' + $source.FullName.Substring($firmwareRoot.Length + 1)
        foreach ($field in @('FileName', 'FileType', 'FilePath')) {
            $element = $projectXml.CreateElement($field)
            switch ($field) {
                'FileName' { $element.InnerText = $source.Name }
                'FileType' {
                    switch ($source.Extension) {
                        '.s' { $element.InnerText = '2' }
                        '.c' { $element.InnerText = '1' }
                        default { $element.InnerText = '5' }
                    }
                }
                'FilePath' { $element.InnerText = $relative }
            }
            [void]$fileNode.AppendChild($element)
        }
        [void]$files.AppendChild($fileNode)
    }
    [void]$group.AppendChild($files)
    [void]$groups.AppendChild($group)
}

foreach ($target in @($projectXml.Project.Targets.Target)) {
    $includePaths = @($project.IncludeDirectories | ForEach-Object { '..\' + $_.Replace('/', '\') }) -join ';'
    $target.TargetOption.TargetArmAds.Cads.VariousControls.IncludePath = $includePaths
    $target.TargetOption.TargetArmAds.Aads.VariousControls.IncludePath = $includePaths
    $target.TargetOption.TargetArmAds.LDads.ScatterFile = '..\' + $project.Linker.Armcc.Replace('/', '\')
    $groups = $target.SelectSingleNode('Groups')
    $groups.RemoveAll()
    foreach ($sourceGroup in $project.SourceGroups) {
        $sourceDir = Join-Path $firmwareRoot $sourceGroup.Path
        if (-not (Test-Path -LiteralPath $sourceDir -PathType Container)) {
            throw "Project source directory not found: $sourceDir"
        }
        $sources = @(Get-ChildItem -LiteralPath $sourceDir -Filter '*.c' -File -Recurse | Sort-Object FullName)
        if ($sourceGroup.Name -eq 'Core') {
            $sources = @((Get-Item -LiteralPath (Join-Path $firmwareRoot $project.Startup.Armcc))) + $sources
            $sources += Get-Item -LiteralPath (Join-Path $firmwareRoot $project.Linker.Armcc)
        }
        Add-SourceGroup $sourceGroup.Name $sources
    }
    foreach ($headerGroup in $project.HeaderGroups) {
        $headerDir = Join-Path $firmwareRoot $headerGroup.Path
        Add-SourceGroup $headerGroup.Name @(Get-ChildItem -LiteralPath $headerDir -File -Recurse |
            Where-Object { $_.Extension -in @('.h','.def') } | Sort-Object FullName)
    }
}
$settings = New-Object System.Xml.XmlWriterSettings
$settings.Indent = $true
$settings.Encoding = New-Object System.Text.UTF8Encoding($false)
$writer = [System.Xml.XmlWriter]::Create($projectFile, $settings)
try { $projectXml.Save($writer) } finally { $writer.Dispose() }
Write-Host "Updated source groups: $projectFile"
