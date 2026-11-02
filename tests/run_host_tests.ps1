param([string]$Compiler = 'gcc',
      [ValidateSet('2808','4015')][string]$Motor = '2808')
$ErrorActionPreference = 'Stop'
$project = & (Join-Path $PSScriptRoot 'project_paths.ps1')
$buildPath = $project.BuildDirectory
New-Item -ItemType Directory -Force -Path $buildPath | Out-Null
$executablePath = Join-Path $buildPath 'test_foc.exe'
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
$commonArguments = $project.CompilerArguments + $project.IncludeArguments
$commonArguments += '-DMOTOR_MODEL=' + $Motor
& $compilerPath @commonArguments $project.Sources.Foc `
    (Join-Path $PSScriptRoot 'test_foc.c') '-lm' '-o' $executablePath
if ($LASTEXITCODE -ne 0) { throw "Host compilation failed: $LASTEXITCODE" }
& $executablePath
if ($LASTEXITCODE -ne 0) { throw "Host test failed: $LASTEXITCODE" }

foreach ($parameterVariant in @('default','profile')) {
    $parameterDefinitions = @()
    if ($parameterVariant -eq 'profile') {
        $parameterDefinitions = @('-DMOTOR_CURRENT_KI=1500.0f',
            '-DMOTOR_CURRENT_LIMIT_A=0.1f','-DMOTOR_VOLTAGE_LIMIT_V=1.0f')
    }
    $parameterExecutable = Join-Path $buildPath ('test_foc_parameters_' + $parameterVariant + '.exe')
    & $compilerPath @commonArguments @parameterDefinitions $project.Sources.FocParameters `
        (Join-Path $PSScriptRoot 'test_foc_parameters.c') '-lm' '-o' $parameterExecutable
    if ($LASTEXITCODE -ne 0) { throw "Parameter catalog $parameterVariant compilation failed: $LASTEXITCODE" }
    & $parameterExecutable
    if ($LASTEXITCODE -ne 0) { throw "Parameter catalog $parameterVariant tests failed: $LASTEXITCODE" }
}
