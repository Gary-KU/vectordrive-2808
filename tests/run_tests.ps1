param([string]$Compiler = 'gcc',
      [ValidateSet('2808','4015')][string]$Motor = '2808')
$ErrorActionPreference = 'Stop'
$project = & (Join-Path $PSScriptRoot 'project_paths.ps1')
$buildPath = $project.BuildDirectory
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
New-Item -ItemType Directory -Force -Path $buildPath | Out-Null

& (Join-Path $PSScriptRoot 'run_host_tests.ps1') -Compiler $compilerPath -Motor $Motor

$commonArguments = $project.CompilerArguments + $project.IncludeArguments
$commonArguments += '-DMOTOR_MODEL=' + $Motor
$nodeStoreExecutable = Join-Path $buildPath 'test_node_id_store.exe'
& $compilerPath @commonArguments `
    (Join-Path $project.WorkspaceRoot 'software/BSP/Src/node_id_store.c') `
    (Join-Path $PSScriptRoot 'test_node_id_store.c') '-o' $nodeStoreExecutable
if ($LASTEXITCODE -ne 0) { throw "Node ID storage compilation failed: $LASTEXITCODE" }
& $nodeStoreExecutable
if ($LASTEXITCODE -ne 0) { throw "Node ID storage tests failed: $LASTEXITCODE" }
$calibrationStoreExecutable = Join-Path $buildPath 'test_calibration_store.exe'
& $compilerPath @commonArguments `
    (Join-Path $project.WorkspaceRoot 'software/BSP/Src/calibration_store.c') `
    (Join-Path $PSScriptRoot 'test_calibration_store.c') '-lm' '-o' $calibrationStoreExecutable
if ($LASTEXITCODE -ne 0) { throw "Calibration storage compilation failed: $LASTEXITCODE" }
& $calibrationStoreExecutable
if ($LASTEXITCODE -ne 0) { throw "Calibration storage tests failed: $LASTEXITCODE" }
$appSources = @($project.Sources.App, $project.Sources.Console, $project.Sources.Foc, $project.Sources.FocParameters)
$sources = $appSources + (Join-Path $PSScriptRoot 'test_app.c')

foreach ($variant in @('default', 'confirmed', 'shunt_nan', 'shunt_inf', 'gain_nan', 'gain_inf')) {
    # The unconfirmed-board negative case is explicit: production defaults
    # use the now-verified sense conversion and must not be changed for tests.
    $variantArguments = @('-DMOTOR_CURRENT_SENSE_CONFIRMED=0', '-DMOTOR_BOOT_AUTO_ALIGN=0')
    if ($variant -ne 'default') {
        # These definitions apply only to this host test executable.
        $shuntU = '0.01f'; $shuntW = '0.01f'
        $gainU = '50.0f'; $gainW = '50.0f'
        switch ($variant) {
            'shunt_nan' { $shuntU = 'NAN' }
            'shunt_inf' { $shuntW = 'INFINITY' }
            'gain_nan'  { $gainU = 'NAN' }
            'gain_inf'  { $gainW = 'INFINITY' }
        }
        $variantArguments = @(
            '-DMOTOR_CURRENT_SENSE_CONFIRMED=1',
            '-DMOTOR_BOOT_AUTO_ALIGN=0',
            ('-DMOTOR_SHUNT_U_OHM=' + $shuntU), ('-DMOTOR_SHUNT_W_OHM=' + $shuntW),
            ('-DMOTOR_SENSE_GAIN_U=' + $gainU), ('-DMOTOR_SENSE_GAIN_W=' + $gainW)
        )
        if ($variant -ne 'confirmed') { $variantArguments += '-DTEST_BAD_CURRENT_CONFIG=1' }
    }
    $executablePath = Join-Path $buildPath ('test_app_' + $variant + '.exe')
    & $compilerPath @commonArguments @variantArguments @sources '-lm' '-o' $executablePath
    if ($LASTEXITCODE -ne 0) { throw "$variant app test compilation failed: $LASTEXITCODE" }
    & $executablePath
    if ($LASTEXITCODE -ne 0) { throw "$variant app integration test failed: $LASTEXITCODE" }
}

$protocolExecutable = Join-Path $buildPath 'test_can_protocol.exe'
& $compilerPath @commonArguments `
    $project.Sources.CanProtocol `
    (Join-Path $PSScriptRoot 'test_can_protocol.c') '-lm' '-o' $protocolExecutable
if ($LASTEXITCODE -ne 0) { throw "CAN protocol compilation failed: $LASTEXITCODE" }
& $protocolExecutable
if ($LASTEXITCODE -ne 0) { throw "CAN protocol tests failed: $LASTEXITCODE" }

foreach ($stackVariant in @('default','confirmed','open_autotest')) {
    $stackDefinitions = @('-DMOTOR_CURRENT_SENSE_CONFIRMED=1', '-DMOTOR_BOOT_AUTO_ALIGN=0')
    if ($stackVariant -eq 'default') {
        $stackDefinitions = @('-DMOTOR_CURRENT_SENSE_CONFIRMED=0', '-DMOTOR_BOOT_AUTO_ALIGN=0')
    }
    if ($stackVariant -eq 'confirmed') {
        $stackDefinitions = @('-DMOTOR_CURRENT_SENSE_CONFIRMED=1',
            '-DMOTOR_BOOT_AUTO_ALIGN=0',
            '-DMOTOR_SHUNT_U_OHM=0.01f','-DMOTOR_SHUNT_W_OHM=0.01f',
            '-DMOTOR_SENSE_GAIN_U=50.0f','-DMOTOR_SENSE_GAIN_W=50.0f')
    }
    if ($stackVariant -eq 'open_autotest') { $stackDefinitions += '-DMOTOR_OPEN_AUTOTEST=1' }
    $stackExecutable = Join-Path $buildPath ('test_can_stack_' + $stackVariant + '.exe')
    & $compilerPath @commonArguments @stackDefinitions `
        @appSources $project.Sources.CanProtocol $project.Sources.CanService `
        (Join-Path $PSScriptRoot 'test_can_stack.c') '-lm' '-o' $stackExecutable
    if ($LASTEXITCODE -ne 0) { throw "$stackVariant CAN stack compilation failed: $LASTEXITCODE" }
    & $stackExecutable
    if ($LASTEXITCODE -ne 0) { throw "$stackVariant CAN stack tests failed: $LASTEXITCODE" }
}

foreach ($autoEnabled in @(0,1)) {
    $autoExecutable = Join-Path $buildPath ('test_open_autotest_' + $autoEnabled + '.exe')
    & $compilerPath @commonArguments ('-DMOTOR_OPEN_AUTOTEST=' + $autoEnabled) `
        '-DMOTOR_BOOT_AUTO_ALIGN=0' `
        @appSources `
        (Join-Path $PSScriptRoot 'test_open_autotest.c') '-lm' '-o' $autoExecutable
    if ($LASTEXITCODE -ne 0) { throw "Autotest=$autoEnabled compilation failed: $LASTEXITCODE" }
    & $autoExecutable
    if ($LASTEXITCODE -ne 0) { throw "Autotest=$autoEnabled integration failed: $LASTEXITCODE" }
}

foreach ($alignEnabled in @(0,1)) {
    $alignExecutable = Join-Path $buildPath ('test_boot_alignment_' + $alignEnabled + '.exe')
    & $compilerPath @commonArguments ('-DMOTOR_BOOT_AUTO_ALIGN=' + $alignEnabled) `
        '-DMOTOR_OPEN_AUTOTEST=0' @appSources `
        (Join-Path $PSScriptRoot 'test_boot_alignment.c') '-lm' '-o' $alignExecutable
    if ($LASTEXITCODE -ne 0) { throw "Boot alignment=$alignEnabled compilation failed: $LASTEXITCODE" }
    & $alignExecutable
    if ($LASTEXITCODE -ne 0) { throw "Boot alignment=$alignEnabled integration failed: $LASTEXITCODE" }
}

$pythonCommand = Get-Command python -ErrorAction Stop
& $pythonCommand.Source '-B' '-m' 'unittest' 'discover' '-s' $PSScriptRoot '-p' 'test_can_*.py' '-v'
if ($LASTEXITCODE -ne 0) { throw "CAN host frame helper tests failed: $LASTEXITCODE" }
