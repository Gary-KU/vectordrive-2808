[CmdletBinding()]
param(
    [ValidateSet('Debug', 'Release', 'OpenLoopTest', 'OpenLoopSpin', 'CurrentLoopTest', 'CurrentLoopRunTest', 'CurrentPulseTest', 'CurrentLoop1500')]
    [string]$Configuration = 'Release',
    [string]$Toolchain = '',
    [ValidateSet('2808', '4015')]
    [string]$Motor = '2808',
    [switch]$Test
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$project = Import-PowerShellDataFile -LiteralPath (Join-Path $projectRoot 'project.psd1')
$outputDir = Join-Path $projectRoot "build\$Configuration"
$objectDir = Join-Path $outputDir 'obj'
New-Item -ItemType Directory -Force -Path $objectDir | Out-Null

# Each argument is passed as one PowerShell array element; paths are never
# interpolated into shell code. -Toolchain accepts a GCC toolchain bin folder.
if ($Toolchain) {
    $gcc = Join-Path $Toolchain 'arm-none-eabi-gcc.exe'
} else {
    $gccCommand = Get-Command 'arm-none-eabi-gcc' -ErrorAction SilentlyContinue
    if (-not $gccCommand) {
        throw 'arm-none-eabi-gcc not found. Add its bin folder to PATH or use -Toolchain.'
    }
    $gcc = $gccCommand.Source
}
if (-not (Test-Path -LiteralPath $gcc -PathType Leaf)) { throw "Compiler not found: $gcc" }
$compilerDir = Split-Path -Parent $gcc
$objcopy = Join-Path $compilerDir 'arm-none-eabi-objcopy.exe'
$size = Join-Path $compilerDir 'arm-none-eabi-size.exe'
$nm = Join-Path $compilerDir 'arm-none-eabi-nm.exe'
foreach ($toolPath in @($objcopy, $size, $nm)) {
    if (-not (Test-Path -LiteralPath $toolPath -PathType Leaf)) { throw "Tool not found: $toolPath" }
}

function Invoke-CompilerTool {
    param([string]$Executable, [string[]]$ToolArguments)
    & $Executable @ToolArguments
    if ($LASTEXITCODE -ne 0) { throw "$(Split-Path -Leaf $Executable) failed (exit $LASTEXITCODE)." }
}

$common = @('-mcpu=cortex-m3', '-mthumb', '-mfloat-abi=soft', '-DSTM32F103xB', '-DHSE_VALUE=8000000U')
$common += '-DMOTOR_MODEL=' + $Motor
foreach ($folder in $project.IncludeDirectories) {
    $includePath = Join-Path $projectRoot $folder
    if (-not (Test-Path -LiteralPath $includePath -PathType Container)) {
        throw "Project include directory not found: $includePath"
    }
    $common += '-I' + $includePath
}
$cFlags = @('-std=c99', '-ffunction-sections', '-fdata-sections', '-fno-common', '-Wall', '-Wextra', '-Werror')
if ($Configuration -eq 'Debug') {
    $cFlags += @('-Og', '-g3', '-DDEBUG')
} else {
    # STM32F103 has no FPU. Prefer execution speed for the PWM interrupt;
    # the resulting image still fits the 64 KiB device.
    $cFlags += @('-O2', '-g3', '-DNDEBUG')
}
if ($Configuration -eq 'OpenLoopTest') {
    # Dedicated first-run image; production Debug/Release never enable this.
    $cFlags += @('-DMOTOR_OPEN_AUTOTEST=1', '-DMOTOR_BOOT_AUTO_ALIGN=0')
}
if ($Configuration -eq 'OpenLoopSpin') {
    # Slow continuous open-loop bring-up image: starts once on power-on and
    # keeps spinning (MOTOR_OPEN_SPIN_ELECTRICAL_RAD_S) until STOP or a fault.
    $cFlags += @('-DMOTOR_OPEN_AUTOTEST=1', '-DMOTOR_BOOT_AUTO_ALIGN=0')
    $cFlags += '-DMOTOR_OPEN_AUTOTEST_CONTINUOUS=1'
    # Arm after any reset so SWD flash + reset spins without a full power cycle.
    $cFlags += '-DMOTOR_OPEN_AUTOTEST_IGNORE_RESET=1'
    $cFlags += '-DMOTOR_VOLTAGE_LIMIT_V=2.0f'
    # Uncalibrated sensing: PWM-synchronous offset sampling is noisy. Open loop
    # uses no current feedback, so relax the noise gate (range check still runs).
    $cFlags += '-DMOTOR_OFFSET_MAX_NOISE_COUNTS=4095u'
    # Output disabled: do not drive PWM / raise EN (motor stays stopped).
    $cFlags += '-DMOTOR_OUTPUT_DISABLED=0'
}
if ($Configuration -in @('CurrentLoopTest', 'CurrentLoopRunTest', 'CurrentPulseTest', 'CurrentLoop1500')) {
    # Explicit-command-only image. User confirmed 10 mOhm and INA181A2 (50 V/V).
    # First verify phase mapping/polarity, then encoder direction/alignment,
    # before sending a current command. No startup motion or protection bypass.
    $cFlags += @(
        '-DMOTOR_CURRENT_SENSE_CONFIRMED=1',
        '-DMOTOR_CONTROL_TRACE=1',
        '-DMOTOR_ALIGN_VOLTAGE_V=0.6f',
        '-DMOTOR_OPEN_AUTOTEST=0',
        '-DMOTOR_BOOT_AUTO_ALIGN=0'
    )
    if ($Configuration -eq 'CurrentLoopRunTest') {
        # Separate short spin-test image; production defaults remain 0.1 A / 1 V.
        # Capture the whole one-second command without halting active PWM.
        $cFlags += @(
            '-DMOTOR_CURRENT_LIMIT_A=0.2f', '-DMOTOR_VOLTAGE_LIMIT_V=2.0f',
            '-DMOTOR_OVERCURRENT_TRIP_A=0.5f',
            '-DMOTOR_CURRENT_KP=1.0f', '-DMOTOR_CURRENT_KI=1000.0f',
            '-DAPP_TRACE_CAPACITY=256u', '-DAPP_TRACE_DECIMATION=8u',
            '-DMOTOR_COMMAND_TIMEOUT_MS=1000u'
        )
    } elseif ($Configuration -eq 'CurrentPulseTest') {
        # Explicitly commanded 20 ms pulses; no startup or automatic restart.
        $cFlags += @(
            '-DMOTOR_CURRENT_LIMIT_A=1.0f', '-DMOTOR_VOLTAGE_LIMIT_V=5.0f',
            '-DMOTOR_OVERCURRENT_TRIP_A=1.5f',
            '-DMOTOR_CURRENT_KP=1.0f', '-DMOTOR_CURRENT_KI=1000.0f',
            '-DMOTOR_IQ_SLEW_A_PER_S=200.0f',
            '-DMOTOR_COMMAND_TIMEOUT_MS=20u',
            '-DAPP_TRACE_CAPACITY=128u', '-DAPP_TRACE_DECIMATION=1u'
        )
    } elseif ($Configuration -eq 'CurrentLoop1500') {
        # Continuous (non-step) current regulation for live PI tuning to 1.5 A.
        $cFlags += @(
            '-DMOTOR_CURRENT_LIMIT_A=1.5f', '-DMOTOR_VOLTAGE_LIMIT_V=6.0f',
            '-DMOTOR_OVERCURRENT_TRIP_A=2.5f',
            '-DMOTOR_CURRENT_KP=1.0f', '-DMOTOR_CURRENT_KI=1000.0f',
            '-DMOTOR_IQ_SLEW_A_PER_S=5.0f',
            '-DMOTOR_COMMAND_TIMEOUT_MS=60000u',
            '-DAPP_TRACE_CAPACITY=256u', '-DAPP_TRACE_DECIMATION=8u'
        )
    } else {
        $cFlags += @(
            '-DMOTOR_CURRENT_LIMIT_A=0.1f', '-DMOTOR_VOLTAGE_LIMIT_V=1.0f',
            '-DMOTOR_OVERCURRENT_TRIP_A=0.5f',
            '-DMOTOR_CURRENT_KP=0.2f', '-DMOTOR_CURRENT_KI=20.0f',
            '-DMOTOR_COMMAND_TIMEOUT_MS=2000u', '-DAPP_TRACE_DECIMATION=32u'
        )
    }
}

$sourceFiles = @()
foreach ($group in $project.SourceGroups) {
    $sourceDir = Join-Path $projectRoot $group.Path
    if (-not (Test-Path -LiteralPath $sourceDir -PathType Container)) {
        throw "Project source directory not found: $sourceDir"
    }
    $sourceFiles += @(Get-ChildItem -LiteralPath $sourceDir -Filter '*.c' -File -Recurse | Sort-Object FullName)
}
$sourceFiles = @($sourceFiles | Sort-Object FullName -Unique)
if ($sourceFiles.Count -eq 0) { throw 'No C sources found in project.psd1 SourceGroups.' }
$objectFiles = @()
foreach ($source in $sourceFiles) {
    $relative = $source.FullName.Substring($projectRoot.Length + 1)
    $objectName = ($relative -replace '[^A-Za-z0-9_.-]', '_') + '.o'
    $objectPath = Join-Path $objectDir $objectName
    Write-Host "CC $relative"
    Invoke-CompilerTool $gcc ($common + $cFlags + @('-MMD', '-MP', '-c', $source.FullName, '-o', $objectPath))
    $objectFiles += $objectPath
}
$startupPath = Join-Path $projectRoot $project.Startup.Gcc
$startupObject = Join-Path $objectDir (([System.IO.Path]::GetFileNameWithoutExtension($startupPath)) + '.o')
Invoke-CompilerTool $gcc ($common + @('-g3', '-x', 'assembler-with-cpp', '-c', $startupPath, '-o', $startupObject))
$objectFiles += $startupObject

$elf = Join-Path $outputDir ($project.Name + '.elf')
$map = Join-Path $outputDir ($project.Name + '.map')
$linkerScript = Join-Path $projectRoot $project.Linker.Gcc
$linkArguments = $common + @('--specs=nano.specs', '--specs=nosys.specs', '-nostartfiles', '-T', $linkerScript, '-Wl,--gc-sections', '-Wl,--print-memory-usage', '-Xlinker', '-Map', '-Xlinker', $map, '-o', $elf)
$linkArguments += $objectFiles
$linkArguments += @('-Wl,--start-group', '-lc', '-lm', '-lgcc', '-lnosys', '-Wl,--end-group')
Write-Host "LD $($project.Name).elf ($Configuration, motor $Motor)"
Invoke-CompilerTool $gcc $linkArguments
# The linker reserves zero heap. Catch libc routines that silently pull in an
# allocator (for example newlib-nano strtok or snprintf) before exporting images.
$definedSymbols = @(& $nm '--defined-only' $elf)
if ($LASTEXITCODE -ne 0) { throw 'arm-none-eabi-nm failed while checking the linked image.' }
$heapSymbols = @($definedSymbols | Where-Object {
    $_ -match '\b(malloc|calloc|realloc|free|_malloc_r|_calloc_r|_realloc_r|_free_r|_sbrk|_sbrk_r|strtof|strtod|strtold)$'
})
if ($heapSymbols.Count -gt 0) {
    throw "Zero-heap image contains forbidden library dependencies:`n$($heapSymbols -join [Environment]::NewLine)"
}
Invoke-CompilerTool $objcopy @('-O', 'ihex', $elf, (Join-Path $outputDir ($project.Name + '.hex')))
Invoke-CompilerTool $objcopy @('-O', 'binary', $elf, (Join-Path $outputDir ($project.Name + '.bin')))
Invoke-CompilerTool $size @($elf)
# Read the selected pole count from the actual compiler definitions. The
# profile header remains the source of truth for the two motor configurations.
$motorMacros = @('' | & $gcc @common @cFlags '-E' '-dM' '-x' 'c' '-include' 'motor_config.h' '-include' 'can_config.h' '-')
if ($LASTEXITCODE -ne 0) { throw 'Could not inspect the motor profile.' }
$poleDefinition = @($motorMacros | Where-Object { $_ -match '^#define MOTOR_POLE_PAIRS\s+\d+u?$' })
if ($poleDefinition.Count -ne 1) { throw 'Motor profile must define one integer pole count.' }
$polePairs = [int]([regex]::Match($poleDefinition[0], '\s(\d+)u?$').Groups[1].Value)
$nodeDefaultMatch = [regex]::Match(($motorMacros -join "`n"), '(?m)^#define\s+CAN_NODE_ID\s+(\d+)u?\s*$')
if (-not $nodeDefaultMatch.Success) { throw 'CAN default node ID must be an integer compiler definition.' }
$requestCapMatch = [regex]::Match(($motorMacros -join "`n"), '(?m)^#define\s+MOTOR_IQ_REQUEST_MAX_MA\s+(\d+)u?\s*$')
if (-not $requestCapMatch.Success) { throw 'Motor request ceiling must be an integer mA compiler definition.' }
$bootAlignMatch = [regex]::Match(($motorMacros -join "`n"), '(?m)^#define\s+MOTOR_BOOT_AUTO_ALIGN\s+([01])u?\s*$')
if (-not $bootAlignMatch.Success) { throw 'Startup alignment switch must be 0 or 1.' }
$alignDurationMatch = [regex]::Match(($motorMacros -join "`n"), '(?m)^#define\s+MOTOR_ALIGN_DURATION_MS\s+(\d+)u?\s*$')
if (-not $alignDurationMatch.Success) { throw 'Alignment duration must be an integer number of milliseconds.' }
function Get-CompiledFloat([string]$Macro) {
    $match = [regex]::Match(($motorMacros -join "`n"), ('(?m)^#define\s+' + [regex]::Escape($Macro) + '\s+([0-9]+(?:\.[0-9]+)?)[fF]?\s*$'))
    if (-not $match.Success) { throw "Expected numeric compiler definition: $Macro" }
    return [double]::Parse($match.Groups[1].Value, [Globalization.CultureInfo]::InvariantCulture)
}
$defaultIqA = Get-CompiledFloat 'MOTOR_CURRENT_LIMIT_A'
$phaseTripA = Get-CompiledFloat 'MOTOR_OVERCURRENT_TRIP_A'
$alignVoltageV = Get-CompiledFloat 'MOTOR_ALIGN_VOLTAGE_V'
$bootAlignEnabled = $bootAlignMatch.Groups[1].Value -eq '1'
$modelBase = $project.Name + '_' + $Motor
foreach ($extension in @('elf','hex','bin')) {
    Copy-Item -LiteralPath (Join-Path $outputDir ($project.Name + '.' + $extension)) `
        -Destination (Join-Path $outputDir ($modelBase + '.' + $extension)) -Force
}
$motorManifest = [ordered]@{
    motor = $Motor
    pole_pairs = $polePairs
    configuration = $Configuration
    startup = if ($Configuration -in @('OpenLoopTest','OpenLoopSpin')) {
        'dedicated-open-loop-test'
    } elseif ($bootAlignEnabled) {
        'restore-saved-calibration-else-power-on-auto-align-then-stopped'
    } else {
        'stopped'
    }
    hex = $modelBase + '.hex'
    sha256 = (Get-FileHash -LiteralPath (Join-Path $outputDir ($modelBase + '.hex')) -Algorithm SHA256).Hash
    features = @('persistent-can-node-id-v1','persistent-electrical-calibration-v1','fast-mechanical-feedback-v1','bounded-can-hardware-retry','current-request-limits-v1') +
        $(if ($bootAlignEnabled) { @('power-on-auto-alignment-v1') } else { @() })
    boot_auto_align = $bootAlignEnabled
    alignment_voltage_v = $alignVoltageV
    alignment_duration_ms = [int]$alignDurationMatch.Groups[1].Value
    iq_request_cap_ma = [int]$requestCapMatch.Groups[1].Value
    iq_default_limit_ma = [int][Math]::Round($defaultIqA * 1000.0)
    phase_overcurrent_trip_ma = [int][Math]::Round($phaseTripA * 1000.0)
    current_limit_semantics = 'request ceiling; independent fixed measured-phase trip, no guaranteed achievable current'
    limits_read_opcode = 20
    limits_feedback_base_id = 1536
    calibration_read_opcode = 21
    calibration_save_opcode = 22
    calibration_clear_opcode = 23
    calibration_feedback_base_id = 1600
    calibration_storage = '0x0800F000..0x0800F7FF (two 1 KiB append-only journal pages)'
    calibration_save_semantics = 'explicit stopped save after valid alignment; profile-checked restore after ADC offset; no automatic save'
    can_node_id_range = @(1, 63)
    node_id_storage = '0x0800F800..0x0800FFFF (two 1 KiB pages)'
    node_id_configuration = if ($Configuration -in @('OpenLoopTest','OpenLoopSpin')) { 'read-only' } else { 'stopped-save-and-reset' }
    can_node_id_default = [int]$nodeDefaultMatch.Groups[1].Value
    fast_feedback_period_us = 5000
}
$motorManifest | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $outputDir ($modelBase + '.json')) -Encoding utf8
Write-Host "Motor: $Motor; pole pairs: $polePairs; labeled image: $modelBase.hex"
Write-Host "Build outputs: $outputDir"

if ($Test) {
    $testScript = Join-Path (Split-Path -Parent $projectRoot) 'tests\run_tests.ps1'
    if (-not (Test-Path -LiteralPath $testScript -PathType Leaf)) {
        throw 'tests/run_tests.ps1 is missing. Build completed; host tests could not run.'
    }
    & $testScript -Motor $Motor
    if (-not $?) { throw 'Host tests failed.' }
}
