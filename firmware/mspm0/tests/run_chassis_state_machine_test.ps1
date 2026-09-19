param(
    [string]$ProjectRoot = ""
)

$ErrorActionPreference = "Stop"

# Host-only state-machine test. It does not flash or access hardware.
if([string]::IsNullOrWhiteSpace($ProjectRoot))
{
    $currentDirectory = (Get-Location).Path
    $candidateRoots = @(
        $currentDirectory,
        (Join-Path $currentDirectory "MSPM0G3507_SeekFree_Project")
    )
    foreach($candidate in $candidateRoots)
    {
        $candidateChassis = Join-Path $candidate "project\code\car_chassis.c"
        if(Test-Path -LiteralPath $candidateChassis)
        {
            $ProjectRoot = $candidate
            break
        }
    }
}

if([string]::IsNullOrWhiteSpace($ProjectRoot))
{
    throw "Project root not found. Run from workspace/project root or pass -ProjectRoot."
}
$projectChassis = Join-Path $ProjectRoot "project\code\car_chassis.c"
if(!(Test-Path -LiteralPath $projectChassis))
{
    throw "Invalid -ProjectRoot: project/code/car_chassis.c is missing."
}

$ProjectRoot = (Resolve-Path -LiteralPath $ProjectRoot).Path
$workspaceRoot = Split-Path -Parent $ProjectRoot
$testRoot = Join-Path $ProjectRoot "tests"
$outputDirectory = Join-Path $workspaceRoot "tmp\test-artifacts"
$stateMachineOutput = Join-Path $outputDirectory "chassis_state_machine_test.exe"
$driverOutput = Join-Path $outputDirectory "encoder_motor_driver_test.exe"
$controlOutput = Join-Path $outputDirectory "control_loop_test.exe"
$appKeyOutput = Join-Path $outputDirectory "app_key_safety_test.exe"
$ballTask3Output = Join-Path $outputDirectory "ball_tracker_task3_test.exe"
$telemetryOutput = Join-Path $outputDirectory "telemetry_test.exe"
$plannerOutput = Join-Path $outputDirectory "planner_test.exe"
$imuOutput = Join-Path $outputDirectory "imu_calibration_test.exe"
$commandOutput = Join-Path $outputDirectory "command_test.exe"

New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null

$arguments = @(
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-I$ProjectRoot\project\code",
    "-I$ProjectRoot\libraries\zf_common",
    "$testRoot\chassis_state_machine_test.c",
    "$ProjectRoot\project\code\car_chassis.c",
    "$ProjectRoot\project\code\car_safety.c",
    "$ProjectRoot\project\code\car_planner.c",
    "$ProjectRoot\project\code\car_telemetry.c",
    "-lm",
    "-o",
    $stateMachineOutput
)

& gcc @arguments
if($LASTEXITCODE -ne 0)
{
    throw "GCC failed with exit code $LASTEXITCODE."
}

& $stateMachineOutput
if($LASTEXITCODE -ne 0)
{
    throw "Chassis state-machine test failed with exit code $LASTEXITCODE."
}

$driverArguments = @(
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-I$testRoot\host_stubs",
    "-I$ProjectRoot\project\code",
    "-I$ProjectRoot\libraries\zf_common",
    "$testRoot\encoder_motor_driver_test.c",
    "$ProjectRoot\project\code\car_encoder.c",
    "$ProjectRoot\project\code\car_gray.c",
    "$ProjectRoot\project\code\car_keys.c",
    "$ProjectRoot\project\code\car_motor.c",
    "-o",
    $driverOutput
)

& gcc @driverArguments
if($LASTEXITCODE -ne 0)
{
    throw "GCC driver test build failed with exit code $LASTEXITCODE."
}

& $driverOutput
if($LASTEXITCODE -ne 0)
{
    throw "Encoder/motor driver test failed with exit code $LASTEXITCODE."
}

$controlArguments = @(
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-I$ProjectRoot\project\code",
    "-I$ProjectRoot\libraries\zf_common",
    "$testRoot\control_loop_test.c",
    "$ProjectRoot\project\code\car_control.c",
    "$ProjectRoot\project\code\car_pid.c",
    "-lm",
    "-o",
    $controlOutput
)

& gcc @controlArguments
if($LASTEXITCODE -ne 0)
{
    throw "GCC control-loop test build failed with exit code $LASTEXITCODE."
}

& $controlOutput
if($LASTEXITCODE -ne 0)
{
    throw "Control-loop test failed with exit code $LASTEXITCODE."
}

$appKeyArguments = @(
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-I$ProjectRoot\project\code",
    "-I$ProjectRoot\libraries\zf_common",
    "$testRoot\app_key_safety_test.c",
    "$ProjectRoot\project\code\car_app.c",
    "-o",
    $appKeyOutput
)

& gcc @appKeyArguments
if($LASTEXITCODE -ne 0)
{
    throw "GCC app key-safety test build failed with exit code $LASTEXITCODE."
}

& $appKeyOutput
if($LASTEXITCODE -ne 0)
{
    throw "App key-safety test failed with exit code $LASTEXITCODE."
}

$ballTask3Arguments = @(
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-I$testRoot\host_stubs",
    "-I$ProjectRoot\project\code",
    "-I$ProjectRoot\libraries\zf_common",
    "$testRoot\ball_tracker_task3_test.c",
    "$ProjectRoot\project\code\car_ball_tracker.c",
    "$ProjectRoot\project\code\car_pid.c",
    "-lm",
    "-o",
    $ballTask3Output
)

& gcc @ballTask3Arguments
if($LASTEXITCODE -ne 0)
{
    throw "GCC ball task3 test build failed with exit code $LASTEXITCODE."
}

& $ballTask3Output
if($LASTEXITCODE -ne 0)
{
    throw "Ball task3 test failed with exit code $LASTEXITCODE."
}

$telemetryArguments = @(
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-I$ProjectRoot\project\code",
    "-I$ProjectRoot\libraries\zf_common",
    "$testRoot\telemetry_test.c",
    "$ProjectRoot\project\code\car_telemetry.c",
    "-lm",
    "-o",
    $telemetryOutput
)

& gcc @telemetryArguments
if($LASTEXITCODE -ne 0)
{
    throw "GCC telemetry test build failed with exit code $LASTEXITCODE."
}

& $telemetryOutput
if($LASTEXITCODE -ne 0)
{
    throw "Telemetry test failed with exit code $LASTEXITCODE."
}

$plannerArguments = @(
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-I$ProjectRoot\project\code",
    "-I$ProjectRoot\libraries\zf_common",
    "$testRoot\planner_test.c",
    "$ProjectRoot\project\code\car_planner.c",
    "-lm",
    "-o",
    $plannerOutput
)

& gcc @plannerArguments
if($LASTEXITCODE -ne 0)
{
    throw "GCC planner test build failed with exit code $LASTEXITCODE."
}

& $plannerOutput
if($LASTEXITCODE -ne 0)
{
    throw "Planner test failed with exit code $LASTEXITCODE."
}

$imuArguments = @(
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-I$testRoot\host_stubs",
    "-I$ProjectRoot\project\code",
    "-I$ProjectRoot\libraries\zf_common",
    "$testRoot\imu_calibration_test.c",
    "$ProjectRoot\project\code\car_imu.c",
    "-lm",
    "-o",
    $imuOutput
)

& gcc @imuArguments
if($LASTEXITCODE -ne 0)
{
    throw "GCC IMU calibration test build failed with exit code $LASTEXITCODE."
}

& $imuOutput
if($LASTEXITCODE -ne 0)
{
    throw "IMU calibration test failed with exit code $LASTEXITCODE."
}

$commandArguments = @(
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-I$ProjectRoot\project\code",
    "-I$ProjectRoot\libraries\zf_common",
    "$testRoot\command_test.c",
    "$ProjectRoot\project\code\car_command.c",
    "-lm",
    "-o",
    $commandOutput
)

& gcc @commandArguments
if($LASTEXITCODE -ne 0)
{
    throw "GCC command test build failed with exit code $LASTEXITCODE."
}

& $commandOutput
if($LASTEXITCODE -ne 0)
{
    throw "UART command test failed with exit code $LASTEXITCODE."
}
