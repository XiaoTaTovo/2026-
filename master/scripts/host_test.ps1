[CmdletBinding()]
param(
    [string]$Compiler = "gcc"
)

$ErrorActionPreference = "Stop"
$projectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$token = [guid]::NewGuid().ToString("N")
$mainExe = Join-Path ([System.IO.Path]::GetTempPath()) "h2026-host-$token.exe"
$safetyExe = Join-Path ([System.IO.Path]::GetTempPath()) "h2026-safety-$token.exe"
$watchdogExe = Join-Path ([System.IO.Path]::GetTempPath()) `
    "h2026-motion-watchdog-$token.exe"

try {
    $sources = @(
        (Get-ChildItem -LiteralPath (Join-Path $projectDir "core") -Filter "*.c" |
            Sort-Object Name | ForEach-Object FullName)
        (Get-ChildItem -LiteralPath (Join-Path $projectDir "drivers") -Filter "*.c" |
            Sort-Object Name | ForEach-Object FullName)
        (Join-Path $projectDir "app\car_app.c")
        (Join-Path $projectDir "app\h2024_task.c")
        (Join-Path $projectDir "app\h2026_task.c")
        (Join-Path $projectDir "firmware.c")
        (Join-Path $projectDir "tests\test_main.c.reference")
    )

    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic `
        "-I$projectDir" -x c @sources -o $mainExe
    if ($LASTEXITCODE -ne 0) {
        throw "Main host-test compilation failed with exit code $LASTEXITCODE"
    }
    & $mainExe
    if ($LASTEXITCODE -ne 0) {
        throw "Main host tests failed with exit code $LASTEXITCODE"
    }

    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic `
        "-I$projectDir" -x c `
        (Join-Path $projectDir "core\safety_supervisor.c") `
        (Join-Path $projectDir "tests\test_safety_timestamp.c.reference") `
        -o $safetyExe
    if ($LASTEXITCODE -ne 0) {
        throw "Safety-test compilation failed with exit code $LASTEXITCODE"
    }
    & $safetyExe
    if ($LASTEXITCODE -ne 0) {
        throw "Safety timestamp tests failed with exit code $LASTEXITCODE"
    }

    & $Compiler -std=c11 -Wall -Wextra -Werror -pedantic `
        "-I$projectDir" -x c `
        (Join-Path $projectDir "core\motion_watchdog.c") `
        (Join-Path $projectDir "tests\test_motion_watchdog.c.reference") `
        -o $watchdogExe
    if ($LASTEXITCODE -ne 0) {
        throw "Motion-watchdog test compilation failed with exit code $LASTEXITCODE"
    }
    & $watchdogExe
    if ($LASTEXITCODE -ne 0) {
        throw "Motion-watchdog tests failed with exit code $LASTEXITCODE"
    }

    Write-Host "HOST PASS: strict C11 tests completed"
}
finally {
    Remove-Item -LiteralPath $mainExe -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $safetyExe -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $watchdogExe -Force -ErrorAction SilentlyContinue
}
