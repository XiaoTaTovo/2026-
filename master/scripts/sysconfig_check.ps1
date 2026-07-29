[CmdletBinding()]
param(
    [string]$SysConfigCli =
        "D:\software\Code\CCS_21.0.0\ccs\utils\sysconfig_1.28.0\sysconfig_cli.bat",
    [string]$Product =
        "D:\software\Code\TI\SDK\mspm0_sdk_2_09_00_01\.metadata\product.json",
    [switch]$KeepOutput
)

$ErrorActionPreference = "Stop"
$projectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$scriptPath = Join-Path $projectDir "empty_mspm0g3507.syscfg"
$outputDir = Join-Path ([System.IO.Path]::GetTempPath()) `
    ("h2026-syscfg-" + [guid]::NewGuid().ToString("N"))

if (-not (Test-Path -LiteralPath $SysConfigCli -PathType Leaf)) {
    throw "SysConfig CLI not found: $SysConfigCli"
}
if (-not (Test-Path -LiteralPath $Product -PathType Leaf)) {
    throw "MSPM0 SDK product metadata not found: $Product"
}

New-Item -ItemType Directory -Path $outputDir | Out-Null
try {
    & $SysConfigCli --script $scriptPath --output $outputDir `
        --product $Product --compiler ticlang --treatWarningsAsErrors
    if ($LASTEXITCODE -ne 0) {
        throw "SysConfig validation failed with exit code $LASTEXITCODE"
    }
    Write-Host "SYSCONFIG PASS: warnings are treated as errors"
    if ($KeepOutput) {
        Write-Host "Generated files: $outputDir"
    }
}
finally {
    if (-not $KeepOutput) {
        Remove-Item -LiteralPath $outputDir -Recurse -Force `
            -ErrorAction SilentlyContinue
    }
}
