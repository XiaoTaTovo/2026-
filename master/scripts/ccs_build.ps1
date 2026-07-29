[CmdletBinding()]
param(
    [string]$CcsCli =
        "D:\software\Code\CCS_21.0.0\ccs\eclipse\ccs-server-cli.bat",
    [string]$Workspace = ""
)

$ErrorActionPreference = "Stop"
$projectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$ownsWorkspace = [string]::IsNullOrWhiteSpace($Workspace)
if ($ownsWorkspace) {
    $Workspace = Join-Path ([System.IO.Path]::GetTempPath()) `
        ("h2026-master-ccs-" + [guid]::NewGuid().ToString("N"))
}
$artifact = Join-Path $projectDir `
    "Debug\empty_mspm0g3507_nortos_ticlang.out"
$mapFile = Join-Path $projectDir `
    "Debug\empty_mspm0g3507_nortos_ticlang.map"

if (-not (Test-Path -LiteralPath $CcsCli -PathType Leaf)) {
    throw "CCS CLI not found: $CcsCli"
}

function Invoke-CcsBuildType {
    param([Parameter(Mandatory)][string]$BuildType)

    $commandLine = "`"$CcsCli`" -workspace `"$Workspace`" " +
        "-application projectBuild -ccs.locations `"$projectDir`" " +
        "-ccs.buildType $BuildType -ccs.autoImport -ccs.listProblems"
    & cmd.exe /d /s /c $commandLine
    if ($LASTEXITCODE -ne 0) {
        throw "CCS $BuildType build failed with exit code $LASTEXITCODE"
    }
}

try {
    New-Item -ItemType Directory -Path $Workspace -Force | Out-Null

    # CCS treats clean as a standalone action; full must be a second invocation.
    Invoke-CcsBuildType -BuildType "clean"
    if (Test-Path -LiteralPath $artifact -PathType Leaf) {
        throw "CCS clean left a stale .out artifact: $artifact"
    }
    Invoke-CcsBuildType -BuildType "full"

    if (-not (Test-Path -LiteralPath $artifact -PathType Leaf)) {
        throw "CCS reported success but the .out artifact is missing: $artifact"
    }

    $hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $artifact).Hash
    $size = (Get-Item -LiteralPath $artifact).Length
    Write-Host "BUILD PASS: $artifact"
    Write-Host "OUT bytes: $size"
    Write-Host "OUT SHA-256: $hash"
    if (Test-Path -LiteralPath $mapFile -PathType Leaf) {
        Write-Host "MAP: $mapFile"
    }
}
finally {
    if ($ownsWorkspace -and (Test-Path -LiteralPath $Workspace)) {
        $tempRoot = [System.IO.Path]::GetFullPath(
            [System.IO.Path]::GetTempPath())
        $workspacePath = [System.IO.Path]::GetFullPath($Workspace)
        if (-not $workspacePath.StartsWith(
                $tempRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Refusing to remove CCS workspace outside temp: $workspacePath"
        }
        Remove-Item -LiteralPath $workspacePath -Recurse -Force `
            -ErrorAction SilentlyContinue
    }
}
