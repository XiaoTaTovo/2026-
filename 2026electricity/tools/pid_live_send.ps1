[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$SessionDirectory,
    [Parameter(Mandatory = $true)]
    [string]$Command
)

$ErrorActionPreference = "Stop"

if (($Command.IndexOf([char]10) -ge 0) -or
    ($Command.IndexOf([char]13) -ge 0) -or
    [string]::IsNullOrWhiteSpace($Command))
{
    throw "Command must be one non-empty line."
}

$sessionPath = [System.IO.Path]::GetFullPath($SessionDirectory)
$commandPath = Join-Path $sessionPath "commands"
$statusPath = Join-Path $sessionPath "status.txt"

if (!(Test-Path -LiteralPath $commandPath) -or
    !(Test-Path -LiteralPath $statusPath))
{
    throw "PID monitor session is not initialized: $sessionPath"
}

$status = Get-Content -LiteralPath $statusPath -Raw -Encoding ASCII
if (!$status.StartsWith("READY "))
{
    throw "PID monitor is not ready: $($status.Trim())"
}

$name = (Get-Date -Format "yyyyMMdd_HHmmss_fff") + "_" +
    [Guid]::NewGuid().ToString("N")
$temporaryPath = Join-Path $commandPath ($name + ".tmp")
$readyPath = Join-Path $commandPath ($name + ".cmd")

[System.IO.File]::WriteAllText(
    $temporaryPath,
    $Command,
    [System.Text.Encoding]::ASCII)
Move-Item -LiteralPath $temporaryPath -Destination $readyPath

Write-Output ("QUEUED " + $Command)
