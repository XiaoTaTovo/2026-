[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$SessionDirectory
)

$ErrorActionPreference = "Stop"

$sessionPath = [System.IO.Path]::GetFullPath($SessionDirectory)
if (!(Test-Path -LiteralPath $sessionPath))
{
    throw "PID monitor session does not exist: $sessionPath"
}

$stopPath = Join-Path $sessionPath "stop.flag"
[void](New-Item -ItemType File -Path $stopPath -Force)
Write-Output "STOP_REQUESTED"
