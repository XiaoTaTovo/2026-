[CmdletBinding()]
param(
    [string]$PortName = "COM7",
    [int]$BaudRate = 115200,
    [Parameter(Mandatory = $true)]
    [string]$SessionDirectory,
    [int]$DurationMinutes = 120
)

$ErrorActionPreference = "Stop"

$sessionPath = [System.IO.Path]::GetFullPath($SessionDirectory)
$commandPath = Join-Path $sessionPath "commands"
$sentPath = Join-Path $sessionPath "sent"
$serialLogPath = Join-Path $sessionPath "serial.log"
$hostLogPath = Join-Path $sessionPath "host.log"
$statusPath = Join-Path $sessionPath "status.txt"
$stopPath = Join-Path $sessionPath "stop.flag"

[void](New-Item -ItemType Directory -Path $sessionPath -Force)
[void](New-Item -ItemType Directory -Path $commandPath -Force)
[void](New-Item -ItemType Directory -Path $sentPath -Force)

function Set-MonitorStatus
{
    param([string]$Value)

    [System.IO.File]::WriteAllText(
        $statusPath,
        $Value + [Environment]::NewLine,
        [System.Text.Encoding]::ASCII)
}

function Write-HostEvent
{
    param([string]$Value)

    $line = (Get-Date -Format "yyyy-MM-dd HH:mm:ss.fff") + " " + $Value +
        [Environment]::NewLine
    [System.IO.File]::AppendAllText(
        $hostLogPath,
        $line,
        [System.Text.Encoding]::ASCII)
}

$port = [System.IO.Ports.SerialPort]::new(
    $PortName,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One)
$port.ReadBufferSize = 65536
$port.ReadTimeout = 200
$port.WriteTimeout = 500
$port.DtrEnable = $false
$port.RtsEnable = $false

$receivedBytes = 0L
$startedAt = [DateTime]::UtcNow
$deadline = $startedAt.AddMinutes($DurationMinutes)
$nextStatusAt = $startedAt

try
{
    $port.Open()
    Write-HostEvent "MONITOR_READY port=$PortName baud=$BaudRate"
    Set-MonitorStatus "READY port=$PortName baud=$BaudRate bytes=0"

    while (([DateTime]::UtcNow -lt $deadline) -and
           !(Test-Path -LiteralPath $stopPath))
    {
        $chunk = $port.ReadExisting()
        if ($chunk.Length -gt 0)
        {
            [System.IO.File]::AppendAllText(
                $serialLogPath,
                $chunk,
                [System.Text.Encoding]::ASCII)
            $receivedBytes += $chunk.Length
        }

        Get-ChildItem -LiteralPath $commandPath -Filter "*.cmd" -File |
            Sort-Object Name |
            ForEach-Object {
                $command = (Get-Content -LiteralPath $_.FullName -Raw -Encoding ASCII).Trim()
                if ($command.Length -gt 0)
                {
                    $port.Write($command + [Environment]::NewLine)
                    Write-HostEvent ("TX " + $command)
                }
                Move-Item -LiteralPath $_.FullName -Destination (
                    Join-Path $sentPath $_.Name) -Force
            }

        if ([DateTime]::UtcNow -ge $nextStatusAt)
        {
            Set-MonitorStatus (
                "READY port=$PortName baud=$BaudRate bytes=$receivedBytes updated=" +
                (Get-Date -Format "yyyy-MM-dd_HH:mm:ss"))
            $nextStatusAt = [DateTime]::UtcNow.AddSeconds(1)
        }

        Start-Sleep -Milliseconds 20
    }

    Write-HostEvent "MONITOR_STOP_REQUESTED"
    Set-MonitorStatus "STOPPED bytes=$receivedBytes"
}
catch
{
    Write-HostEvent ("MONITOR_FAILED " + $_.Exception.Message)
    Set-MonitorStatus ("FAILED " + $_.Exception.Message)
    exit 1
}
finally
{
    if ($port.IsOpen)
    {
        $port.Close()
    }
    $port.Dispose()
}
