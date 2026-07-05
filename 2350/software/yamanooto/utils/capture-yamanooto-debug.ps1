param(
    [string]$Port,
    [int]$BaudRate = 115200,
    [string]$OutputPath
)

$ErrorActionPreference = 'Stop'

if (-not $Port) {
    Write-Host "Available serial ports:"
    Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Name, Description | Format-Table -AutoSize
    Write-Host ""
    Write-Host "Usage: .\capture-yamanooto-debug.ps1 -Port COMx [-OutputPath path]"
    exit 1
}

if (-not $OutputPath) {
    $timestamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $logDir = Join-Path (Split-Path -Parent $PSScriptRoot) 'logs'
    New-Item -ItemType Directory -Force -Path $logDir | Out-Null
    $OutputPath = Join-Path $logDir "yamanooto-debug-$timestamp.log"
}

$serial = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, 'None', 8, 'One'
$serial.NewLine = "`n"
$serial.ReadTimeout = 1000

Write-Host "Capturing Yamanooto USB CDC debug from $Port to $OutputPath"
Write-Host "Press Ctrl+C to stop."

try {
    $serial.Open()
    while ($true) {
        try {
            $line = $serial.ReadLine().TrimEnd("`r", "`n")
            if ($line.Length -gt 0) {
                $stamp = Get-Date -Format 'HH:mm:ss.fff'
                $entry = "[$stamp] $line"
                Write-Host $entry
                Add-Content -Path $OutputPath -Value $entry
            }
        } catch [System.TimeoutException] {
        }
    }
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
