# log_serial.ps1 - Log a serial port to a timestamped CSV file.
# Usage:  powershell -ExecutionPolicy Bypass -File log_serial.ps1 [-Port COM9] [-Baud 115200] [-OutFile name.csv]
# Press Ctrl+C to stop; the file is flushed every 5 s and on exit.

param(
    [string]$Port    = "COM9",
    [int]   $Baud    = 115200,
    [string]$OutFile = ""
)

if ([string]::IsNullOrEmpty($OutFile)) {
    $OutFile = Join-Path $PSScriptRoot ("log_" + (Get-Date -Format "yyyyMMdd_HHmmss") + ".csv")
}

$sp = New-Object System.IO.Ports.SerialPort
$sp.PortName     = $Port
$sp.BaudRate     = $Baud
$sp.Parity       = [System.IO.Ports.Parity]::None
$sp.DataBits     = 8
$sp.StopBits     = [System.IO.Ports.StopBits]::One
$sp.Handshake    = [System.IO.Ports.Handshake]::None
$sp.ReadTimeout  = 500
$sp.Encoding     = [System.Text.Encoding]::ASCII

$writer = New-Object System.IO.StreamWriter($OutFile, $false, [System.Text.Encoding]::UTF8)
$writer.AutoFlush = $false

$buf       = ""
$count     = 0
$lastStat  = Get-Date

try {
    $sp.Open()
    Write-Host ("Logging {0} @ {1} -> {2}  (Ctrl+C to stop)" -f $Port, $Baud, $OutFile)

    while ($true) {
        if ($sp.IsOpen -and ($sp.BytesToRead -gt 0)) {
            $buf += $sp.ReadExisting()
            # Split on CRLF; a partial line stays in $buf for the next read.
            while ($true) {
                $i = $buf.IndexOf("`r`n")
                if ($i -lt 0) { break }
                $line = $buf.Substring(0, $i)
                $buf  = $buf.Substring($i + 2)
                $writer.WriteLine($line)
                $count++
            }
        }
        if (((Get-Date) - $lastStat).TotalSeconds -ge 5) {
            $writer.Flush()
            Write-Host ("{0}  lines={1}  file={2}" -f (Get-Date -Format HH:mm:ss), $count, $OutFile)
            $lastStat = Get-Date
        }
        Start-Sleep -Milliseconds 50
    }
}
finally {
    $writer.Flush()
    $writer.Close()
    if ($sp.IsOpen) { $sp.Close() }
    Write-Host ("Stopped. Total lines: {0}  -> {1}" -f $count, $OutFile)
}
