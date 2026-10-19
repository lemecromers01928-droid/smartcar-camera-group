param(
    [Parameter(Mandatory = $true)]
    [string]$Port,
    [int]$Baud = 115200,
    [int]$Seconds = 60,
    [string]$OutputDirectory = (Join-Path $PSScriptRoot '..\logs')
)

$ErrorActionPreference = 'Stop'
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$path = Join-Path $OutputDirectory "telemetry-$stamp.txt"
$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    $Baud,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.ReadTimeout = 1000
$serial.Open()
try {
    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
    Write-Host "正在记录 $Port 的 @RACE 遥测，共 $Seconds 秒。"
    while ([DateTime]::UtcNow -lt $deadline) {
        try {
            $line = $serial.ReadLine()
            $marker = $line.IndexOf('@RACE,')
            if ($marker -ge 0) {
                $record = $line.Substring($marker).Trim()
                Add-Content -LiteralPath $path -Value $record -Encoding utf8
                Write-Host $record
            }
        }
        catch [System.TimeoutException] {
        }
    }
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
Write-Host "遥测已保存：$path"
