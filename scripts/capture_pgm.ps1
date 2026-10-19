param(
    [Parameter(Mandatory = $true)]
    [string]$Port,
    [int]$Baud = 115200,
    [string]$OutputDirectory = (Join-Path $PSScriptRoot '..\captures')
)

$ErrorActionPreference = 'Stop'

function Read-ExactBytes {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [int]$Count
    )
    $buffer = New-Object byte[] $Count
    $offset = 0
    while ($offset -lt $Count) {
        $read = $Serial.Read($buffer, $offset, $Count - $offset)
        if ($read -le 0) {
            throw "串口在读取 $Count 字节图像时中断"
        }
        $offset += $read
    }
    return ,$buffer
}

function Read-AsciiLine {
    param([System.IO.Ports.SerialPort]$Serial)
    $bytes = [System.Collections.Generic.List[byte]]::new()
    while ($true) {
        $value = $Serial.ReadByte()
        if ($value -lt 0) {
            throw '串口已关闭'
        }
        if ($value -eq 10) {
            break
        }
        if ($value -ne 13) {
            $bytes.Add([byte]$value)
        }
    }
    return [Text.Encoding]::ASCII.GetString($bytes.ToArray())
}

$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    $Baud,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.ReadTimeout = 30000
$serial.WriteTimeout = 30000
$serial.Open()
try {
    Write-Host "正在等待 $Port 上的 PGM 帧。让车辆保持停止，然后短按主板 S1。"
    while ($true) {
        $line = Read-AsciiLine -Serial $serial
        if ($line -notmatch '^@PGM_BEGIN,w=(\d+),h=(\d+),bytes=(\d+)$') {
            if ($line.StartsWith('@RACE')) {
                Write-Host $line
            }
            continue
        }
        $width = [int]$Matches[1]
        $height = [int]$Matches[2]
        $count = [int]$Matches[3]
        if ($count -ne $width * $height) {
            throw "帧长度 $count 与 ${width}x${height} 不一致"
        }
        $headerText = "P5`n$width $height`n255`n"
        $expectedHeader = [Text.Encoding]::ASCII.GetBytes($headerText)
        $receivedHeader = Read-ExactBytes -Serial $serial -Count $expectedHeader.Length
        if ([Convert]::ToBase64String($receivedHeader) -ne [Convert]::ToBase64String($expectedHeader)) {
            throw 'PGM 文件头不匹配，可能选错了串口或波特率'
        }
        $pixels = Read-ExactBytes -Serial $serial -Count $count
        $output = New-Object byte[] ($expectedHeader.Length + $pixels.Length)
        [Array]::Copy($expectedHeader, 0, $output, 0, $expectedHeader.Length)
        [Array]::Copy($pixels, 0, $output, $expectedHeader.Length, $pixels.Length)
        New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
        $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
        $path = Join-Path $OutputDirectory "frame-$stamp.pgm"
        [IO.File]::WriteAllBytes($path, $output)
        Write-Host "已保存：$path"
        break
    }
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
