param([string]$ProbeSerial = '602710971')
$ErrorActionPreference = 'Stop'
$projectDir = Split-Path -Parent $PSScriptRoot
$jlinkExe = 'C:\Program Files\JLink_V960\JLink.exe'
$elfPath = Join-Path $projectDir 'debug\test1.elf'
if (-not (Test-Path -LiteralPath $jlinkExe)) { throw "J-Link not found: $jlinkExe" }
Write-Host 'Disconnect motor driver 12V before flashing. This resets and runs the MCU.'
Push-Location $projectDir
try {
    & ninja -C debug
    if ($LASTEXITCODE -ne 0) { throw 'Build failed; flash cancelled.' }
    if (-not (Test-Path -LiteralPath $elfPath)) { throw "ELF missing: $elfPath" }
    @("loadfile `"$elfPath`"", 'r', 'g', 'q') |
        & $jlinkExe -device MCXA346 -if SWD -speed 100 -SelectEmuBySN $ProbeSerial -autoconnect 1 -ExitOnError 1
    if ($LASTEXITCODE -ne 0) { throw 'J-Link failed; inspect the output above.' }
} finally { Pop-Location }
