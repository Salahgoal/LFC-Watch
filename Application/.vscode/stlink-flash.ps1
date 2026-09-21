$ErrorActionPreference = "Stop"

$stlinkCli = "D:\studaysoftware\stlink utility\ST-LINK Utility\ST-LINK_CLI.exe"
$hex = Join-Path $PSScriptRoot "..\MDK-ARM\07_LVGL_FreeRTOS\07_LVGL_FreeRTOS.hex"

if (-not (Test-Path -LiteralPath $stlinkCli)) {
    throw "ST-LINK_CLI.exe not found: $stlinkCli"
}

if (-not (Test-Path -LiteralPath $hex)) {
    throw "HEX file not found. Run the build task first: $hex"
}

& $stlinkCli -c SWD -P $hex -V after_programming -Rst -Run
exit $LASTEXITCODE
