param(
    [switch]$Rebuild
)

$ErrorActionPreference = "Stop"

$uv4 = "D:\studaysoftware\Keil5MDK\UV4\UV4.exe"
$project = Join-Path $PSScriptRoot "..\MDK-ARM\07_LVGL_FreeRTOS.uvprojx"
$log = Join-Path $PSScriptRoot "..\MDK-ARM\vscode_build.log"
$target = "07_LVGL_FreeRTOS"
$action = if ($Rebuild) { "-r" } else { "-b" }

if (-not (Test-Path -LiteralPath $uv4)) {
    throw "Keil UV4.exe not found: $uv4"
}

if (-not (Test-Path -LiteralPath $project)) {
    throw "Keil project not found: $project"
}

$argumentLine = "$action `"$project`" -t `"$target`" -j0 -o `"$log`""
$process = Start-Process -FilePath $uv4 -ArgumentList $argumentLine -Wait -PassThru -WindowStyle Hidden

if (Test-Path -LiteralPath $log) {
    Get-Content -LiteralPath $log
}

exit $process.ExitCode
