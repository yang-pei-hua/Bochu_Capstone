param(
    [switch]$Cuda,
    [string]$Generator = "Ninja"
)

$ErrorActionPreference = "Stop"
$ModuleRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$Source = Join-Path $ModuleRoot "third_party\colmap"
$Build = Join-Path $ModuleRoot "build\colmap"
$Install = Join-Path $ModuleRoot "install\colmap"
$CudaValue = if ($Cuda) { "ON" } else { "OFF" }

cmake -S $Source -B $Build -G $Generator `
    -DCMAKE_BUILD_TYPE=Release `
    -DCMAKE_INSTALL_PREFIX=$Install `
    -DGUI_ENABLED=OFF `
    -DTESTS_ENABLED=OFF `
    -DBENCHMARK_ENABLED=OFF `
    -DOPENGL_ENABLED=OFF `
    -DONNX_ENABLED=OFF `
    -DCGAL_ENABLED=OFF `
    -DCASPAR_ENABLED=OFF `
    -DCUDA_ENABLED=$CudaValue `
    -DHIP_ENABLED=OFF

cmake --build $Build --config Release --target install

$Executable = Join-Path $Install "bin\colmap.exe"
if (-not (Test-Path -LiteralPath $Executable)) {
    $Executable = Join-Path $Install "bin\colmap"
}
Write-Host "COLMAP installed at $Executable"

