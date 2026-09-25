$ErrorActionPreference = "Stop"

$DepsRoot = $PSScriptRoot
$RequiredFiles = @(
    (Join-Path $DepsRoot "occt-8.0.1\cmake\OpenCASCADEConfig.cmake"),
    (Join-Path $DepsRoot "occt-8.0.1\win64\vc14\lib\TKernel.lib"),
    (Join-Path $DepsRoot "occt-8.0.1\win64\vc14\bin\TKernel.dll"),
    (Join-Path $DepsRoot "vtk-9.7.0\lib\cmake\vtk-9.7\vtk-config.cmake"),
    (Join-Path $DepsRoot "colmap\bin\colmap.exe")
)

foreach ($File in $RequiredFiles) {
    if (-not (Test-Path -LiteralPath $File -PathType Leaf)) {
        throw "Missing dependency anchor: $File"
    }
}

$Colmap = Join-Path $DepsRoot "colmap\bin\colmap.exe"
$ColmapBanner = & $Colmap -h 2>&1 | Select-Object -First 1

Write-Host "Dependency layout OK"
Write-Host "OCCT:  8.0.1 MSVC x64"
Write-Host "VTK:   9.7.0 MSVC x64"
Write-Host "COLMAP: $ColmapBanner"
