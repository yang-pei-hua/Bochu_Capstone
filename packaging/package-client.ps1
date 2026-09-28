[CmdletBinding()]
param(
    [ValidateSet("All", "Offline", "Bootstrap")]
    [string]$Variant = "All",
    [ValidateSet("cuda", "nocuda")]
    [string]$ColmapFlavor = "cuda",
    [string]$QtRoot = $env:QTDIR,
    [string]$VcpkgRoot = $env:VCPKG_ROOT,
    [string]$Version = "0.1.0",
    [string]$OutputDirectory,
    [string]$BuildDirectory,
    [switch]$Clean,
    [switch]$SkipBuild,
    [switch]$SkipSmokeTest
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version 2.0

function Invoke-Native([string]$Program, [string[]]$Arguments) {
    Write-Host "> $Program $($Arguments -join ' ')"
    # Windows PowerShell 5 wraps native stderr as ErrorRecord objects. With the
    # script-wide Stop policy, harmless CMake warnings would otherwise abort the
    # package before LASTEXITCODE can be checked.
    $previousErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        & $Program @Arguments 2>&1 | ForEach-Object {
            Write-Host ([string]$_)
        }
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousErrorActionPreference
    }
    if ($exitCode -ne 0) {
        throw "$Program failed with exit code $exitCode"
    }
}

function Assert-WorkspaceChild([string]$Root, [string]$Path) {
    $rootPath = [System.IO.Path]::GetFullPath($Root).TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
    $candidate = [System.IO.Path]::GetFullPath($Path)
    if (-not $candidate.StartsWith($rootPath, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to modify a path outside the workspace: $candidate"
    }
}

function Reset-Directory([string]$Root, [string]$Path) {
    Assert-WorkspaceChild $Root $Path
    if (Test-Path -LiteralPath $Path) {
        Remove-Item -LiteralPath $Path -Recurse -Force
    }
    New-Item -ItemType Directory -Path $Path -Force | Out-Null
}

function Copy-DirectoryContents([string]$Source, [string]$Destination) {
    New-Item -ItemType Directory -Path $Destination -Force | Out-Null
    Get-ChildItem -LiteralPath $Source -Force | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination $Destination -Recurse -Force
    }
}

function Assert-ClientRuntime([string]$Directory) {
    # CGAL is header-heavy, so its GMP dependency is not always reported by
    # CMake's TARGET_RUNTIME_DLLS traversal through the static reconstruction
    # library. Validate the loader-visible file explicitly before smoke tests
    # or archives are created.
    foreach ($file in @("gmp-10.dll")) {
        $path = Join-Path $Directory $file
        if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
            throw "Missing required GUI runtime dependency: $path"
        }
    }
}

function Invoke-GuiSmokeTest([string]$Executable, [string]$WorkingDirectory) {
    Assert-ClientRuntime $WorkingDirectory

    # Suppress Windows loader/crash dialogs so a missing DLL produces an exit
    # code instead of leaving a modal process alive and fooling the timeout.
    if (-not ("ParamCadPackaging.NativeMethods" -as [type])) {
        Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
namespace ParamCadPackaging {
    public static class NativeMethods {
        [DllImport("kernel32.dll")]
        public static extern uint SetErrorMode(uint mode);
    }
}
"@
    }
    $previousErrorMode = [ParamCadPackaging.NativeMethods]::SetErrorMode(0x8003)
    try {
        $process = Start-Process `
            -FilePath $Executable `
            -WorkingDirectory $WorkingDirectory `
            -WindowStyle Hidden `
            -PassThru
        if ($process.WaitForExit(3000)) {
            throw "Packaged GUI exited unexpectedly during smoke test with code $($process.ExitCode)"
        }
        $process.Kill()
        $process.WaitForExit()
    }
    finally {
        [void][ParamCadPackaging.NativeMethods]::SetErrorMode($previousErrorMode)
    }
}

function New-PackageStage([string]$Name, [bool]$IncludeDependencies) {
    $stage = Join-Path $stagingRoot $Name
    Reset-Directory $projectRoot $stage
    Copy-DirectoryContents $releaseDirectory $stage
    Assert-ClientRuntime $stage

    Get-ChildItem -LiteralPath $stage -Filter "*.pdb" -File -Recurse | Remove-Item -Force

    $reconstructionTarget = Join-Path $stage "core\reconstruction"
    New-Item -ItemType Directory -Path $reconstructionTarget -Force | Out-Null
    Copy-Item -LiteralPath (Join-Path $projectRoot "core\reconstruction\reconstruct.py") -Destination $reconstructionTarget
    Copy-Item -LiteralPath (Join-Path $projectRoot "core\reconstruction\camera_info.example.json") -Destination $reconstructionTarget
    Copy-Item -LiteralPath (Join-Path $projectRoot "core\reconstruction\colmap_reconstruction") -Destination $reconstructionTarget -Recurse

    foreach ($file in @(
        "dependencies.json",
        "Install-Dependencies.ps1",
        "Install-Dependencies.cmd",
        "Install-ParamCADStudio.ps1",
        "Install-ParamCADStudio.cmd"
    )) {
        Copy-Item -LiteralPath (Join-Path $PSScriptRoot $file) -Destination $stage
    }
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot "PACKAGE_README.md") -Destination (Join-Path $stage "README.md")

    if ($IncludeDependencies) {
        $arguments = @(
            "-NoProfile", "-ExecutionPolicy", "Bypass",
            "-File", (Join-Path $stage "Install-Dependencies.ps1"),
            "-InstallRoot", $stage,
            "-ManifestPath", (Join-Path $stage "dependencies.json"),
            "-CacheDirectory", (Join-Path $projectRoot "artifacts\cache"),
            "-ColmapFlavor", $ColmapFlavor
        )
        if ($ColmapFlavor -eq "cuda") {
            $arguments += @("-LocalColmapPath", (Join-Path $projectRoot "deps\colmap"))
        }
        Invoke-Native "powershell.exe" $arguments
    }

    if (-not $SkipSmokeTest) {
        $executable = Join-Path $stage "ParamCAD Studio.exe"
        Invoke-GuiSmokeTest $executable $stage
    }

    $archive = Join-Path $OutputDirectory "$Name.zip"
    if (Test-Path -LiteralPath $archive) {
        Remove-Item -LiteralPath $archive -Force
    }
    # ZipFile streams the archive without the high memory usage of
    # Compress-Archive and, unlike bsdtar, does not prefix entries with "./".
    # That prefix is legal ZIP but makes Windows Explorer display an empty
    # archive, so use the .NET writer for native Explorer compatibility.
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    [System.IO.Compression.ZipFile]::CreateFromDirectory(
        $stage,
        $archive,
        [System.IO.Compression.CompressionLevel]::Optimal,
        $false
    )
    Write-Host "Created $archive"
    return $archive
}

$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$appRoot = Join-Path $projectRoot "apps\ParamCAD Studio"
if ([string]::IsNullOrWhiteSpace($BuildDirectory)) {
    $BuildDirectory = Join-Path $appRoot "build\package"
}
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $OutputDirectory = Join-Path $projectRoot "artifacts\packages"
}
$BuildDirectory = [System.IO.Path]::GetFullPath($BuildDirectory)
$OutputDirectory = [System.IO.Path]::GetFullPath($OutputDirectory)
$stagingRoot = Join-Path $projectRoot "artifacts\staging"
Assert-WorkspaceChild $projectRoot $BuildDirectory
Assert-WorkspaceChild $projectRoot $OutputDirectory
Assert-WorkspaceChild $projectRoot $stagingRoot

if ([string]::IsNullOrWhiteSpace($QtRoot)) {
    throw "QtRoot is required. Pass -QtRoot <Qt MSVC x64 root> or set QTDIR."
}
if ([string]::IsNullOrWhiteSpace($VcpkgRoot)) {
    throw "VcpkgRoot is required for CGAL. Pass -VcpkgRoot <vcpkg root> or set VCPKG_ROOT."
}
$QtRoot = [System.IO.Path]::GetFullPath($QtRoot)
$VcpkgRoot = [System.IO.Path]::GetFullPath($VcpkgRoot)

$requiredFiles = @(
    (Join-Path $QtRoot "lib\cmake\Qt6\Qt6Config.cmake"),
    (Join-Path $VcpkgRoot "scripts\buildsystems\vcpkg.cmake"),
    (Join-Path $projectRoot "deps\vtk-9.7.0\lib\cmake\vtk-9.7\vtk-config.cmake"),
    (Join-Path $projectRoot "deps\occt-8.0.1\cmake\OpenCASCADEConfig.cmake")
)
foreach ($file in $requiredFiles) {
    if (-not (Test-Path -LiteralPath $file -PathType Leaf)) {
        throw "Missing packaging prerequisite: $file"
    }
}
if (($Variant -eq "All" -or $Variant -eq "Offline") -and $ColmapFlavor -eq "cuda") {
    $localColmap = Join-Path $projectRoot "deps\colmap\bin\colmap.exe"
    if (-not (Test-Path -LiteralPath $localColmap -PathType Leaf)) {
        throw "Missing local CUDA COLMAP runtime for the offline package: $localColmap"
    }
}

if ($Clean -and -not $SkipBuild) {
    if (Test-Path -LiteralPath $BuildDirectory) {
        Assert-WorkspaceChild $projectRoot $BuildDirectory
        Remove-Item -LiteralPath $BuildDirectory -Recurse -Force
    }
}
if (-not $SkipBuild) {
    $configure = @(
        "-S", $appRoot,
        "-B", $BuildDirectory,
        "-G", "Visual Studio 17 2022",
        "-A", "x64",
        "-DCMAKE_TOOLCHAIN_FILE=$((Join-Path $VcpkgRoot 'scripts\buildsystems\vcpkg.cmake') -replace '\\', '/')",
        "-DCMAKE_PREFIX_PATH=$($QtRoot -replace '\\', '/')",
        "-DVTK_DIR=$((Join-Path $projectRoot 'deps\vtk-9.7.0\lib\cmake\vtk-9.7') -replace '\\', '/')",
        "-DOpenCASCADE_DIR=$((Join-Path $projectRoot 'deps\occt-8.0.1\cmake') -replace '\\', '/')"
    )
    Invoke-Native "cmake" $configure
    Invoke-Native "cmake" @("--build", $BuildDirectory, "--config", "Release", "--target", "CadPointCloudClient", "--parallel")
}

$releaseDirectory = Join-Path $BuildDirectory "Release"
$releaseExecutable = Join-Path $releaseDirectory "ParamCAD Studio.exe"
if (-not (Test-Path -LiteralPath $releaseExecutable -PathType Leaf)) {
    throw "Release executable was not produced: $releaseExecutable"
}
# TARGET_RUNTIME_DLLS does not reliably see GMP through CGAL's static target.
# Copy it from the same x64 vcpkg installation used to configure this build,
# then validate again after every package stage is populated.
$vcpkgRuntimeDirectory = Join-Path $VcpkgRoot "installed\x64-windows\bin"
$gmpRuntime = Join-Path $vcpkgRuntimeDirectory "gmp-10.dll"
if (-not (Test-Path -LiteralPath $gmpRuntime -PathType Leaf)) {
    throw "Missing CGAL GMP runtime in vcpkg: $gmpRuntime"
}
Copy-Item -LiteralPath $gmpRuntime -Destination $releaseDirectory -Force
Assert-ClientRuntime $releaseDirectory

New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
New-Item -ItemType Directory -Path $stagingRoot -Force | Out-Null

$created = @()
if ($Variant -eq "All" -or $Variant -eq "Offline") {
    $created += New-PackageStage "ParamCAD-Studio-$Version-windows-x64-offline-$ColmapFlavor" $true
}
if ($Variant -eq "All" -or $Variant -eq "Bootstrap") {
    $created += New-PackageStage "ParamCAD-Studio-$Version-windows-x64-bootstrap" $false
}

Write-Host "Packages:"
$created | ForEach-Object { Write-Host "  $_" }
