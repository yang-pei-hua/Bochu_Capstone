[CmdletBinding()]
param(
    [ValidateSet('Full','Offline')][string]$Variant = 'Full',
    [ValidateSet('cuda','nocuda')][string]$ColmapFlavor = 'cuda',
    [string]$QtRoot = $env:QTDIR,
    [string]$VcpkgRoot = $env:VCPKG_ROOT,
    [string]$VCRuntimeRoot,
    [ValidatePattern('^[0-9]+\.[0-9]+\.[0-9]+([-.][A-Za-z0-9.]+)?$')][string]$Version = '0.2.2',
    [string]$OutputDirectory,
    [string]$BuildDirectory,
    [string]$LocalColmapPath,
    [string]$PythonArchive,
    [switch]$Clean,
    [switch]$SkipBuild,
    [switch]$SkipSmokeTest
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version 2.0
. (Join-Path $PSScriptRoot 'Runtime-Helpers.ps1')

function Invoke-Native([string]$Program, [string[]]$Arguments) {
    Write-Host "> $Program $($Arguments -join ' ')"
    $previous = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        & $Program @Arguments 2>&1 | ForEach-Object { Write-Host ([string]$_) }
        $code = $LASTEXITCODE
    } finally { $ErrorActionPreference = $previous }
    if ($code -ne 0) { throw "$Program failed with exit code $code" }
}
function Reset-Directory([string]$Path) {
    Assert-ChildPath $projectRoot $Path
    if (Test-Path -LiteralPath $Path) { Remove-Item -LiteralPath $Path -Recurse -Force }
    New-Item -ItemType Directory -Path $Path -Force | Out-Null
}
function Write-Utf8([string]$Path, [string]$Text) {
    [IO.File]::WriteAllText($Path,$Text,(New-Object Text.UTF8Encoding($true)))
}
function Get-CacheValue([string]$Name) {
    $cache = Join-Path $BuildDirectory 'CMakeCache.txt'
    if (-not (Test-Path -LiteralPath $cache)) { return '' }
    $line = Get-Content -LiteralPath $cache | Where-Object { $_ -match "^${Name}:[^=]+=" } | Select-Object -First 1
    if ($null -eq $line) { return '' }
    return ($line -split '=',2)[1]
}
function Get-CrtDirectory {
    if ($VCRuntimeRoot) { return [IO.Path]::GetFullPath($VCRuntimeRoot) }
    if ($env:VCToolsRedistDir) { $base = $env:VCToolsRedistDir }
    else {
        $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
        if (-not (Test-Path -LiteralPath $vswhere)) { throw 'Set -VCRuntimeRoot to the x64 Microsoft.VC143.CRT redistribution directory' }
        $vsRoot = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        $versions = Get-ChildItem -LiteralPath (Join-Path $vsRoot 'VC\Redist\MSVC') -Directory | Where-Object { $_.Name -match '^\d+\.\d+\.\d+$' } | Sort-Object { [version]$_.Name } -Descending
        $base = $versions[0].FullName
    }
    return (Join-Path $base 'x64\Microsoft.VC143.CRT')
}

$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$appRoot = Join-Path $projectRoot 'apps\ParamCAD Studio'
if (-not $BuildDirectory) { $BuildDirectory = Join-Path $appRoot 'build\msvc-debug' }
if (-not $OutputDirectory) { $OutputDirectory = Join-Path $projectRoot "artifacts\releases\$Version" }
if (-not $LocalColmapPath) { $LocalColmapPath = Join-Path $projectRoot 'deps\colmap' }
$BuildDirectory = [IO.Path]::GetFullPath($BuildDirectory)
$OutputDirectory = [IO.Path]::GetFullPath($OutputDirectory)
$LocalColmapPath = [IO.Path]::GetFullPath($LocalColmapPath)
Assert-ChildPath $projectRoot $BuildDirectory
Assert-ChildPath $projectRoot $OutputDirectory

# Infer producer dependencies from the existing CMake cache. End users never need them.
if (-not $QtRoot) {
    $qtConfig = Get-CacheValue 'Qt6_DIR'
    if ($qtConfig) { $QtRoot = [IO.Path]::GetFullPath((Join-Path $qtConfig '..\..\..')) }
}
if (-not $VcpkgRoot) {
    $toolchain = Get-CacheValue 'CMAKE_TOOLCHAIN_FILE'
    if ($toolchain) { $VcpkgRoot = [IO.Path]::GetFullPath((Join-Path (Split-Path -Parent $toolchain) '..\..')) }
}
if (-not $QtRoot -or -not $VcpkgRoot) { throw 'Pass -QtRoot and -VcpkgRoot for the first build' }
$QtRoot = [IO.Path]::GetFullPath($QtRoot)
$VcpkgRoot = [IO.Path]::GetFullPath($VcpkgRoot)
$crtDirectory = Get-CrtDirectory
$manifest = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'dependencies.json') -Raw | ConvertFrom-Json
if (-not $PythonArchive) { $PythonArchive = Join-Path $projectRoot "artifacts\cache\$($manifest.dependencies.python.fileName)" }
$PythonArchive = [IO.Path]::GetFullPath($PythonArchive)
foreach ($file in @($PythonArchive, (Join-Path $LocalColmapPath 'bin\colmap.exe'),
                    (Join-Path $crtDirectory 'vcruntime140.dll'))) {
    if (-not (Test-Path -LiteralPath $file -PathType Leaf)) { throw "Missing offline packaging input: $file. No dependency is downloaded by this script." }
}
if ((Get-FileHash -LiteralPath $PythonArchive -Algorithm SHA256).Hash -ne $manifest.dependencies.python.sha256) {
    throw 'Embedded Python archive failed SHA-256 verification'
}
$colmapVersion = (& (Join-Path $LocalColmapPath 'bin\colmap.exe') version | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or $colmapVersion -notmatch 'COLMAP 4\.2\.0') { throw "Unexpected COLMAP runtime: $colmapVersion" }
if (($ColmapFlavor -eq 'cuda') -ne ($colmapVersion -match 'with CUDA')) { throw 'COLMAP flavor does not match the local runtime' }

if ($Clean -and -not $SkipBuild) { Reset-Directory $BuildDirectory }
if (-not $SkipBuild) {
    Invoke-Native 'cmake' @('-S',$appRoot,'-B',$BuildDirectory,'-G','Visual Studio 17 2022','-A','x64',
        "-DCMAKE_TOOLCHAIN_FILE=$(Join-Path $VcpkgRoot 'scripts\buildsystems\vcpkg.cmake')",
        "-DCMAKE_PREFIX_PATH=$QtRoot",
        "-DVTK_DIR=$(Join-Path $projectRoot 'deps\vtk-9.7.0\lib\cmake\vtk-9.7')",
        "-DOpenCASCADE_DIR=$(Join-Path $projectRoot 'deps\occt-8.0.1\cmake')")
    Invoke-Native 'cmake' @('--build',$BuildDirectory,'--config','Release','--target','CadPointCloudClient','--parallel')
}
$release = Join-Path $BuildDirectory 'Release'
$name = "ParamCAD-Studio-$Version-windows-x64-full-$ColmapFlavor"
$stage = Join-Path $projectRoot "artifacts\staging\release\$name"
Reset-Directory $stage
foreach ($directory in @('app','app\plugins','deps','core\reconstruction\colmap_reconstruction','installer','docs\licenses')) {
    New-Item -ItemType Directory -Path (Join-Path $stage $directory) -Force | Out-Null
}
$app = Join-Path $stage 'app'
# Copy only the production executable and runtime libraries, never tests, tools, PDBs or SDKs.
Copy-Item -LiteralPath (Join-Path $release 'ParamCAD Studio.exe') -Destination $app
Get-ChildItem -LiteralPath $release -Filter '*.dll' -File | ForEach-Object { Copy-Item -LiteralPath $_.FullName -Destination $app }
foreach ($plugin in @('platforms','styles','imageformats','iconengines','networkinformation','tls','generic')) {
    $source = Join-Path $release $plugin
    if (Test-Path -LiteralPath $source -PathType Container) {
        Copy-Item -LiteralPath $source -Destination (Join-Path $app "plugins\$plugin") -Recurse
    }
}
Write-Utf8 (Join-Path $app 'qt.conf') "[Paths]`r`nPrefix=.`r`nPlugins=plugins`r`n"

Write-Host 'Bundling Python and the opaque COLMAP runtime payload'
Add-Type -AssemblyName System.IO.Compression.FileSystem
$pythonRoot = Join-Path $stage 'deps\python'
[IO.Compression.ZipFile]::ExtractToDirectory($PythonArchive,$pythonRoot)
$pth = Get-ChildItem -LiteralPath $pythonRoot -Filter 'python*._pth' -File | Select-Object -First 1
if ($null -eq $pth) { throw 'Embedded Python path configuration is missing' }
$lines = @(Get-Content -LiteralPath $pth.FullName)
if ($lines -notcontains '../../core/reconstruction') { $lines += '../../core/reconstruction' }
Set-Content -LiteralPath $pth.FullName -Value $lines -Encoding ASCII
# Bulk-copy the runtime without inspecting or logging files in the local COLMAP dependency.
Copy-Item -LiteralPath $LocalColmapPath -Destination (Join-Path $stage 'deps\colmap') -Recurse
# Clean only the copied payload; never enumerate the opaque local dependency.
$developerFiles = @(Get-ChildItem -LiteralPath $stage -File -Recurse | Where-Object {
    $_.Extension -in @('.pdb','.lib','.exp','.obj','.pyc') -or $_.Name -like '*_test.exe'
})
foreach ($file in $developerFiles) {
    Assert-ChildPath $stage $file.FullName
    Remove-Item -LiteralPath $file.FullName -Force
}
Write-Host "Removed $($developerFiles.Count) developer artifacts from the copied payload"
foreach ($runtime in @($app,$pythonRoot,(Join-Path $stage 'deps\colmap\bin'))) {
    Get-ChildItem -LiteralPath $crtDirectory -Filter '*.dll' -File | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination $runtime -Force
    }
}
$binaryCount = Assert-AppRuntime $app @((Join-Path $QtRoot 'bin'),(Join-Path $VcpkgRoot 'installed\x64-windows\bin'),$crtDirectory) -TrimUnused
$pipeline = Join-Path $projectRoot 'core\reconstruction'
Copy-Item -LiteralPath (Join-Path $pipeline 'reconstruct.py'),(Join-Path $pipeline 'camera_info.example.json') -Destination (Join-Path $stage 'core\reconstruction')
Get-ChildItem -LiteralPath (Join-Path $pipeline 'colmap_reconstruction') -Filter '*.py' -File | ForEach-Object {
    Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $stage 'core\reconstruction\colmap_reconstruction')
}
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'Install-ParamCADStudio.ps1') -Destination (Join-Path $stage 'installer')
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'Install-ParamCADStudio.cmd'),(Join-Path $PSScriptRoot 'Start-ParamCADStudio.cmd') -Destination $stage
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'PACKAGE_README.md') -Destination (Join-Path $stage 'README.md')
Copy-Item -LiteralPath (Join-Path $projectRoot 'LICENSE') -Destination (Join-Path $stage 'docs\licenses\ParamCAD-MIT.txt')
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'THIRD-PARTY-NOTICES.md') -Destination (Join-Path $stage 'docs')
foreach ($library in @('cgal','gmp','mpfr','eigen3')) {
    $copyright = Join-Path $VcpkgRoot "installed\x64-windows\share\$library\copyright"
    if (Test-Path -LiteralPath $copyright) { Copy-Item -LiteralPath $copyright -Destination (Join-Path $stage "docs\licenses\$library.txt") }
}
# Exact license files only; the local VTK/OCCT SDKs are not scanned or copied.
foreach ($entry in @(
    @('deps\vtk-9.7.0\Copyright.txt','VTK.txt'),
    @('deps\occt-8.0.1\LICENSE_LGPL_21.txt','OCCT-LGPL-2.1.txt'),
    @('deps\occt-8.0.1\OCCT_LGPL_EXCEPTION.txt','OCCT-exception.txt'))) {
    $license = Join-Path $projectRoot $entry[0]
    if (Test-Path -LiteralPath $license -PathType Leaf) { Copy-Item -LiteralPath $license -Destination (Join-Path $stage "docs\licenses\$($entry[1])") }
}
# Preserve Qt's installed license notices without accessing any local opaque SDK.
$qtLicenses = [IO.Path]::GetFullPath((Join-Path $QtRoot '..\..\Licenses'))
foreach ($file in @('COPYING.txt','Copyright.txt')) {
    $license = Join-Path $qtLicenses $file
    if (Test-Path -LiteralPath $license) { Copy-Item -LiteralPath $license -Destination (Join-Path $stage "docs\licenses\Qt-$file") }
}
if (-not $SkipSmokeTest) { Invoke-IsolatedRuntimeValidation $stage }

$commit = (& git -C $projectRoot rev-parse HEAD | Out-String).Trim()
$dirty = -not [string]::IsNullOrWhiteSpace((& git -C $projectRoot status --porcelain | Out-String))
Write-Host 'Recording release provenance and file checksums'
$files = @(Get-ChildItem -LiteralPath $stage -File -Recurse | Sort-Object FullName | ForEach-Object {
    [ordered]@{ path=$_.FullName.Substring($stage.Length+1).Replace('\','/'); bytes=$_.Length;
        sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant() }
})
$releaseInfo = [ordered]@{
    schema='paramcad.release'; formatVersion=1; version=$Version; platform='windows-x64'; variant='full';
    offline=$true; entryPoint='Start-ParamCADStudio.cmd'; executable='app/ParamCAD Studio.exe';
    builtAtUtc=[DateTime]::UtcNow.ToString('o'); gitCommit=$commit; workingTreeModified=$dirty;
    dependencies=[ordered]@{ python=$manifest.dependencies.python.version; pythonArchiveSha256=$manifest.dependencies.python.sha256;
        colmap=$colmapVersion; qt=(Get-Item (Join-Path $app 'Qt6Core.dll')).VersionInfo.FileVersion;
        vtk='9.7.0'; occt='8.0.1'; msvc=(Get-Item (Join-Path $app 'vcruntime140.dll')).VersionInfo.FileVersion };
    validation=[ordered]@{ auditedGuiBinaries=$binaryCount; isolatedRuntimeSmokeTest=(-not $SkipSmokeTest) };
    files=$files
}
Write-Utf8 (Join-Path $stage 'package-manifest.json') ($releaseInfo | ConvertTo-Json -Depth 8)
Invoke-Native 'powershell.exe' @('-NoProfile','-ExecutionPolicy','Bypass','-File',
    (Join-Path $PSScriptRoot 'Test-Package.ps1'),'-PackageRoot',$stage,'-VerifyHashes','-SkipRuntime')

New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
$archive = Join-Path $OutputDirectory "$name.zip"
$pending = "$archive.pending-$([guid]::NewGuid().ToString('N'))"
Assert-ChildPath $projectRoot $pending
try {
    Write-Host "Compressing $name (one top-level folder)"
    # Windows PowerShell's .NET Framework ZIP helper uses backslashes in entries.
    # Create entries explicitly so Explorer and cross-platform unzip tools agree.
    $archiveStream = [IO.File]::Create($pending)
    $writer = $null
    try {
        $writer = New-Object IO.Compression.ZipArchive($archiveStream,[IO.Compression.ZipArchiveMode]::Create,$true)
        foreach ($file in Get-ChildItem -LiteralPath $stage -File -Recurse) {
            $relative = $file.FullName.Substring($stage.Length+1).Replace('\','/')
            $entry = $writer.CreateEntry("$name/$relative",[IO.Compression.CompressionLevel]::Optimal)
            $entry.LastWriteTime = [DateTimeOffset]$file.LastWriteTime
            $inputStream = [IO.File]::OpenRead($file.FullName)
            $entryStream = $null
            try {
                $entryStream = $entry.Open()
                $inputStream.CopyTo($entryStream)
            } finally {
                if ($null -ne $entryStream) { $entryStream.Dispose() }
                $inputStream.Dispose()
            }
        }
    } finally {
        if ($null -ne $writer) { $writer.Dispose() }
        $archiveStream.Dispose()
    }
    $zip = [IO.Compression.ZipFile]::OpenRead($pending)
    try {
        foreach ($entry in $zip.Entries) {
            if (-not $entry.FullName.StartsWith("$name/",[StringComparison]::Ordinal)) { throw 'Unexpected archive root' }
        }
        Write-Host "Archive contains $($zip.Entries.Count) entries"
    } finally { $zip.Dispose() }
    if (Test-Path -LiteralPath $archive) { Remove-Item -LiteralPath $archive -Force }
    Move-Item -LiteralPath $pending -Destination $archive
} finally {
    if (Test-Path -LiteralPath $pending) { Remove-Item -LiteralPath $pending -Force }
}
$hash = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant()
[IO.File]::WriteAllText((Join-Path $OutputDirectory 'SHA256SUMS.txt'),"$hash  $name.zip`r`n",(New-Object Text.UTF8Encoding($false)))
Write-Host "Created $archive"
Write-Host "Size: $([math]::Round((Get-Item -LiteralPath $archive).Length/1MB,2)) MiB; SHA256: $hash"
Write-Host 'Portable launch and installation are fully offline. No bootstrap package was created.'
