[CmdletBinding()]
param(
    [ValidateSet("cuda", "nocuda")]
    [string]$ColmapFlavor = "cuda",
    [string]$InstallRoot,
    [string]$ManifestPath,
    [string]$CacheDirectory,
    [string]$LocalColmapPath,
    [switch]$Force
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version 2.0

function Get-FullPath([string]$Path) {
    return [System.IO.Path]::GetFullPath($Path)
}

function Assert-ChildPath([string]$Root, [string]$Path) {
    $rootPath = Get-FullPath $Root
    $candidate = Get-FullPath $Path
    $prefix = $rootPath.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
    if (-not $candidate.StartsWith($prefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to modify a dependency path outside the installation root: $candidate"
    }
}

function Test-Hash([string]$Path, [string]$Expected) {
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        return $false
    }
    $actual = (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash
    return $actual.Equals($Expected, [System.StringComparison]::OrdinalIgnoreCase)
}

function Get-Archive([pscustomobject]$Dependency) {
    New-Item -ItemType Directory -Path $CacheDirectory -Force | Out-Null
    $archive = Join-Path $CacheDirectory $Dependency.fileName
    if (Test-Hash $archive $Dependency.sha256) {
        Write-Host "Using cached $($Dependency.fileName)"
        return $archive
    }
    if (Test-Path -LiteralPath $archive) {
        Remove-Item -LiteralPath $archive -Force
    }

    $partial = "$archive.partial-$([guid]::NewGuid().ToString('N'))"
    try {
        Write-Host "Downloading $($Dependency.url)"
        Invoke-WebRequest -Uri $Dependency.url -OutFile $partial -UseBasicParsing
        if (-not (Test-Hash $partial $Dependency.sha256)) {
            $actual = (Get-FileHash -LiteralPath $partial -Algorithm SHA256).Hash
            throw "SHA-256 mismatch for $($Dependency.fileName): expected $($Dependency.sha256), got $actual"
        }
        Move-Item -LiteralPath $partial -Destination $archive
    }
    finally {
        if (Test-Path -LiteralPath $partial) {
            Remove-Item -LiteralPath $partial -Force
        }
    }
    return $archive
}

function Write-DependencyMarker([string]$Target, [string]$Id, [pscustomobject]$Dependency) {
    $marker = [ordered]@{
        id = $Id
        version = $Dependency.version
        sha256 = $Dependency.sha256
        installedAtUtc = [DateTime]::UtcNow.ToString("o")
    }
    $marker | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $Target ".paramcad-dependency.json") -Encoding UTF8
}

function Test-Installed([string]$Target, [string]$Id, [pscustomobject]$Dependency) {
    $anchor = Join-Path $Target ($Dependency.anchor -replace '/', '\')
    $markerPath = Join-Path $Target ".paramcad-dependency.json"
    if (-not (Test-Path -LiteralPath $anchor -PathType Leaf) -or
        -not (Test-Path -LiteralPath $markerPath -PathType Leaf)) {
        return $false
    }
    try {
        $marker = Get-Content -LiteralPath $markerPath -Raw | ConvertFrom-Json
        return $marker.id -eq $Id -and $marker.version -eq $Dependency.version -and
            $marker.sha256 -eq $Dependency.sha256
    }
    catch {
        return $false
    }
}

function Install-PreparedDirectory(
    [string]$Prepared,
    [string]$Target,
    [string]$Id,
    [pscustomobject]$Dependency
) {
    Assert-ChildPath $InstallRoot $Target
    $anchor = Join-Path $Prepared ($Dependency.anchor -replace '/', '\')
    if (-not (Test-Path -LiteralPath $anchor -PathType Leaf)) {
        throw "Unexpected $Id payload layout: missing $($Dependency.anchor)"
    }

    New-Item -ItemType Directory -Path (Split-Path -Parent $Target) -Force | Out-Null
    $backup = "$Target.backup-$([guid]::NewGuid().ToString('N'))"
    try {
        if (Test-Path -LiteralPath $Target) {
            Move-Item -LiteralPath $Target -Destination $backup
        }
        Move-Item -LiteralPath $Prepared -Destination $Target
        Write-DependencyMarker $Target $Id $Dependency
        if (Test-Path -LiteralPath $backup) {
            Remove-Item -LiteralPath $backup -Recurse -Force
        }
    }
    catch {
        if (Test-Path -LiteralPath $Target) {
            Remove-Item -LiteralPath $Target -Recurse -Force
        }
        if (Test-Path -LiteralPath $backup) {
            Move-Item -LiteralPath $backup -Destination $Target
        }
        throw
    }
}

function Install-ArchiveDependency([string]$Id, [pscustomobject]$Dependency) {
    $target = Join-Path $InstallRoot ($Dependency.installPath -replace '/', '\')
    if (-not $Force -and (Test-Installed $target $Id $Dependency)) {
        Write-Host "$Id $($Dependency.version) is already installed"
        return
    }

    $archive = Get-Archive $Dependency
    $tempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("paramcad-dependency-" + [guid]::NewGuid().ToString("N"))
    $extracted = Join-Path $tempRoot "payload"
    try {
        New-Item -ItemType Directory -Path $extracted -Force | Out-Null
        Expand-Archive -LiteralPath $archive -DestinationPath $extracted
        Install-PreparedDirectory $extracted $target $Id $Dependency
        $extracted = $null
    }
    finally {
        if (Test-Path -LiteralPath $tempRoot) {
            Remove-Item -LiteralPath $tempRoot -Recurse -Force
        }
    }
}

function Install-LocalColmap([string]$Source, [pscustomobject]$Dependency) {
    $sourcePath = Get-FullPath $Source
    $sourceAnchor = Join-Path $sourcePath "bin\colmap.exe"
    if (-not (Test-Path -LiteralPath $sourceAnchor -PathType Leaf)) {
        throw "Local COLMAP payload is missing bin\colmap.exe: $sourcePath"
    }
    $target = Join-Path $InstallRoot ($Dependency.installPath -replace '/', '\')
    if (-not $Force -and (Test-Installed $target "colmap-$ColmapFlavor" $Dependency)) {
        Write-Host "COLMAP $($Dependency.version) is already installed"
        return
    }
    $tempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("paramcad-colmap-" + [guid]::NewGuid().ToString("N"))
    try {
        New-Item -ItemType Directory -Path $tempRoot -Force | Out-Null
        $prepared = Join-Path $tempRoot "payload"
        Copy-Item -LiteralPath $sourcePath -Destination $prepared -Recurse
        Install-PreparedDirectory $prepared $target "colmap-$ColmapFlavor" $Dependency
    }
    finally {
        if (Test-Path -LiteralPath $tempRoot) {
            Remove-Item -LiteralPath $tempRoot -Recurse -Force
        }
    }
}

function Configure-EmbeddedPython([string]$PythonRoot) {
    $pathFile = Get-ChildItem -LiteralPath $PythonRoot -Filter "python*._pth" -File | Select-Object -First 1
    if ($null -eq $pathFile) {
        throw "Embedded Python path configuration was not found under $PythonRoot"
    }
    $reconstructionPath = "../../core/reconstruction"
    $lines = @(Get-Content -LiteralPath $pathFile.FullName)
    if ($lines -notcontains $reconstructionPath) {
        $lines += $reconstructionPath
        Set-Content -LiteralPath $pathFile.FullName -Value $lines -Encoding ASCII
    }
}

if ([string]::IsNullOrWhiteSpace($InstallRoot)) {
    if (Test-Path -LiteralPath (Join-Path $PSScriptRoot "ParamCAD Studio.exe")) {
        $InstallRoot = $PSScriptRoot
    }
    else {
        $InstallRoot = Split-Path -Parent $PSScriptRoot
    }
}
if ([string]::IsNullOrWhiteSpace($ManifestPath)) {
    $ManifestPath = Join-Path $PSScriptRoot "dependencies.json"
}
if ([string]::IsNullOrWhiteSpace($CacheDirectory)) {
    $CacheDirectory = Join-Path $env:LOCALAPPDATA "ParamCAD Studio\download-cache"
}
$InstallRoot = Get-FullPath $InstallRoot
$ManifestPath = Get-FullPath $ManifestPath
$CacheDirectory = Get-FullPath $CacheDirectory

if (-not (Test-Path -LiteralPath $ManifestPath -PathType Leaf)) {
    throw "Dependency manifest was not found: $ManifestPath"
}
$manifest = Get-Content -LiteralPath $ManifestPath -Raw | ConvertFrom-Json
if ($manifest.schemaVersion -ne 1) {
    throw "Unsupported dependency manifest schema: $($manifest.schemaVersion)"
}

New-Item -ItemType Directory -Path $InstallRoot -Force | Out-Null
Install-ArchiveDependency "python" $manifest.dependencies.python
Configure-EmbeddedPython (Join-Path $InstallRoot "deps\python")

$colmap = if ($ColmapFlavor -eq "cuda") {
    $manifest.dependencies.colmapCuda
}
else {
    $manifest.dependencies.colmapNoCuda
}
if ([string]::IsNullOrWhiteSpace($LocalColmapPath)) {
    Install-ArchiveDependency "colmap-$ColmapFlavor" $colmap
}
else {
    Install-LocalColmap $LocalColmapPath $colmap
}

$python = Join-Path $InstallRoot "deps\python\python.exe"
$script = Join-Path $InstallRoot "core\reconstruction\reconstruct.py"
$colmapExe = Join-Path $InstallRoot "deps\colmap\bin\colmap.exe"
if (Test-Path -LiteralPath $script -PathType Leaf) {
    & $python $script --help | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "Embedded Python could not load the packaged reconstruction script"
    }
}
& $colmapExe version | Select-Object -First 1 | Write-Host
if ($LASTEXITCODE -ne 0) {
    throw "COLMAP validation failed with exit code $LASTEXITCODE"
}
Write-Host "ParamCAD Studio reconstruction dependencies are ready under $InstallRoot"
