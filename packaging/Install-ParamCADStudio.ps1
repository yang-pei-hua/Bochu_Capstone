[CmdletBinding()]
param(
    [string]$Destination = (Join-Path $env:LOCALAPPDATA 'Programs\ParamCAD Studio'),
    [switch]$NoShortcut
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version 2.0

function Assert-InstallChild([string]$Path) {
    $candidate = [IO.Path]::GetFullPath($Path)
    $prefix = $programsRoot.TrimEnd('\','/') + [IO.Path]::DirectorySeparatorChar
    if (-not $candidate.StartsWith($prefix,[StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to modify a path outside the installation parent: $candidate"
    }
}
function Test-Payload([string]$Root) {
    $data = Get-Content -LiteralPath (Join-Path $Root 'package-manifest.json') -Raw | ConvertFrom-Json
    if ($data.schema -ne 'paramcad.release' -or $data.variant -ne 'full' -or -not $data.offline) {
        throw 'Installation requires a complete offline ParamCAD Studio release'
    }
    $prefix = [IO.Path]::GetFullPath($Root).TrimEnd('\','/') + [IO.Path]::DirectorySeparatorChar
    foreach ($file in $data.files) {
        $path = [IO.Path]::GetFullPath((Join-Path $Root $file.path))
        if (-not $path.StartsWith($prefix,[StringComparison]::OrdinalIgnoreCase)) { throw 'Invalid release manifest path' }
        if ((Get-Item -LiteralPath $path).Length -ne $file.bytes -or
            (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash -ne $file.sha256) {
            throw "Package integrity check failed: $($file.path)"
        }
    }
    foreach ($file in @('app\ParamCAD Studio.exe','deps\python\python.exe','deps\colmap\bin\colmap.exe')) {
        if (-not (Test-Path -LiteralPath (Join-Path $Root $file) -PathType Leaf)) { throw "Missing bundled runtime: $file" }
    }
}

$source = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..')).TrimEnd('\','/')
$destinationPath = [IO.Path]::GetFullPath($Destination).TrimEnd('\','/')
$sourcePrefix = $source + [IO.Path]::DirectorySeparatorChar
$destinationPrefix = $destinationPath + [IO.Path]::DirectorySeparatorChar
if ($source.Equals($destinationPath,[StringComparison]::OrdinalIgnoreCase) -or
    $source.StartsWith($destinationPrefix,[StringComparison]::OrdinalIgnoreCase) -or
    $destinationPath.StartsWith($sourcePrefix,[StringComparison]::OrdinalIgnoreCase)) {
    throw 'Source and destination must be separate directories, not ancestors of one another'
}
$programsRoot = Split-Path -Parent $destinationPath
if ([string]::IsNullOrWhiteSpace($programsRoot) -or $destinationPath -eq [IO.Path]::GetPathRoot($destinationPath).TrimEnd('\')) {
    throw 'An installation destination must name a folder, not a drive root'
}
Assert-InstallChild $destinationPath
if (Test-Path -LiteralPath $destinationPath) {
    $existingManifest = Join-Path $destinationPath 'package-manifest.json'
    if (Test-Path -LiteralPath $existingManifest) {
        $existingInfo = Get-Content -LiteralPath $existingManifest -Raw | ConvertFrom-Json
        if ($existingInfo.schema -ne 'paramcad.release') { throw 'The destination belongs to a different application' }
    } elseif (-not (Test-Path -LiteralPath (Join-Path $destinationPath 'ParamCAD Studio.exe') -PathType Leaf)) {
        throw 'The existing destination is not a ParamCAD Studio installation'
    }
}
Write-Host 'Checking the full offline package (no downloads or separate runtime installers)'
Test-Payload $source
New-Item -ItemType Directory -Path $programsRoot -Force | Out-Null
$temporary = "$destinationPath.installing-$([guid]::NewGuid().ToString('N'))"
$backup = "$destinationPath.backup-$([guid]::NewGuid().ToString('N'))"
Assert-InstallChild $temporary
Assert-InstallChild $backup
$replaced = $false
try {
    New-Item -ItemType Directory -Path $temporary | Out-Null
    Get-ChildItem -LiteralPath $source -Force | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination $temporary -Recurse
    }
    Test-Payload $temporary
    # Captures and reconstruction results are user data, retained when upgrading.
    $existingOutputs = Join-Path $destinationPath 'outputs'
    if (Test-Path -LiteralPath $existingOutputs) {
        $newOutputs = Join-Path $temporary 'outputs'
        New-Item -ItemType Directory -Path $newOutputs -Force | Out-Null
        Get-ChildItem -LiteralPath $existingOutputs -Force | ForEach-Object {
            Copy-Item -LiteralPath $_.FullName -Destination $newOutputs -Recurse -Force
        }
    }
    if (Test-Path -LiteralPath $destinationPath) {
        Move-Item -LiteralPath $destinationPath -Destination $backup
    }
    Move-Item -LiteralPath $temporary -Destination $destinationPath
    $replaced = $true
    if (-not $NoShortcut) {
        $startMenu = Join-Path $env:APPDATA 'Microsoft\Windows\Start Menu\Programs'
        New-Item -ItemType Directory -Path $startMenu -Force | Out-Null
        $shell = New-Object -ComObject WScript.Shell
        $shortcut = $shell.CreateShortcut((Join-Path $startMenu 'ParamCAD Studio.lnk'))
        $shortcut.TargetPath = Join-Path $destinationPath 'app\ParamCAD Studio.exe'
        $shortcut.WorkingDirectory = $destinationPath
        $shortcut.Save()
    }
} catch {
    if ($replaced -and (Test-Path -LiteralPath $destinationPath)) {
        Assert-InstallChild $destinationPath
        Remove-Item -LiteralPath $destinationPath -Recurse -Force
    }
    if (Test-Path -LiteralPath $backup) {
        Assert-InstallChild $backup
        Move-Item -LiteralPath $backup -Destination $destinationPath
    }
    throw
} finally {
    if (Test-Path -LiteralPath $temporary) {
        Assert-InstallChild $temporary
        Remove-Item -LiteralPath $temporary -Recurse -Force
    }
}
if (Test-Path -LiteralPath $backup) {
    Assert-InstallChild $backup
    Remove-Item -LiteralPath $backup -Recurse -Force
}
Write-Host "ParamCAD Studio installed at $destinationPath"
Write-Host 'All dependencies were copied from the package. Nothing was downloaded.'
if (-not $NoShortcut) { Write-Host 'Start menu shortcut: ParamCAD Studio' }
