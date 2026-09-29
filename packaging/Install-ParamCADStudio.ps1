[CmdletBinding()]
param(
    [string]$Destination = (Join-Path $env:LOCALAPPDATA "Programs\ParamCAD Studio"),
    [switch]$InstallDependencies,
    [switch]$NoShortcut,
    [ValidateSet("cuda", "nocuda")]
    [string]$ColmapFlavor = "cuda"
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version 2.0

$source = [System.IO.Path]::GetFullPath($PSScriptRoot)
$destinationPath = [System.IO.Path]::GetFullPath($Destination)
if ($source.Equals($destinationPath, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "The package is already running from the installation directory: $destinationPath"
}

$programsRoot = [System.IO.Path]::GetFullPath((Split-Path -Parent $destinationPath))
$prefix = $programsRoot.TrimEnd('\', '/') + [System.IO.Path]::DirectorySeparatorChar
if (-not $destinationPath.StartsWith($prefix, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Invalid installation destination: $destinationPath"
}

New-Item -ItemType Directory -Path $programsRoot -Force | Out-Null
$temporary = "$destinationPath.installing-$([guid]::NewGuid().ToString('N'))"
$backup = "$destinationPath.backup-$([guid]::NewGuid().ToString('N'))"
try {
    New-Item -ItemType Directory -Path $temporary -Force | Out-Null
    Get-ChildItem -LiteralPath $source -Force | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination $temporary -Recurse -Force
    }

    # Preserve an already downloaded optional runtime when upgrading with a
    # bootstrap package that intentionally does not carry deps/.
    $existingDeps = Join-Path $destinationPath "deps"
    $newDeps = Join-Path $temporary "deps"
    if ((Test-Path -LiteralPath $existingDeps) -and -not (Test-Path -LiteralPath $newDeps)) {
        Copy-Item -LiteralPath $existingDeps -Destination $newDeps -Recurse
    }

    if (Test-Path -LiteralPath $destinationPath) {
        Move-Item -LiteralPath $destinationPath -Destination $backup
    }
    Move-Item -LiteralPath $temporary -Destination $destinationPath

    if (-not $NoShortcut) {
        $startMenu = Join-Path $env:APPDATA "Microsoft\Windows\Start Menu\Programs"
        New-Item -ItemType Directory -Path $startMenu -Force | Out-Null
        $shell = New-Object -ComObject WScript.Shell
        $shortcut = $shell.CreateShortcut((Join-Path $startMenu "ParamCAD Studio.lnk"))
        $shortcut.TargetPath = Join-Path $destinationPath "ParamCAD Studio.exe"
        $shortcut.WorkingDirectory = $destinationPath
        $shortcut.Save()
    }

    if ($InstallDependencies) {
        & (Join-Path $destinationPath "Install-Dependencies.ps1") -ColmapFlavor $ColmapFlavor
    }
    if (Test-Path -LiteralPath $backup) {
        Remove-Item -LiteralPath $backup -Recurse -Force
    }
}
catch {
    if (Test-Path -LiteralPath $temporary) {
        Remove-Item -LiteralPath $temporary -Recurse -Force
    }
    if (-not (Test-Path -LiteralPath $destinationPath) -and (Test-Path -LiteralPath $backup)) {
        Move-Item -LiteralPath $backup -Destination $destinationPath
    }
    throw
}

Write-Host "ParamCAD Studio installed at $destinationPath"
if (-not $NoShortcut) {
    Write-Host "Start menu shortcut: ParamCAD Studio"
}
$python = Join-Path $destinationPath "deps\python\python.exe"
$colmap = Join-Path $destinationPath "deps\colmap\bin\colmap.exe"
if (-not (Test-Path -LiteralPath $python -PathType Leaf) -or
    -not (Test-Path -LiteralPath $colmap -PathType Leaf)) {
    Write-Host "Optional reconstruction dependencies are not installed. Run:"
    Write-Host "  `"$destinationPath\Install-Dependencies.cmd`""
}
