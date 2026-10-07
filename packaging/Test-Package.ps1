[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$PackageRoot, [switch]$VerifyHashes, [switch]$SkipRuntime)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version 2.0
. (Join-Path $PSScriptRoot 'Runtime-Helpers.ps1')
$root = [IO.Path]::GetFullPath($PackageRoot)
foreach ($file in @('README.md','Start-ParamCADStudio.cmd','Install-ParamCADStudio.cmd',
                   'installer\Install-ParamCADStudio.ps1','package-manifest.json',
                   'app\ParamCAD Studio.exe','app\qt.conf','app\gmp-10.dll',
                   'deps\python\python.exe','deps\colmap\bin\colmap.exe','core\reconstruction\reconstruct.py')) {
    if (-not (Test-Path -LiteralPath (Join-Path $root $file) -PathType Leaf)) { throw "Missing release file: $file" }
}
$info = Get-Content -LiteralPath (Join-Path $root 'package-manifest.json') -Raw | ConvertFrom-Json
if ($info.schema -ne 'paramcad.release' -or $info.variant -ne 'full' -or -not $info.offline) { throw 'Only the full offline release layout is supported' }
$extra = Get-ChildItem -LiteralPath $root -Recurse -File | Where-Object {
    $_.Extension -in @('.pdb','.lib','.exp','.obj','.pyc') -or $_.Name -like '*_test.exe' -or
    $_.Name -like 'Install-Dependencies.*'
}
if ($extra) { throw 'Release contains developer artifacts or a dependency downloader' }
if ($VerifyHashes) {
    foreach ($file in $info.files) {
        $path = Join-Path $root $file.path
        Assert-ChildPath $root $path
        if ((Get-Item -LiteralPath $path).Length -ne $file.bytes -or
            (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash -ne $file.sha256) {
            throw "Release integrity check failed: $($file.path)"
        }
    }
    Write-Host "Verified $($info.files.Count) bundled file checksums"
}
$count = Assert-AppRuntime (Join-Path $root 'app')
Write-Host "Verified $count GUI binary dependency closures"
if (-not $SkipRuntime) { Invoke-IsolatedRuntimeValidation $root }
Write-Host 'Full offline package validation passed'
