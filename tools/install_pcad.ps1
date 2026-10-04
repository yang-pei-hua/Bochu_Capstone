# Register this checkout's pcad command in the current user's PATH.
[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$pcadToolsDirectory = [System.IO.Path]::GetFullPath($PSScriptRoot)
if (-not (Test-Path -LiteralPath (Join-Path $pcadToolsDirectory 'pcad.cmd') -PathType Leaf)) {
    throw "pcad.cmd is missing from $pcadToolsDirectory"
}

$pcadUserPath = [Environment]::GetEnvironmentVariable('Path', 'User')
$pcadPathEntries = @($pcadUserPath -split ';' | Where-Object { $_ })
$pcadAlreadyRegistered = @($pcadPathEntries | Where-Object {
    $_.TrimEnd('\', '/') -ieq $pcadToolsDirectory.TrimEnd('\', '/')
}).Count -gt 0
if (-not $pcadAlreadyRegistered) {
    $pcadUpdatedPath = (@($pcadToolsDirectory) + $pcadPathEntries) -join ';'
    [Environment]::SetEnvironmentVariable('Path', $pcadUpdatedPath, 'User')
}
if (-not (@($env:Path -split ';' | Where-Object {
    $_.TrimEnd('\', '/') -ieq $pcadToolsDirectory.TrimEnd('\', '/')
}).Count -gt 0)) {
    $env:Path = "$pcadToolsDirectory;$env:Path"
}
Write-Host "pcad registered: $pcadToolsDirectory"
Write-Host 'Try pcad --help. Open a new terminal for other sessions to use the updated PATH.'
