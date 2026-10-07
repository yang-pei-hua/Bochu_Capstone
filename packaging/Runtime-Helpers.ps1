# Producer-side helpers. Not shipped in the end-user package.
Set-StrictMode -Version 2.0

function Assert-ChildPath([string]$Root, [string]$Path) {
    $prefix = [IO.Path]::GetFullPath($Root).TrimEnd('\', '/') + [IO.Path]::DirectorySeparatorChar
    $candidate = [IO.Path]::GetFullPath($Path)
    if (-not $candidate.StartsWith($prefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to modify a path outside $Root : $candidate"
    }
}

function Initialize-PeReader {
    if ('ParamCadPackaging.PeReader' -as [type]) { return }
    Add-Type -TypeDefinition @'
using System;
using System.IO;
using System.Collections.Generic;
using System.Runtime.InteropServices;
namespace ParamCadPackaging {
    public static class PeReader {
        [DllImport("kernel32.dll")] public static extern uint SetErrorMode(uint mode);
        static int Map(byte[] b, uint rva, int table, int count) {
            for (int i=0; i<count; i++) {
                int s=table+40*i;
                uint size=Math.Max(BitConverter.ToUInt32(b,s+8),BitConverter.ToUInt32(b,s+16));
                uint start=BitConverter.ToUInt32(b,s+12);
                if(rva>=start && rva-start<size)
                    return checked((int)(rva-start+BitConverter.ToUInt32(b,s+20)));
            }
            throw new InvalidDataException("Unmapped PE address");
        }
        static string Name(byte[] b, int p) {
            int end=p; while(end<b.Length && b[end]!=0) end++;
            return System.Text.Encoding.ASCII.GetString(b,p,end-p);
        }
        public static string[] Imports(string path) {
            byte[] b=File.ReadAllBytes(path);
            int pe=BitConverter.ToInt32(b,0x3c);
            if(BitConverter.ToUInt32(b,pe)!=0x4550 || BitConverter.ToUInt16(b,pe+4)!=0x8664)
                throw new InvalidDataException("Expected a Windows x64 PE file: "+path);
            int count=BitConverter.ToUInt16(b,pe+6), optional=pe+24;
            if(BitConverter.ToUInt16(b,optional)!=0x20b) throw new InvalidDataException("Expected PE32+");
            int sections=optional+BitConverter.ToUInt16(b,pe+20), directories=optional+112;
            ulong imageBase=BitConverter.ToUInt64(b,optional+24);
            var names=new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            uint importRva=BitConverter.ToUInt32(b,directories+8);
            if(importRva!=0) {
                int p=Map(b,importRva,sections,count);
                while(BitConverter.ToUInt32(b,p+12)!=0) {
                    names.Add(Name(b,Map(b,BitConverter.ToUInt32(b,p+12),sections,count))); p+=20;
                }
            }
            uint delayRva=BitConverter.ToUInt32(b,directories+13*8);
            if(delayRva!=0) {
                int p=Map(b,delayRva,sections,count);
                while(BitConverter.ToUInt32(b,p+4)!=0) {
                    uint name=BitConverter.ToUInt32(b,p+4);
                    if((BitConverter.ToUInt32(b,p)&1)==0) name=checked((uint)(name-imageBase));
                    names.Add(Name(b,Map(b,name,sections,count))); p+=32;
                }
            }
            var result=new string[names.Count]; names.CopyTo(result); return result;
        }
    }
}
'@
}

function Test-SystemImport([string]$Name) {
    if ($Name -match '^(api-ms-|ext-ms-)') { return $true }
    # Installed developer CRTs in System32 must not hide a missing app-local CRT.
    if ($Name -match '^(msvcp\d|msvcr\d|vcruntime|concrt)') { return $false }
    return (Test-Path -LiteralPath (Join-Path $env:SystemRoot "System32\$Name") -PathType Leaf)
}

function Assert-AppRuntime([string]$AppDirectory, [string[]]$SearchDirectories = @(), [switch]$TrimUnused) {
    Initialize-PeReader
    # Audit only the staged GUI binaries. SDK source trees stay opaque.
    $queue = New-Object 'Collections.Generic.Queue[string]'
    if ($TrimUnused) {
        $queue.Enqueue((Join-Path $AppDirectory 'ParamCAD Studio.exe'))
        Get-ChildItem -LiteralPath (Join-Path $AppDirectory 'plugins') -Recurse -File -Filter '*.dll' | ForEach-Object { $queue.Enqueue($_.FullName) }
        # Software rendering and shader compilers are loaded dynamically by Qt.
        # Keep the full redistributable CRT alongside the client as well.
        Get-ChildItem -LiteralPath $AppDirectory -File -Filter '*.dll' | Where-Object {
            $_.Name -match '^(msvcp\d|vcruntime|concrt|gmp-10\.|opengl32sw\.|D3Dcompiler_47\.|dxcompiler\.|dxil\.)'
        } | ForEach-Object { $queue.Enqueue($_.FullName) }
    } else {
        Get-ChildItem -LiteralPath $AppDirectory -Recurse -File | Where-Object {
            $_.Extension -in @('.dll', '.exe')
        } | ForEach-Object { $queue.Enqueue($_.FullName) }
    }
    $checked = New-Object 'Collections.Generic.HashSet[string]'
    while ($queue.Count -gt 0) {
        $binary = $queue.Dequeue()
        if (-not $checked.Add($binary)) { continue }
        foreach ($name in [ParamCadPackaging.PeReader]::Imports($binary)) {
            if (Test-SystemImport $name) { continue }
            $target = Join-Path $AppDirectory $name
            if (-not (Test-Path -LiteralPath $target -PathType Leaf)) {
                $source = $null
                foreach ($directory in $SearchDirectories) {
                    $candidate = Join-Path $directory $name
                    if (Test-Path -LiteralPath $candidate -PathType Leaf) { $source = $candidate; break }
                }
                if ($null -eq $source) { throw "Missing runtime $name (imported by $binary)" }
                Copy-Item -LiteralPath $source -Destination $target
                Write-Host "Bundled missing runtime: $name"
            }
            $queue.Enqueue($target)
        }
    }
    if ($TrimUnused) {
        # The developer build stages the entire OCCT bin directory, including
        # Draw/Tcl developer tools. Only ship the verified runtime closure.
        Get-ChildItem -LiteralPath $AppDirectory -File -Filter '*.dll' | ForEach-Object {
            if (-not $checked.Contains($_.FullName)) {
                Assert-ChildPath $AppDirectory $_.FullName
                Remove-Item -LiteralPath $_.FullName
            }
        }
    }
    return $checked.Count
}

function Invoke-IsolatedRuntimeValidation([string]$Root, [switch]$SkipGui) {
    $app = Join-Path $Root 'app'
    $python = Join-Path $Root 'deps\python\python.exe'
    $colmap = Join-Path $Root 'deps\colmap\bin\colmap.exe'
    $script = Join-Path $Root 'core\reconstruction\reconstruct.py'
    foreach ($file in @((Join-Path $app 'ParamCAD Studio.exe'), $python, $colmap, $script,
                        (Join-Path $app 'plugins\platforms\qwindows.dll'))) {
        if (-not (Test-Path -LiteralPath $file -PathType Leaf)) { throw "Missing packaged file: $file" }
    }
    $names = @('PATH','QTDIR','QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH','QT_QPA_PLATFORM',
               'QML2_IMPORT_PATH','PYTHONPATH','PYTHONHOME','COLMAP_EXECUTABLE','PYTHONDONTWRITEBYTECODE')
    $saved = @{}
    foreach ($name in $names) { $saved[$name] = [Environment]::GetEnvironmentVariable($name,'Process') }
    try {
        foreach ($name in $names) { [Environment]::SetEnvironmentVariable($name,$null,'Process') }
        $env:PATH = "$env:SystemRoot\System32;$env:SystemRoot"
        $env:PYTHONDONTWRITEBYTECODE = '1'
        $code = "import sys, sqlite3, ssl, subprocess; from pathlib import Path; from colmap_reconstruction.pipeline import find_colmap_executable; p=find_colmap_executable(); assert p == Path(sys.argv[1]), p; print('Bundled Python and reconstruction modules OK')"
        & $python -c $code $colmap
        if ($LASTEXITCODE -ne 0) { throw 'Bundled Python or COLMAP discovery failed' }
        & $python $script --help | Out-Null
        if ($LASTEXITCODE -ne 0) { throw 'Bundled reconstruction CLI failed' }
        & $colmap version
        if ($LASTEXITCODE -ne 0) { throw 'Bundled COLMAP failed with a clean PATH' }
        if (-not $SkipGui) {
            Initialize-PeReader
            $previous = [ParamCadPackaging.PeReader]::SetErrorMode(0x8003)
            $process = $null
            try {
                $process = Start-Process -FilePath (Join-Path $app 'ParamCAD Studio.exe') -WorkingDirectory $Root -WindowStyle Hidden -PassThru
                if ($process.WaitForExit(4000)) { throw "GUI startup failed: exit $($process.ExitCode)" }
                if (-not $process.Responding) { throw 'Packaged GUI is not responding' }
                Write-Host 'Packaged GUI stayed running and responsive with a clean PATH'
            } finally {
                if ($null -ne $process -and -not $process.HasExited) { $process.Kill(); $process.WaitForExit() }
                [void][ParamCadPackaging.PeReader]::SetErrorMode($previous)
            }
        }
    } finally {
        foreach ($name in $names) { [Environment]::SetEnvironmentVariable($name,$saved[$name],'Process') }
    }
}
