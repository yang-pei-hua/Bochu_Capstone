@echo off
setlocal
rem Let Windows PowerShell use its own modules when launched from PowerShell 7.
set PSModulePath=
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0installer\Install-ParamCADStudio.ps1" %*
exit /b %errorlevel%
