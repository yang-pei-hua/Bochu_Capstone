@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Install-ParamCADStudio.ps1" %*
exit /b %errorlevel%
