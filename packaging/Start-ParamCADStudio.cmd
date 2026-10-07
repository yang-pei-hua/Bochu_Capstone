@echo off
setlocal
set QT_PLUGIN_PATH=
set QT_QPA_PLATFORM_PLUGIN_PATH=
set QML2_IMPORT_PATH=
if not exist "%~dp0app\ParamCAD Studio.exe" (
  echo ParamCAD Studio is missing. Extract the complete release archive first. 1>&2
  exit /b 2
)
start "" /D "%~dp0" "%~dp0app\ParamCAD Studio.exe" %*
exit /b %errorlevel%
