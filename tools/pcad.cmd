@echo off
setlocal
where python.exe >nul 2>nul
if not errorlevel 1 goto run_python
where py.exe >nul 2>nul
if not errorlevel 1 goto run_py
echo error: Python 3 is required. Install Python and add it to PATH. 1>&2
exit /b 2

:run_python
python.exe "%~dp0pcad.py" %*
exit /b %errorlevel%

:run_py
py.exe -3 "%~dp0pcad.py" %*
exit /b %errorlevel%
