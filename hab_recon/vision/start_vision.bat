@echo off
setlocal
REM HAB-RECON vision server for Windows - just double-click this file.
REM First run: creates a virtual environment in hab_recon\.venv and installs
REM the packages there (nothing is installed into your main Python).
cd /d "%~dp0"
set "VENV=%~dp0..\.venv"
set "VPY=%VENV%\Scripts\python.exe"

if exist "%VPY%" goto run

set "PY="
py -3 --version >nul 2>nul && set "PY=py -3"
if not defined PY (python --version >nul 2>nul && set "PY=python")
if not defined PY (
  echo.
  echo  Python was not found.
  echo  Install Python 3.12 or 3.13 from https://www.python.org/downloads/windows/
  echo  and tick "Add python.exe to PATH" on the first installer screen.
  echo  Then double-click this file again.
  echo.
  pause
  exit /b 1
)
echo Creating the virtual environment in hab_recon\.venv ...
%PY% -m venv "%VENV%"
if errorlevel 1 (
  echo.
  echo  Could not create the virtual environment - see the message above.
  pause
  exit /b 1
)

:run
echo Checking the packages in .venv (the first time this downloads about 35 MB)...
"%VPY%" -m pip install --disable-pip-version-check -q -r requirements.txt
if errorlevel 1 (
  echo.
  echo  Installing the packages failed - see the message above.
  pause
  exit /b 1
)

echo.
echo Starting the vision server. Keep this window open - close it to stop.
"%VPY%" vision_server.py %*
pause
