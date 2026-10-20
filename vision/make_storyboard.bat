@echo off
setlocal
REM HAB-RECON recon-demo storyboard with real satellite pictures - double-click this file.
REM It photographs every stop in demo_route.json and opens recon_storyboard.html.
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
echo Taking the satellite picture of every stop and running the detector.
echo This takes a few minutes the first time. Keep this window open.
echo.
"%VPY%" make_storyboard.py %*
if errorlevel 1 (
  echo.
  echo  Something went wrong - see the message above.
)
echo.
echo The page is vision\recon_storyboard.html - it opens in your browser by itself.
pause
