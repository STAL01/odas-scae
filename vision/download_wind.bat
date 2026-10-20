@echo off
setlocal
REM HAB-RECON wind forecast for offline demos - double-click this file (needs internet).
REM Downloads the NOAA GFS forecast for the next 5 days around Sofia into vision\wind_cache,
REM so the dashboard's "GFS forecast" wind and the swarm work later without internet.
REM Other place or time:  download_wind.bat --lat 43.7 --lon 25.9 --start "2026-10-25 06:00" --hours 48
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
echo Downloading the NOAA GFS wind forecast (about 40 files, a few minutes).
echo Keep this window open.
echo.
if "%~1"=="" ("%VPY%" gfs_wind.py --hours 120) else ("%VPY%" gfs_wind.py %*)
if errorlevel 1 (
  echo.
  echo  Something went wrong - see the message above.
)
echo.
echo The forecast is kept in vision\wind_cache. In the dashboard choose "Wind data: GFS forecast".
pause
