@echo off
rem Double-click to log COM9 @ 115200 into a timestamped CSV next to this file.
rem Ctrl+C or close the window to stop. Use the first argument to pick another port.
set PORT=COM9
if not "%1"=="" set PORT=%1
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0log_serial.ps1" -Port %PORT% -Baud 115200
pause
