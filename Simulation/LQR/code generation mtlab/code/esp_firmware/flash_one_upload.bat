@echo off
setlocal

if "%~1"=="" (
  echo Usage: flash_one_upload.bat COMx [esp32^|esp32s3^|esp32c3]
  exit /b 1
)

set PORT=%~1
set TARGET=%~2
if "%TARGET%"=="" set TARGET=esp32

echo [1/2] Setting target to %TARGET%
idf.py set-target %TARGET%
if errorlevel 1 exit /b 1

echo [2/2] Building, flashing, and opening monitor on %PORT%
idf.py -p %PORT% build flash monitor
exit /b %errorlevel%