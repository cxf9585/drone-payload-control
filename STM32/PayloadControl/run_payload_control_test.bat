@echo off
set PATH=C:\msys64\ucrt64\bin;%PATH%
cd /d "%~dp0"
build\payload_control_test.exe
echo.
echo Exit code: %ERRORLEVEL%
pause
