@echo off
REM verify.bat — native Windows wrapper for verify.sh
REM Usage: verify.bat [--profile quick|physics|full] [--test name] [...]
setlocal
set ROOT=%~dp0
set TMPDIR=%ROOT%temp
set MPE_GAMEPAD_DEVICE=disabled
if not exist "%TMPDIR%" mkdir "%TMPDIR%"
where bash >nul 2>nul
if %ERRORLEVEL%==0 (
  bash "%ROOT%verify.sh" %*
) else (
  python "%ROOT%tools\test_runner.py" %*
)
endlocal
