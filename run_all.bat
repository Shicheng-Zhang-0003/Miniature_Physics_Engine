@echo off
REM run_all.bat — native Windows wrapper for run_all.sh (MSYS2 bash preferred).
REM Usage: run_all.bat [--profile quick|physics|full] [extra args...]
setlocal
set ROOT=%~dp0
set TMPDIR=%ROOT%temp
set MPE_GAMEPAD_DEVICE=disabled
if not exist "%TMPDIR%" mkdir "%TMPDIR%"
where bash >nul 2>nul
if %ERRORLEVEL%==0 (
  bash "%ROOT%run_all.sh" %*
) else (
  python "%ROOT%tools\test_runner.py" --profile full %*
)
endlocal
