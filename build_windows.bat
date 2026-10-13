@echo off
REM build_windows.bat — native Windows build (MSYS2 UCRT64 MinGW or mingw in PATH).
REM Run from 475-MPE\v15S\src or pass no args (script cds there).
REM DESPOT-2026-10-12 P13: this used to build headless-only + a bare
REM test_headless run (silently green when the binary was missing via stale
REM ERRORLEVEL) and never the 45-case suite, tui-smoke, check-flags, or
REM OUTDIR isolation. It now mirrors verify.sh via the Python runner.
REM DESPOT-2026-10-13: TMPDIR/TEMP/TMP now point at temp\qa_runs scratch
REM (runner-enforced isolation; was unset, so paranoia_config wrote the
REM shared temp\ and parallel runs raced). Profile stays quick: full needs
REM MSYS2 ASan/UBSan + MFS toolchain parity, tracked, not snuck in.
setlocal
if "%~1"=="" (
  cd /d "%~dp0v15S\src"
) else (
  cd /d "%~1"
)
set MPE_GAMEPAD_DEVICE=disabled
set TMPDIR=%~dp0temp
set TEMP=%~dp0temp
set TMP=%~dp0temp
where mingw32-make >nul 2>nul
if %ERRORLEVEL%==0 (
  set MAKE=mingw32-make
) else (
  set MAKE=make
)
%MAKE% build_suite
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%
test_mpe_suite.exe --all
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%
%MAKE% check-flags
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%
cd /d "%~dp0"
python tools\test_runner.py --profile quick
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%
endlocal
