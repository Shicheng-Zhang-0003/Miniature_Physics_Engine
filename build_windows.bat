@echo off
REM build_windows.bat — native Windows build (MSYS2 UCRT64 MinGW or mingw in PATH).
REM Run from 475-MPE\v15S\src or pass no args (script cds there).
setlocal
if "%~1"=="" (
  cd /d "%~dp0v15S\src"
) else (
  cd /d "%~1"
)
set MPE_GAMEPAD_DEVICE=disabled
where mingw32-make >nul 2>nul
if %ERRORLEVEL%==0 (
  set MAKE=mingw32-make
) else (
  set MAKE=make
)
%MAKE% headless
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%
if exist test_headless.exe test_headless.exe
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%
bash ecosystem\mfs\build_tests.sh
endlocal
