# Windows Installation Instructions (MSYS2 + native)

```
Windows Installation Instructions (MSYS2 UCRT64 + native .exe/.dll):

This tree is fully Windows-compatible with NO Linux functionality removed.
Linux builds are unchanged; Windows paths are additive (#ifdef _WIN32).

1. Required: MSYS2 (provides bash, make, MinGW-w64 GCC, and UNIX tools)
   - Download from https://www.msys2.org/ and install (default C:\\msys64).
   - Open "MSYS2 UCRT64" terminal (NOT "MSYS" — UCRT64 gives native Windows
     binaries with modern CRT). Update once:
       pacman -Syu
     (close and reopen if asked, then `pacman -Su`).

2. Toolchain + deps (UCRT64 terminal):
     pacman -S --needed mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-make \
       mingw-w64-ucrt-x86_64-pkgconf mingw-w64-ucrt-x86_64-gtk4 \
       mingw-w64-ucrt-x86_64-libepoxy mingw-w64-ucrt-x86_64-ncurses \
       make pkgconf git
   Notes:
     - Headless + MFS robotics need ONLY gcc/make (no GTK/ncurses).
       GTK4/epoxy are for the full `engine` GUI; ncurses is for `mpe-tui`.
     - XInput (gamepad) ships with Windows SDK / MinGW (xinput.h + -lxinput);
       no extra install. Set MPE_GAMEPAD_DEVICE=disabled for headless.
     - Native Windows compiling = MinGW GCC producing native .exe/.dll that
       run WITHOUT MSYS2 (libgcc/winpthread are static-linked by our
       makefiles; system libs winmm/xinput/ws2_32 stay dynamic as they ship
       with Windows).

3. Build (UCRT64 terminal, from v15S/src/):
     cd /path/to/475-MPE/v15S/src
     # Headless smoke (no display, no GTK):
     make headless && ./test_headless.exe
     # Canonical physics suite (needs epoxy for spring tests):
     make build_suite && ./test_mpe_suite.exe --all
     # Robotics suite (MFS: builds mpe_ftc.dll + mfs_suite.exe, runs 8 tests):
     ecosystem/mfs/build_tests.sh
     # Full verification (Python harness, MSYS2 bash + python3):
     python3 ../../tools/test_runner.py --profile quick
     python3 ../../tools/test_runner.py --profile full   # includes ASan skip on Windows
     # GUI engine (needs GTK4/epoxy/X11-free):
     make engine && ./engine.exe
     # TUI debugger (needs ncurses + epoxy):
     make mpe-tui && ./mpe-tui.exe --snapshot 60 --scene demo

4. Native cmd.exe (without MSYS2 bash) — optional:
   - Add C:\\msys64\\ucrt64\\bin to PATH (for libwinpthread/gcc runtime if
     you built without static flags; our defaults static-link so not needed).
   - Use the provided .bat wrappers from the repo root:
       run_all.bat       # == run_all.sh --profile full
       verify.bat --profile quick
   - Or build directly with mingw32-make:
       mingw32-make -C v15S\\src headless
       v15S\\src\\test_headless.exe

5. Cross-compiling from Linux (for CI):
     sudo apt install mingw-w64 wine64
     cd 475-MPE/v15S/src
     make build_two_world CC=x86_64-w64-mingw32-gcc MPE_WINDOWS=1
     wine ./test_two_world.exe
     OUTDIR=/tmp/win MFS_TEST_CC=x86_64-w64-mingw32-gcc MPE_WINDOWS=1 \
       ecosystem/mfs/build_tests.sh   # runs suite via wine automatically

6. Troubleshooting:
   - `pkg-config --exists gtk4 epoxy` fails -> install gtk4/epoxy packages
     above, or build headless/MFS only (they don't need GTK).
   - `X11/Xlib.h not found` -> you are building GUI without MSYS2 gtk;
     headless/MFS don't include X11 (guarded by GDK_WINDOWING_X11).
   - `ncurses.h not found` -> install ncurses package, or skip mpe-tui.
   - `libwinpthread-1.dll missing` under wine -> our makefiles static-link
     it; if you built manually without those flags, copy
     /usr/x86_64-w64-mingw32/lib/libwinpthread-1.dll next to the .exe.
   - `path must resolve inside plugins/<name>.dll` -> run from v15S/src
     (loader jail is CWD-relative, same as Linux).
   - Gamepad: default xinput:0; set MPE_GAMEPAD_DEVICE=disabled on headless
     (scripts do this). F310 switch must be on X for XInput mode.

7. What changed for Windows (additive, Linux untouched):
   - New core/mpe_platform.h (+ mfs_platform.h mirror): pthread/dlopen/
     file/time/string shims, MPE_USED/WEAK/CTOR/DTOR, .dll/.exe macros.
   - gamepad: Linux js unchanged; Windows XInput backend + disabled stub.
   - mpe_loader: accepts .so+.dll, '/'■'\\', case-insensitive; MSVC ctor
     fallback via mpe_capsule_init/fini.
   - makefiles/build_tests.sh/test_runner.py: PLUGIN_EXT/EXE_EXT, no
     -rdynamic/-ldl on Windows, static libgcc/winpthread, wine-aware runs.
```
