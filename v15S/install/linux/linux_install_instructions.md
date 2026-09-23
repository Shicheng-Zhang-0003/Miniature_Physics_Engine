# linux_install_instructions.md

```
Linux Installation Instructions:

Required Software Stack:
        GTK4: libgtk-4-dev. (GTK3 was the v15R3 stack.)
        Epoxy for OpenGL: libepoxy-dev
        GNU C Math Library: libm, part of libc6-dev
        GNU Compiler Collection: gcc, v12+ recommended
        Make: make
        pkg-config: Configuring Linker Flags for building executables

Installation Instructions (Prerequisite Packages):
    Ubuntu/Debian/Debian Derivatives (apt):
        sudo apt update --> update system
        sudo apt install build-essential pkg-config libgtk-4-dev libepoxy-dev
        sudo apt install libncurses-dev --> only needed for the mpe-tui terminal debugger
    Fedora (dnf, yum?):
        sudo dnf update
        sudo dnf install @development-tools pkgconf-pkg-config gtk4-devel libepoxy-devel ncurses-devel --> ncurses for mpe-tui
    Arch/Manjaro/Arch Derivatives (pacman):
        sudo pacman -Syu
        sudo pacman -S base-devel pkgconf gtk4 libepoxy ncurses --> ncurses for mpe-tui
    SUSE Derivatives (zypper):
        sudo zypper install -t pattern devel_basis pkg-config gtk4-devel libepoxy-devel ncurses-devel --> ncurses for mpe-tui
    Alpine (apk):
        sudo apk add build-base pkgconf gtk4.0-dev libepoxy-dev ncurses-dev --> ncurses for mpe-tui
    Gentoo (portage):
        sudo emerge --ask sys-devel/base-system dev-util/pkgconf gui-libs/gtk:4 media-libs/libepoxy sys-libs/ncurses --> ncurses for mpe-tui
    Nix (source compilation):
        nix-shell -p gcc pkg-config gtk4 libepoxy ncurses --> ncurses for mpe-tui

To Check if dependency libraries are actually detected:
    On most Linux system, if you have pkg-config installed, run:
        $ pkg-config --cflags --libs gtk4 epoxy
    To check dependency resolution

After dependencies have been installed:
    git clone <repository-url> --> Replace with the actual repository URL.
    Go to the v15S/src/ folder --> where all of the main code is actually stored.
    Run make
        - This compiles the engine with -O3 (override with CFLAGS=... on older systems).
        - Only worry if you see make: Error at the end of the compilation,
          which usually means a dependency is missing.
    Headless regression suite (no display needed):
        make build_suite && ./test_mpe_suite --all
        (expects 32/32 green: 29 physics + 3 diag-informational)
    Robotics suite:
        ecosystem/mfs/build_tests.sh
        (expects 8 gated green + 5 informational diags)


```
