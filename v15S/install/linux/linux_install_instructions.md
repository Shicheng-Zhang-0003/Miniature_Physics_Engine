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
    git clone <repository-url> --> This gets the actual source code.
    Go to the src/ folder --> where all of the main code is actually stored.
    Run make
        - This makes a new compilation of the source code run using your system's specifications.
        - Especially now that I have added -O3 into compilation flags.
        - Usually nothing, but for older systems gcc optimisations may be in consideration
    The only time you should be worried is if you see make: Error at the end of the compilation
    However, my own testing often reveals such issues, so theoretically this should only happen if you didn't install a dependency properly.


```
