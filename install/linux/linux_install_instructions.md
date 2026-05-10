# linux_install_instructions.md

```
Linux Installation Instructions:

Required Software Stack:
        GTK3/4: libgtk-3-dev, libgtk-4-dev.
        Epoxy for OpenGL: libepoxy-dev
        GNU C Math Library: libm, part of libc6-dev
        GNU Compiler Collection: gcc, v12+ recommended
        Make: make
        pkg-config: Configuring Linker Flags for building executables

Installation Instructions (Prerequisite Packages):
    Ubuntu/Debian/Debian Derivatives (apt):
        sudo apt update --> update system
        sudo apt install build-essential pkg-config libgtk-3-dev libepoxy-dev
    Fedora (dnf, yum?):
        sudo dnf update
        sudo dnf install @development-tools pkgconf-pkg-config gtk3-devel libepoxy-devel
    Arch/Manjaro/Arch Derivatives (pacman):
        sudo pacman -Syu
        sudo pacman -S base-devel pkgconf gtk3 libepoxy
    SUSE Derivatives (zypper):
        sudo zypper install -t pattern devel_basis pkg-config gtk3-devel libepoxy-devel
    Alpine (apk):
        sudo apk add build-base pkgconf gtk+3.0-dev libepoxy-dev
    Gentoo (portage):
        sudo emerge --ask sys-devel/base-system dev-util/pkgconf x11-libs/gtk+:3 media-libs/libepoxy
    Nix (source compilation):
        nix-shell -p gcc pkg-config gtk3 libepoxy

To Check if dependency libraries are actually detected:
    On most Linux system, if you have pkg-config installed, run:
        $ pkg-config --cflags --libs gtk+-3.0 epoxy
    To check dependency resolution

After dependencies have been installed:
    git clone https://github.com/shicheng-zhang/physics-engine.git --> This gets the actual source code.
    Go to the src/ folder --> where all of the main code is actually stored.
    Run: make clean; make;
        - This makes a new compilation of the source code run using your system's specifications.
        - Usually nothing, but for older systems gcc optimisations may be in consideration
    If you see a bunch of warnings from gcc, do not worry.
    The only time you should be worried is if you see make: Error at the end of the compilation
    However, my own testing often reveals such issues, so theoretically this should only happen if you didn't install a dependency properly.


```