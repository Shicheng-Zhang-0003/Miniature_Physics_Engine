# readme.md

```
This is the pre-1.0 release testing edition of the Miniature Physics Engine (MPE, v0.9.7-Alpha).

It is a rigid body physics engine designed around simplicity, ease of use, and efficiency.

The core facets of designing this enigne were:
    1: Use the C programming language and the GTK3/4 library as the basis of programming this physics engine.
    2: Unless required for extremely complex tasks such as ray tracing, do not invoke OpenGL or extraneous graphics libraries.
    3: All mathematical compulation components much be programmed from scratch.

As of right now, the engine is still under development, and it is inadvisable to attempt to run using conversion software on Windows.

Engine Mechanics:
    1. Run ./engine to begin running.
    2. Clicking with the left mouse button locks mouse input onto the screen. Esc to detach cursor.
    3. Once inside the engine:
        3A: Shift Key spawns new objects. Hold down for 1.5 seconds to begin spam spawning objects.
        3B: WASD to move the camera and general viewpoint.
        3C: Pressing 0 toggles between Debug Mode and Game Mode, Debug Mode has no borders and more information presented about individual objects.
        3D: Pressing 9 gives the option to save the current state of the object layout, load a previously saved layout, or exit the engine.
        3E: Pressing 8 manipulates how objects are spawned. Prism objects spawning will be added in future versions.
        3E1: Pressing C for v0.9.7-Alpha spawns a cube, dimensions can be modified.
        3F: Right Click a object to select a object.
        3G: Once a object has been selected, press the space bar to exert a impulse/force.
        3H: Once a object has been selected, left click to delete the object.

Known Bugs:
    PSX = Programming Syntax Bug, OBP = Operational Bug, ECP = External Compatiability Bug, LIC = Library Incompleteness Bug

    PSX = Bugs or issues needing a solution in the present programming.
    OBP = bugs unpredicted in programming but recognised in operation and requires pinpointing source of error.
    ECP = Compatiability issues with different operating systems and particular rendering frameworks (EG: X11 vs Wayland, Linux Native).
    LIC = Given the minimal nature of the engine itself, realism in some cases may be severly impacted when progressing in future to certain applications and feature development.

    PSX-000 (SOLVED) (O2CL, O2 CYCLE LOOP): The current rendering of all objects is done using a O (n ^ 2) stacked for loop, which would heavily increase incremental processing time in continued usage.
        - Change In the future to a O (n) loop at best for looping or dual O (n) separated loops to prevent O (n ^ 2) repetition.
    PSX-001 (PARTIAL) (MLCULMT, MALLOC UPPER LIMIT): Objects spawned quickly exceeding a count of 1136 objects will result in the engine feeling slightly slowed down.
        - Changes in PSX-000 O (n ^ 2) loop may alleviate some of these results.
    PSX-002 (AXLLAG, AXIAL RENDER LAG): Previously, the program rendered a stational x, y, and z reference axes for each individual object. However, this severly impacted performace and was omitted temporarily with v0.9.0.1-STBL.
        - Now that rotational motion of spheres has been rectified, this is no longer of immediate concern.
        - However, in the future, for debug mode, such functionality may return, and resource allocation is of the essence to prevent high stage lag of the system.
    PSX-003 (SOLVED) (FPPSCL, FLOATING POINT PRECISION SCALE): In 0.9.5-Alpha, any form of modification to any values is += 0.01, which is far too low if not in debug mode.
        - Add a specific option for debug mode to change increment/decrement values.
        - For Game Mode, switch to a nominal +- of 0.2 for all changable constant values.
    PSX-004 (STIPLG, STARTING INPUT LAG): In 0.9.7-Alpha, after just starting the engine itself, spawning objects may require some time to load properly and actually respond.
        - Fix by finding comflicting spawning logic later, for now, just marvel at the fact that 0.9.7-Alpha was managed to be done on time in the first place.

    OPB-000 (MTNLCK, MOTION LOCK): Under continuous input, sometimes, the camera viewpoint may be stuck moving in one direction. In this case, manipulating the object into random, rapid motion counteracts and stops the uncommanded movement of the camera.
        - In some cases, save the current state of the engine, and restart the executable to solve the issue.

    ECP-000 (MSEWYLD, MOUSE WAYLAND): Under Wayland, mouse automatically disengages with out of the bounds of the window itself and does not lock properly.
        - When transitioning from IJKL perspective control, this was a major thing in versions 0.9.1 to 0.9.2.
        - Right now the thing is currently running on a forced X11 state, which works like a charm.
        - However, the clarity of the rendering and the quality of objects is absolutely horrid compared to Wayland.
        - Plus, on some newer Linux systems, they are ditching X11 permanently, so extraneous X11 support drivers may be required.

Installation Instructions:
    Refer to the installation folder for instruction details on Linux Platforms.

    Installation and conversion methods for MacOS (M-Series) and Microsoft Windows will be added in the future.

    Users may try to install dependencies as is on Intel MacOS, but please note that the only operating system that this has been officially tested on so far is Ubuntu 24.04.4 LTS.

Resource Usage:
    Nominally consumes approximately 1 Megabyte of extraneous RAM per 1136 objects spawned.
    Initial Run will result in nominal RAM usage of 105 Megabytes nominally, tested with around 1142 objects showed a total RAM consumption of 106 Megabytes.
    Note: All of this is run under X11 given that mouse locking on Wayland is deficient for the current purpose.
        - For systems forced to employ Wayland, it would be optimal to install basic X11 drivers to run this engine.
    (Core Ultra 5 125H, Intel Arc Graphics, 32GB total RAM, Ubuntu 24.04.4 LTS)
    (2880 * 1880 Screen, set to 120 Hz refresh rate (although the engine itself has a inbuilt tick rate of 60 FPS))

```
