Overview
--------

This project contains evaluation-related material for busybox. It contains the `ifdeftoifStatistics.sh` script which starts the `#ifdef` to `if` process on each BusyBox file.

It does include BusyBox itself, header files and the KBuildMiner project.


Preparation
-----------
HELLO WORLD
1. `TypeChef` and `Hercules` folder must be at ".." relative to this project folder; see project [Hercules](https://github.com/joliebig/Hercules) to install both `TypeChef` and `Hercules`.
2. Run prepareBusybox.sh to automatically download missing source files, system headers and create the required .pc files.
3. Execute the busybox transformation with `./ifdeftoifStatistics.sh`.

Good luck. In case of problems contact someone of us.

Notes
-----------
`busybox/config.h` has been altered. The following features have been disabled: `CONFIG_DESKTOP`, `CONFIG_FEATURE_INDIVIDUAL`.
These features are disabled because they are the cause of variability in functions which have to be externally linked for building `BusyBox`.
By removing the variability from these features we avoid duplicating and this renaming functions which have to be visible in other files.