Overview
--------

This project contains evaluation-related material for busybox. It contains the scripting to run the TypeChef parser and type system on Busybox.

It does include Busybox itself, header files and the KBuildMiner project.


Preparation
-----------
Clone this project and the TypeChef project from https://github.com/aJanker/TypeChef/
Run the mkrun_ifdeftoif.sh in the TypeChef project.

Run prepareBusybox.sh to automatically download missing source files, system headers and create the required .pc files. Execute the busybox transformation with `./ifdeftoifStatistics.sh`.

Good luck. In case of problems contact someone of us.