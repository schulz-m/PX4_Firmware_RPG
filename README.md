General information can be found on the [PX4 Developer Page](http://pixhawk.org/dev/start)

Toolchain
=========

You find details about the toolchain installation [here](http://pixhawk.org/dev/toolchain_installation_lin).

To build the firmware you need to clone the NuttX repo into the Firmware folder.

* `git clone https://github.com/PX4/NuttX`

Building
========

When building for the first time:

* `make archives`

When compiling the RPG firmware

* `make px4fmu-v1_rpg`

Uploading the Firmware

* `make upload px4fmu-v1_rpg`


