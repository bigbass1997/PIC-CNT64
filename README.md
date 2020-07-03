[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
### Description
The PIC-CNT64 project is a modern replacement/alternative for the CNT-NUS IC chip used in the Nintendo 64 standard controller. The CNT-NUS chip is used as a serial encoder/decoder to communicate with the N64 console. It allows for sending the state of the controller inputs upon request. However, the N64 controller was also designed with extra capabilities via an expansion port. The port was primarily used for the Rumble PAK, Controller PAK, and the Transfer PAK.

Using a PIC18LF4550 microcontroller, it is now possible to send inputs from real hardware, to the console, without using any original hardware. As far as I could find from extensive research, up until this point, all homebrew controllers and portables, still used the original CNT-NUS chip. This was done by cutting the PCB around it, desoldering the entire chip, or soldering wires directly to the original PCB.

This project will hopefully alleviate the need to tear apart original hardware, and instead use a widely produced microcontroller.

_PIC-CNT64 is still a Work-In-Progress, however, the functionality does work for both button inputs and the analog stick (which requires special interrupt handling as it isn't as simple as a binary switch). Accessory PAK's are not yet implemented, and may require a different MCU with better capabilities._

### Documentation
All documentation can be found on the [wiki](https://github.com/bigbass1997/PIC-CNT64/wiki). Wiring (KiCad) schematics are found in `/pcb-schematic/` in the repo.

### Discord/Support
If you have questions or suggestions, you can find me on the N64 homebrew Discord server https://discord.gg/KERXwaT