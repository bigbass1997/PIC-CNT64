[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
### Description
The PIC-CNT64 project is a modern replacement/alternative for the CNT-NUS IC chip used in the Nintendo 64 standard controller. The CNT-NUS chip is used as a serial encoder/decoder to communicate with the N64 console. In addition to sending the state of the button inputs and analog stick, the N64 controller was also designed with extra capabilities via an expansion port. The port was primarily used for the Rumble PAK, Controller PAK, and the Transfer PAK.

Using a PIC18LF47K42 microcontroller, it is possible to send inputs from real hardware, to the console, without using any original hardware. As far as I could find from extensive research, up until this point, all homebrew controllers and portables, still used the original CNT-NUS chip. This was done by cutting the PCB around it, desoldering the entire chip, or soldering wires directly to the original PCB.

This project will hopefully alleviate the need to tear apart original hardware, by using a widely produced microcontroller instead.

_PIC-CNT64 is still a Work-In-Progress, however, the functionality does work for both button inputs and the analog stick. Accessory PAK's are partially implemented but not completely working yet._

### Documentation
All documentation can be found on the ~~[wiki](https://github.com/bigbass1997/PIC-CNT64/wiki)~~ (while this still exists, most documentation is being migrated to https://n64brew.dev/). KiCad schematics will be found in `/schematics/` in the repo.

### Discord/Support
If you have questions or suggestions, you can find me on the N64 homebrew Discord server https://discord.gg/KERXwaT