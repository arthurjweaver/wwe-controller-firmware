### Background
This repository contains firmware for the [Weaver Wind Energy](https://www.arthurjweaver.net/weaver-wind-energy/) wind turbine controller
built around the [Arduino Due](https://store-usa.arduino.cc/products/arduino-due?selectedStore=us) microcontroller (Amtel SAM3X8E ARM Cortex-M3 CPU). The firmware evolved along with the small wind turbines designed and built by WWE between 2013 and 2021. Although WWE is no longer in business, the code continues to be maintained for existing turbines and for potential use of the controller in other applications. 

Possible alternative uses include _any data logging or control application_ requiring:
- Real-time, high-resolution (up to 10000 Hz) voltage and current sensing from a 3-phase alternator
- Real-time, high-resolution (up to 10000 Hz) control of a 4-wire [stepper motor](https://www.anaheimautomation.com/products/stepper/stepper-motor-item.php?sID=42&pt=i&tID=81&cID=19)
- Real-time control of an [ABB contactor](https://new.abb.com/products/1SBL177501R1100/af16-22-00-11)
- Real-time sensing of a low-voltage limit switch
- Modbus/RTU or TCP communication with [Morningstar](https://www.morningstarcorp.com/) charge controllers
- Modbus/RTU or TCP communication with a [Nuvation](https://www.nuvationenergy.com/) battery management system (BMS)
- Response to near real-time weather conditions (local or otherwise)

The WWE controller consists of a Logic Board hosting an Arduino Due with an [Ethernet Shield 2](https://store-usa.arduino.cc/products/arduino-ethernet-shield-2?selectedStore=us) and a Power Board that manages turbine power flow. These boards were integrated into a larger enclosure to accommodate turbine wiring as well as additional devices needed for turbine operation (anemometry, stepper motor driver). However, only the two circuit boards (without any connections to external hardware) are required for the firmware to run. Feel free to contact me at _arthur.jensen.weaver@gmail.com_ regarding circuit board availability or schematics.

### Implementation notes

#### Arduino IDE
Version 1.8.19 of the [Arduino IDE](https://www.arduino.cc/en/software) was used to compile and export the binaries. Symlinks pointing to short versions of the binary file names are included in the repository. These links are *required* because only the shortened binary file names are referenced by the bootloader code.
```
$ ln -s wwe.ino.arduino_due_x.bin wwe.bin 
$ ln -s updatefw.ino.arduino_due_x.bin updatefw.bin
```
On a development machine with the Arduino IDE, permissions for all files in the working directory should be 755.

`wwe.ino` is the _main_ module for all firmware code _except_ `updatefw.ino`. The beginning of `wwe.ino` contains extensive implementation notes. All code modules are extensively commented.

`updatefw.ino` is a bootloader program used to automate firmware updates over the web. This file also contains implementation notes.

#### Arduino libraries
Libraries used by this code are _not_ provided in this repository and must be downloaded either from the Arduino IDE or manually. Many are hosted on GitHub. Usage notes for all included libraries can be found in the `wwe.ino` preamble comments. A few libraries used by `wwe.ino` and `updatefw.ino` require minor modifications which are also documented in `wwe.ino`.

#### Known issues

##### Ethernet
The firmware uses Ethernet to:
- send Network Time Protocal (NTP) UDP requests (port 123) - once, in setup()
- send Modbus/TCP requests to a Nuvation BMS (port 502) - once per second
- send system data via UDP to a data server (ports 58328, 58329, 58330, 58332) - once per second
- send system configuration via UDP to an update server (port 58331) - once every few minutes
- send HTTPS requests to the [National Weather Service API](https://www.weather.gov/documentation/services-web-api) (port 443) - once per hour
- send HTTP requests to a firmware update server (port 49152) - as needed

Rarely, during Ethernet access, the code has been observed to hang, and recovers only when the watchdog timer times out. Efforts to track down this known problem are currently focused on dealing with "stuck sockets":
- https://forum.arduino.cc/t/dealing-with-lost-arduino-ethernet-connectivity-stuck-sockets/244055/12
- https://github.com/arduino-libraries/Ethernet/issues/82

Fortunately, there appear to be viable workarounds which will be implemented in the next update and noted here.



