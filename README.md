## Background
This repository contains firmware for the [Weaver Wind Energy](https://www.arthurjweaver.net/weaver-wind-energy/) wind turbine controller
built around the [Arduino Due](https://store-usa.arduino.cc/products/arduino-due?selectedStore=us) microcontroller (Amtel SAM3X8E ARM Cortex-M3 CPU). The firmware evolved along with the small wind turbines designed and built by WWE between 2013 and 2021. Although WWE is no longer in business, the code continues to be maintained for existing turbines and particulary for *potential use of the controller in other applications.* 

Possible alternative uses include _any data logging or control application_ requiring:
- Real-time, high-resolution (up to 10000 Hz) voltage and current (waveform) sensing from a 3-phase alternator
- Real-time, high-resolution (up to 10000 Hz) control of a 4-wire [stepper motor](https://www.anaheimautomation.com/products/stepper/stepper-motor-item.php?sID=42&pt=i&tID=81&cID=19) (control mechanism)
- Real-time, high-resolution (up to 10000 Hz) control of a load resistor (for diversion of alternator power)
- Real-time control of an [ABB contactor](https://new.abb.com/products/1SBL177501R1100/af16-22-00-11) (safety mechanism)
- Real-time sensing of a low-voltage limit switch (safety mechanism)
- Modbus/RTU or TCP communication with [Morningstar](https://www.morningstarcorp.com/) charge controllers
- Modbus/RTU or TCP communication with a [Nuvation](https://www.nuvationenergy.com/) battery management system (BMS)
- Response to near real-time weather conditions (local or otherwise)

The WWE controller consists of a Logic Board hosting an Arduino Due with an [Ethernet Shield 2](https://store-usa.arduino.cc/products/arduino-ethernet-shield-2?selectedStore=us) and a Power Board that manages turbine power flow. These boards were integrated into a larger enclosure to accommodate turbine wiring as well as additional devices needed for turbine operation (anemometry, stepper motor driver). However, only the two circuit boards (without any connections to external hardware) are required for the firmware to run. Feel free to contact me at _arthur.jensen.weaver@gmail.com_ regarding circuit board availability or schematics.

## Implementation notes

### Arduino IDE
Version 1.8.19 of the [Arduino IDE](https://www.arduino.cc/en/software) was used to compile and export the binaries. Symlinks pointing to short versions of the binary file names are included in the repository. These links are *required* because only the shortened binary file names are referenced by the bootloader code.
```
$ ln -s wwe.ino.arduino_due_x.bin wwe.bin 
$ ln -s updatefw.ino.arduino_due_x.bin updatefw.bin
```
On a development machine with the Arduino IDE, permissions for all files in the working directory should be set to 755.

`wwe.ino` is the _main_ module for all firmware code _except_ `updatefw.ino`. The beginning of `wwe.ino` contains extensive implementation notes. All code modules are also extensively commented.

`updatefw.ino` is a separate bootloader program used to automate firmware updates over the web. This file also contains implementation notes.

### Arduino libraries
Libraries used by this code are _not_ provided in this repository and must be downloaded either from the Arduino IDE or manually. Many are hosted on GitHub. Usage notes for all included libraries can be found in the `wwe.ino` preamble comments. A few libraries used by `wwe.ino` and `updatefw.ino` require minor modifications which are also documented in `wwe.ino`. Finally, there may be a few libraries which are no longer supported or that have been superseded by other libraries of the same name with incompatible code. Those libraries may be included in this repository at some point (or by request).

### Known issues

#### Ethernet
The firmware uses Ethernet to:
- send Network Time Protocal (NTP) UDP requests (port 123) - once, in `setup()`
- send Modbus/TCP requests to a Nuvation battery management system (port 502) - once per second
- send system data via UDP to a Data Server (ports 58328, 58329, 58330, 58332) - once per second
- send system configuration via UDP to an Update Server (port 58331) - once every few minutes
- send HTTP requests for firmware updates to an Update Server (port 49152) - as needed
- send HTTPS requests to the [National Weather Service API](https://www.weather.gov/documentation/services-web-api) (port 443) - once per hour

Rarely, during Ethernet access, the `loop()` code has been observed to hang and recovers only when the watchdog timer times out. Efforts (by others) to fix this known problem with Arduino Ethernet have focused on dealing with "stuck sockets". For further discussion, see:
- https://forum.arduino.cc/t/dealing-with-lost-arduino-ethernet-connectivity-stuck-sockets/244055/12
- https://github.com/arduino-libraries/Ethernet/issues/82

In our case, stuck sockets appear to be caused by unexpected (and unexplained) connections made from _within_ the LAN to the webserver (see `webserver.ino`). In the current firmware, we simply force close stuck sockets as soon as they are detected. As noted by others, a better solution would be for the Arduino Ethernet library routines to detect and handle this issue.

## Program description
The code is built around a low-level timer interrupt which calls a function called `readADCs()` in `adc.ino` at a rate of 10000 Hz. `readADCs()` is itself time-sliced to perform various tasks at a rate of 1000 Hz. The rationale for this scheme is to accomplish the following:

1. Independent of `loop()` code, `readADCs()` calls "critical" functions at rates up to 10000 Hz which:
   - sample various raw voltages and currents from a 3-phase alternator (wind turbine)
   - sample various temperature sensors (anemometer, power board rectifier, logic board electronics)
   - control a stepper motor (wind turbine tail motor)
   - control output to a high-voltage load resistor (when needed to divert turbine power)
   - compute "instantaneous" alternator rotational frequency (rpm)
   - compute alternator energy production (Energy = Power x time)

2. Within `loop()`, precise _once-per-second_ execution of:
   - data reads - via Modbus/RTU (charge controllers), Modbus/TCP (battery stack), or direct Serial connection (anemometer)
   - data writes - of controller data and state to the SD card of the Ethernet Shield 2
   - data posts - of controller data and state via UDP packets directed to a (remote or local) "Data Server"

3. Within `loop()`, but at rates much _slower_ than once-per-second, the firmware:
   - sends and compares system configuration data saved on a (remote or local) "Update Server"
   - checks for firmware updates on the Update Server
   - monitors a National Weather Service API for alerts, warnings, or advisories (e.g., high winds)

Within `loop()`, various timeouts are used to limit how much time data reads, writes and posts are allowed to take before moving on to the next `loop()` iteration. The idea is to complete execution of all `loop()` tasks (successful or not) in less than one second and thereby minimize "lost" data.

Outside of `loop()`, the interrupt-driven, high-rate "critical" processes _always_ function at their intended rates whether or not `loop()` is slowed (or blocked) for any reason. This can be an important safety feature for managing a machine such as a wind turbine!

Interestingly, the standard watchdog function will reboot the controller should `loop()` freeze up for any reason _regardless_ of the state of interrupt-driven tasks. However, a frozen *interrupt-driven* task **will not be detected** by the watchdog unless such a failure prevents `loop()` itself from executing. While this has **not** been an issue, an open question is how such events might be detected and dealt with.

