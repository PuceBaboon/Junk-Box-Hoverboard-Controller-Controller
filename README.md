# Junk-Box-Hoverboard-Controller-Controller
Using a pair of ESP32s to create a remote control for a Hoverboard controller (ESP-Now)

## What is it?
There are a couple of outstanding projects available on GitHub using the motors and controllers from the cheap, Chinese made "HoverBoards" (also called balance boards or balance scooters in some parts of the world) as a convenient source of power and motive force for mobile projects:-

- [Niklas Fauth's Original Hoverboard Hack](https://github.com/NiklasFauth/hoverboard-firmware-hack)
- [Emanuel Feru's Field Oriented Control Version](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC)
- [Florian's Dual-Controller-Board Version](https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2)

*This* project provides a low cost remote control (currently for the FOC version, but should be easily adaptable to the others) using two ESP32 modules; one for the Hoverboard controller (the receiver) and one as a standalone, portable, simple remote control unit.  The two ESP32s communicate using Espressif's own, lightweight ESP-Now protocol (WiFi connectivity over the normally assigned 2.4GHz channels, but point-to-point, with no access-point involved).

The project features:-
- Power for the hoverboard end provided by a cheap, readily available switching regulator from the Hoverboard supply.
- Power for the remote-control unit provided by three, AA batteries.
- Switch configuration for the remote control is currently two clusters of five switches in the classic number-five-domino-face layout.
- Heartbeat between the remote control and hoverboard controller triggering an emergency stop when out of range.
- Auto heartbeat between the hoverboard controller motherboard and the attached ESP32 to ensure the motherboard firmware doesn't timeout.
- Auto-stop at power-on to ensure that the mobile unit doesn't get unintended commands during start-up.
- The ESP32 attached to the hoverboard motherboard communicates using its UART2 serial hardware, *not* SoftwareSerial.
- Incremental forward/back directional control and steering.

While the project is at a fairly early stage, it is already operational and will provide a crude (but working) remote control for a mobile project.

## How to build it
The project is presented here as a complete PlatformIO project directory, but can easily be compiled under the Arduino IDE.

There are two separate src files:-

- espnowHB.ino  Is the source file for the HB-controller-controller (attaches to the motherboard UART).
- espnowRC.ino  Is the Remote Control source.

*Only one of the files can be compiled at a time*, so to compile the RC version, just move espnowHB.ino up out of the source directory and vice-versa for the HB-controller-controller.
If someone knows the magic platformio incantation for doing this without having two thousand #ifdefs in the source, please drop me a note.

### Current status
The project code is a mess ...but it *does* work!

###### Why "Junk-Box-Hoverboard-Controller-Controller"
The hoverboard itself has a controller board (some of them have two) which takes feedback from the foot-switches, MPU-6050 gyro and hall-effect switches inside the motors and powers the motors based on the feedback (I occasionally use the term "motherboard", both here and in the source code, to make it clear that I'm talking about this hoverboard internal controller).

We use two ESP32s in this project.  One is used for the remote control unit.  It send data to a second unit via the ESP-Now protocol.

This second ESP32 is attached to the hoverboard internal controller via a serial cable.  Based on input from the remote control unit, it sends commands to the motherboard to control the speed and direction of the motors.  Thus it is the "hoverboard controller controller".

As for the "Junk-Box" part; you can see from the pictures that both of the ESP32 units were built from parts which were on hand.  Some parts are new "old stock" (resistors, capacitors, switches), while others were salvaged from older projects, or scavenged from other sources.  I did buy a couple of second-hand hoverboards (both advertised as "junk") for parts, but nothing was bought new for this specific project (so far, anyway).
