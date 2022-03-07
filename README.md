# DDS-bus-AD9852-AD9858-strontium-2

multi-channel DDS rack project:
For the super radian laser experiment we have developed radio frequency generation and control electronics with DDS, the AD9852 and AD9858.
These generators drive acousto-optics in a frequency range of 80MHz to 400MHz.
The electronic control is realized on a common SPI bus and controllable with a USB serial port and an Ethernet port.
We have also developed a version controllable with a Raspberry pi 4.
Attached to the project, the schematics of the electronic boards and the python script for the Raspberry pi4 and the serial link.
A main.c code is also visible and developed under composer studio 10 code to control the ARM of Texas instrument, the TIVA TM4C1294.

