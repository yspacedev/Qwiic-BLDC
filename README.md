# Qwiic-BLDC
A driver for Allegro's A89301 and A89306 (probably) FOC BLDC controller ICs

This is some basic low-level code for the IC. I might make a GUI later if I decide to put in that effort. 

The code is written in Python for use with an MCP2221A USB to I2C bridge using Adafruit's libraries.

The code is not plug-and-play. You will need to use the datasheet to figure out which bits to set for your motor. It's a prototype, so there's not really any documentation.
