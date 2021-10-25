# OBD device based on CAN Bus :red_car: :dash:


[![Generic badge](https://img.shields.io/badge/v1.0.0-PASS-<COLOR>.svg)](https://shields.io/) 


**OBD device** is a C programmed project for communicating with ECUs from a car. This has been implemented in Tiva launchpad C Series, specifically in **TM4C123GH6PM** model.

## Installation

To install it only need to download compressed files from the repository and import the project directly in **Code Composer Studio**.

## Usage 

First, you can choose between three vehicle modules or ECU in a principal menu. Each ECU supports some operation modes of OBD and these are shown in a second menu where you can select one of them.

## Support 

For any question, issue or help, contact with me on my personal email address lolosancheznatera@gmail.com.

## Roadmap

There is a bug that needs to be fixed. It appears when navigate between different modes of the second menu in a normal use. It could be related to a missing pointer or a task that fails to execute again due to some condition or event that did not occur.

Some improvements are pending:

* Implement the SD card code to save 4 txt files, one for each system (Body, Chassis, Network and Powertrain) listing the DTCs and their meaning.

* Optimize the code in order to include the SD driver library (and optimizes this) and link the code with the microcontroller.

* In terms of Hardware one of the design improvements is to move the DB9 connector to the other layer to cover the top layer also with the casing.

* Increasingly include standard and manufacturer-specific DTCs.

* Some modes work with a car but this device has been developed to communicate with an ECU simulator (**OZEN ELEKTRONIK - OE91C1610**, so it only supports a few modes of operation). The idea is to be fully functional with a car.

* Include more and more operation modes.

If you have any ideas to improve this project, welcome!

## Versioned

The versions have to be based on [SemVer](https://semver.org) in order to contribute in this project.

## Authors of used libraries 

Two libraries has been included on this project:

1. The "Tiva-ST7735-Library" by **Julian Fell** (you cand find it here: https://github.com/jtfell/Tiva-ST7735-Library). This is ported Adafruit library for the TIVA Launchpad.

2. The "SD-card-TivaC-library" by **Javier Martínez Arrieta** (you can find it on the following repository: https://github.com/Javierma/SD-card-TivaC-library).

## License

![Image](https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-nc.svg)

This work (software and hardware) is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by-nc/4.0/ or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

## Project Status

This is a Final Year Project that I have recently finished so it will be on hold for a while (I will take a break to embark on other projects). However, I am looking forward to working on it in the future! 
