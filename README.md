# ubiquity_motor
[![Build Status](https://travis-ci.org/UbiquityRobotics/ubiquity_motor.svg?branch=indigo-devel)](https://travis-ci.org/UbiquityRobotics/ubiquity_motor)

## Introduction

This is a Package that provides a ROS interface for the motors in UbiquityRobotics robots

## Installation - From Binary

This package may be installed from binaries for both x86 and ARM architectures.

The package may be installed with:

`sudo apt-get install ros-kinetic-ubiquity-motor`

Configuration launch files for the _Magni_ robot are in the package [magni_robot](https://github.com/UbiquityRobotics/magni_robot).

Running your ubiquity motor controller over USB using an FTDI USB-serial device with ubiquity_motor

Ubiquity motors motor controller has a connector that is designed to operate with FTDI USB-serial devices thus making it possible to add a USB to serial device on the robot and make it run over USB. In order to run ubiquity_motor with an FTDI it is necessary to specify the FTDI device e.g. `/dev/ttyUSB0` in magni_bringup/param/base.yaml.

Running your ubiquity motor controller using a PCI to serial connector (e.g. for NUC or other desktop computers)
