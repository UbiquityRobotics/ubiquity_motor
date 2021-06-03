# Move to Boost ASIO Serial

* Status: proposed 
* Deciders: Rohan Agrawal, Teodor Podobnik
* Date: 2020-06-02

Technical Story: https://github.com/UbiquityRobotics/ubiquity_motor/issues/131

## Context and Problem Statement

We currently use the serial libary written by wjwwood (https://github.com/wjwwood/serial), but it appears to be abandoned with no noetic release.
The library seems to compile and work on Ubuntu 20.04 and ROS Noetic when built from source. 
How do we proceed with a noetic version of ubiquity_motor when the underlying serial libary is not yet released for noetic?

## Considered Options

* Keep waiting for a noetic version of serial
* Move to Boost ASIO
* Use the OS C API directly

## Decision Outcome

Chosen option: Move to Boost ASIO serial.

### Positive Consequences

* We will be able to proceed with a binary noetic release.

## Pros and Cons of the Options <!-- optional -->

### Keep waiting for a noetic version of serial

* Good, because it requires little engineering or re-testing from our side.
* Bad, because it seems like its not going to happen based on the lack of response here: https://github.com/wjwwood/serial/issues/222.

### Move to Boost ASIO

Boost ASIO provides a cross-platform C++ serial library.
https://www.boost.org/doc/libs/1_65_0/doc/html/boost_asio/overview/serial_ports.html 

* Good, because it keeps cross-platform support, leaving the possibility of Windows support open. 
* Good, because boost is heavily used enough that we will continue to be able to depend on it in the future.
* Bad, because the ASIO API is pretty complicated compared to what we are trying to do. 

### Use the OS C API directly

Instead of using a library to handle serial port communication, we could always directly call the OS API.

* Good, because we will depend on less external code.
* Bad, the C APIs can have unexpected pitfalls that we need to be careful to avoid. 
* Bad, because we only have the resources to support Linux, so Windows will not be supported.
