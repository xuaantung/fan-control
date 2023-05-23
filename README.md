# Real-Time Fan Control with Feedback


Similar to cruise control system on car, I use Light sensor as a real time feeback to control the speed of the fan. 

The feedback used the PID algorithm to determine the next output.

## Prerequisites

- AVR-GCC for linking
- AVR-GDB for debugging
- AVR-Libc for support libraries 
- AVRDUDE for uploading code to the on-chip memeories of Michrochip's AVR controller 

## Usage 
To compile:
    make all

To run:
    make main-install 



