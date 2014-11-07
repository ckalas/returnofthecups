#returnofthecups

<img align="right" hspace=15 src="cup.jpg">

<p align="justify">This repository contains code to ultimately detect, locate and track coffee cups in such a manner that a RRR manipulator can pick up the cups in turn and move them to various dispensing locations. It contains standalone code for the vision and the manipulator as well as the main program which combines them.</p>


###INSTALLATION


1. opencv
2. libfreenect

Preferably use a package manager (use homebrew on OSX) - this ensures that dependencies
are correctly installed.

---

##Operation
The main program creates an instance of the vision program, which directs the location of cups and other markers to another process that handles the dynamixel movement. Both programs block at certain stages waiting for input from the other.

To locate the cups a Haar-cascade classifier is used, whereas to find the various dropoff locations standard feature matching techniques are used.

The dynamixels used are 3 AX-12A models and one __MX something__. 

###FILES

#####Vision

* device.cpp : the abstracted freenect device class
* cups.cpp : functions to help identify and located cups
* fiducial.cpp : functions to detect and locate fiducial markers
* main.cpp : the main program


####Robot

* control_and_input.cpp
* fkine.cpp
* ikine.cpp
* interpolate.cpp
* multi_motor.cpp
* dxl_hal.c
* dynamixel.c
* main.cpp

---

###Notes

Currently the dynamixel code runs only on __Linux__. The vision code compiles and runs on both __Linux__ and __OSX__.
