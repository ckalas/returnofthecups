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
* Navigate to __/vision__
* Type __make__
* If successful, type __sudo ./main__

Unfortunately the dynamixel code only runs on Linux. If testing the vision on its own (Linux OR OSX), change __#define ROBOT__ to __0__ in /vision/main.cpp and re-make.


The main program creates an instance of the vision program, which directs the location of cups and other markers to another process that handles the dynamixel movement. Both programs block at certain stages waiting for input from the other.

To locate the cups a Haar-cascade classifier is used, whereas to find the various dropoff locations standard feature matching techniques are used.

The dynamixels used are 3 AX-12A models and one MX model. 

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

Currently the dynamixel code runs only on __Linux__. The vision code compiles and runs on both __Linux__ and __OSX__. Other things to keep in mind are the frame transformations from the __camera__ to the first __fiducial__ and to the __robot base__.
