#returnofthecups

###INSTALLATION

1. Install OpenCV
2. Install libfreeenct

Preferably use a package manager (use homebrew on OSX) - this ensures that dependencies
are correctly installed.

###FILES

#####Vision
* device.cpp / device.h : the abstracted freenect device class
* cups.cpp / cups.h : functions to help identify and located cups
* main.cpp : the main program

####Robot
<This required a rebuild in the Multi_Dynamixel_Robot_Control folder - need to make a better Makefile without the QT rubbish>