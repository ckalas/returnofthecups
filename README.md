#returnofthecups

###INSTALLATION

1. Install OpenCV
2. Install libfreeenct

Preferably use a package manager (use homebrew on OSX) - this ensures that dependencies
are correctly installed.

3. Install FTDI drivers http://www.ftdichip.com/Drivers/VCP.htm

###FILES

#####Vision
* device.cpp / device.h : the abstracted freenect device class
* cups.cpp / cups.h : functions to help identify and located cups
* main.cpp : the main progra