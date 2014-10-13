//============================================================================
// Name        : CPP_AXControl.cpp
// Author      : JCA
// Version     :
// Copyright   : GPL
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
using namespace std;

#include "SerialPort.h"
#include "Dynamixel.h"

int main() {
	cout << "AX Control starts" << endl; // prints AX Control

	int error=0;
	int idAX12=18;

	SerialPort serialPort;
	Dynamixel dynamixel;


	if (serialPort.connect("//dev//ttyS0")!=0) {
		dynamixel.sendTossModeCommand(&serialPort);

		int pos=dynamixel.getPosition(&serialPort, idAX12);

		if (pos>250 && pos <1023)
			dynamixel.setPosition(&serialPort, idAX12, pos-100);
		else
			printf ("\nPosition <%i> under 250 or over 1023\n", pos);

		serialPort.disconnect();
	}
	else {
		printf ("\nCan't open serial port");
		error=-1;
	}

	cout << endl << "AX Control ends" << endl; // prints AX Control
	return error;
}
