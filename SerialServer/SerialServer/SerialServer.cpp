// SerialServer.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <vector>


using namespace std;

int main()
{
    cout << "Hello World!\n";
	char* levelSensorPort = "/dev/ttyUSB0"; //Serial Device Address

	int levelSensor = serialOpen(levelSensorPort, 19200);
	wiringPiSetup();
	serialPuts(levelSensor, "DP"); //Send command to the serial device

	while (1)
	{
		char buffer[100];
		ssize_t length = read(levelSensor, &buffer, sizeof(buffer));
		if (length == -1)
		{
			cerr << "Error reading from serial port" << endl;
			break;
		}
		else if (length == 0)
		{
			cerr << "No more data" << endl;
			break;
		}
		else
		{
			buffer[length] = '\0';
			cout << buffer; //Read serial data
		}
	}

	return 0;
}
