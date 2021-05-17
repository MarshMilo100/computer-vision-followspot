/**********************************************************************
Name		: Example UDP Client
Author		: Sloan Kelly
Date		: 2017-12-16
Purpose		: Example of a bare bones UDP client

***********************************************************************/

#include <iostream>
#include <WS2tcpip.h>
#include <string>
#include <iostream>
#include <sstream>

// Include the Winsock library (lib) file
#pragma comment (lib, "ws2_32.lib")

// Saves us from typing std::cout << etc. etc. etc.
using namespace std;


// Code stems from YouTube Follow along here: https://youtu.be/uIanSvWou1M
// and bitbucket repository here: https://bitbucket.org/sloankelly/youtube-source-repository/src/bb84cf7f8d95d37354cf7dd0f0a57e48f393bd4b/cpp/networking/UDPClientServerBasic/?at=master

/// <summary>
/// UDP Client that connects to local host and sends UDP
/// </summary>
class UDPClient
{

private:
	SOCKET socketConn;
	sockaddr_in server;

public:
	UDPClient()
	{
		Initialize();
		Connect();
	}


	void Initialize()
	{
		WSADATA data;
		WORD version = MAKEWORD(2, 2);
		int wsOk = WSAStartup(version, &data);
		if (wsOk != 0)
		{
			cout << "Can't start Winsock! " << wsOk;
			return;
		}
	}

	void Connect()
	{

		this->server.sin_family = AF_INET; // AF_INET = IPv4 addresses
		this->server.sin_port = htons(54000); // Little to big endian conversion
		inet_pton(AF_INET, "127.0.0.1", &this->server.sin_addr); // Convert from string to byte array

		// Socket creation, note that the socket type is datagram
		this->socketConn = socket(AF_INET, SOCK_DGRAM, 0);
	}

	void SendUDP(string message)
	{
		// Write out to that socket
		string s(message);
		int sendOk = sendto(this->socketConn, s.c_str(), s.size() + 1, 0, (sockaddr*)&this->server, sizeof(this->server));

		if (sendOk == SOCKET_ERROR)
		{
			cout << "That didn't work! " << WSAGetLastError() << endl;
		}
	}

	void CleanUp()
	{
		// Close the socket
		closesocket(this->socketConn);

		// Close down Winsock
		WSACleanup();
	}
};


struct Coordinate
{
	int width, height;
};

struct BoundaryBox
{
	int widthIndex, heightIndex;
	int width, height;
};

Coordinate nice;


BoundaryBox getSinglePersonLocation(Coordinate* whitePixels)
{
	BoundaryBox personBoundaryBox = BoundaryBox();

	float widthAverage, widthStd = 0;
	float heightAverage, heightStd = 0;
	int length = sizeof(whitePixels) / sizeof(whitePixels[0]);

	int widthSum = 0, heightSum = 0;

	for (int whitePixelsIndex = 0; whitePixelsIndex < length; whitePixelsIndex++)
	{
		widthSum += whitePixels[whitePixelsIndex].width;
		heightSum += whitePixels[whitePixelsIndex].height;
	}


	widthAverage = widthSum / length;
	heightAverage = heightSum / length;

	for (int whitePixelsIndex = 0; whitePixelsIndex < length; whitePixelsIndex++)
	{
		widthStd += (whitePixels[whitePixelsIndex].width - widthAverage) * (whitePixels[whitePixelsIndex].width - widthAverage);
		heightStd += (whitePixels[whitePixelsIndex].height - heightAverage) * (whitePixels[whitePixelsIndex].height - heightAverage);
	}


	widthStd = widthStd * (1 / length - 1);
	heightStd = heightStd * (1 / length - 1);

	widthStd = pow(widthStd, 0.5);
	heightStd = pow(heightStd, 0.5);

	personBoundaryBox.widthIndex = widthAverage;
	personBoundaryBox.heightIndex = heightAverage;
	personBoundaryBox.width = widthStd * 1.5;
	personBoundaryBox.height = heightStd * 1.5;

	return personBoundaryBox;
}

Coordinate * getWhitePixels()
{
	Coordinate coordinatePixels[2000];

	for (int pixelCount = 0; pixelCount < 20; pixelCount++)
	{
		Coordinate temp = Coordinate();
		temp.height = 200;
		temp.width = 320;
		coordinatePixels[pixelCount] = temp;
	}

	return coordinatePixels;
}

int mainMethod()
{
	Coordinate* whitePixels = getWhitePixels();
	BoundaryBox centerLocation = getSinglePersonLocation(whitePixels);
	delete whitePixels;

	BoundaryBox previousLocation = centerLocation;

	while (true)
	{
		Coordinate* whitePixels = getWhitePixels();
		BoundaryBox currentLocation = getSinglePersonLocation(whitePixels);
		delete whitePixels;

		int upperLeftHeight = previousLocation.heightIndex - (previousLocation.height / 2);
		int uperLeftWidth = previousLocation.widthIndex - (previousLocation.width / 2);

		int lowerRightHeight = previousLocation.heightIndex + (previousLocation.height / 2);
		int lowerRightWidth = previousLocation.widthIndex + (previousLocation.width / 2);


		if (!(currentLocation.heightIndex < lowerRightHeight && currentLocation.height > upperLeftHeight &&
			currentLocation.widthIndex < lowerRightWidth && currentLocation.widthIndex > uperLeftWidth))
		{
			// This is when we create new UDP light packets

		}

		previousLocation = currentLocation;
	}
}


int main(int argc, char* argv[]) // We can pass in a command line option!! 
{

	//mainMethod();

	/*UDPClient client;

	client.Initialize();
	client.Connect();
	for (int i = 0; i < 20; i++)
	{
		client.SendUDP("Yeet times " + to_string(i));
	}*/

	BYTE byteArray[8] = { 0x31, 0x38, 0x37, 0x38, 0x37, 0x37, 0x37, 0x37 };
	BYTE byteArrayNumber[8] = { 49, 56, 57, 56, 57, 58, 58, 57 };

	int a = int((unsigned char)(byteArrayNumber[0]) << 24 |
		(unsigned char)(byteArrayNumber[1]) << 16 |
		(unsigned char)(byteArrayNumber[2]) << 8 |
		(unsigned char)(byteArrayNumber[3]));

	printf("%i\n", a);

	int x = 4567;
	char bytes[sizeof x];
	std::copy(static_cast<const char*>(static_cast<const void*>(&x)),
		static_cast<const char*>(static_cast<const void*>(&x)) + sizeof x,
		bytes);

	printf("\n", bytes);

	int c = int((0x22)<< 24 |
		(0x23) << 16 |
		(0x02) << 8 |
		(0x20));

	int b = int((bytes[0]) << 24 |
		(bytes[1]) << 16 |
		(bytes[2]) << 8 |
		(bytes[3]));

	printf("%i\n", b);
	printf("%i\n", c);

	std::string s(reinterpret_cast<char*>(byteArray), 4);
	int i_auto = std::stoi(s, nullptr, 0);
	std::cout << s << std::endl;

	uint8_t byteArray1[5] = { 0x48, 0x65, 0x6C, 0x6C, 0x6F };
	std::string str[(sizeof byteArray1) + 1];
	memcpy(str, byteArray1, sizeof byteArray1);
	//str[sizeof byteArray1] = 0; // Null termination.
	printf("%s\n", str);


	unsigned char morebytes[4]{ 0x31, 0x38, 0x37, 0x38 };

	int value;
	std::memcpy(&value, morebytes, sizeof(int));

	std::cout << value << '\n';

	// Should probably clear these in case data left over from a previous read
	BYTE input_buffer[8] = { 0x31, 0x38, 0x37, 0x38, 0x37, 0x37, 0x37, 0x37 };

	// Parse length
	//int length = *((int *)&input_buffer[4]);
	// Since I don't know what size an int is on your system, this way is better
	int length = input_buffer[0] << 16 | (input_buffer[1] << 8) | (input_buffer[3]);

	std::string text{ "123" };
	errno = 0; // pre set to 0
	int number = (int)std::strtol(text.c_str(), nullptr, 10);

	// input_buffer should now have a complete message

			//std::string s(reinterpret_cast<char*>(Rx_Data), 4);
		//int next_msg_len = std::stoi(s, nullptr, 0);

		//string s(reinterpret_cast<char*>(Rx_Data), 4);
		//int i_auto = std::stoi(s, nullptr, 0);

		//char str[(sizeof Rx_Data) + 1];
		//memcpy(str, Rx_Data, sizeof Rx_Data);
		//str[sizeof byteArray1] = 0; // Null termination.

		//int next_msg_length = 1
		//memcpy(&next_msg_length, Rx_Data, sizeof(int));

		//string stringy = "999";
		//stringstream degree(s);
		//int next_msg_len = 0;
		//stringy >> next_msg_len;




	return 0;
}





