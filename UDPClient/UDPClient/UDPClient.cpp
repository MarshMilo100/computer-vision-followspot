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




int main(int argc, char* argv[]) // We can pass in a command line option!! 
{

	UDPClient client;

	client.Initialize();
	client.Connect();
	for (int i = 0; i < 20; i++)
	{
		client.SendUDP("Yeet times " + to_string(i));
	}

	
	return 0;
}





