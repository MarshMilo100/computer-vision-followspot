using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;

namespace AIForm.network
{
    class TcpClientWrapper
    {
        private int port;
        private string hostname;
        private TcpClient socket;
        private bool connected = false;
        public TcpClientWrapper(string hostname, int port)
        {
            this.hostname = hostname;
            this.port = port;
            this.socket = new TcpClient();
        }

        public bool Connect()
        {
            try
            {
                this.socket.Connect(this.hostname, this.port);
            } catch (Exception e)
            {
                this.connected = false;
                throw e;
            }
            this.connected = true;
            return true;
        }

        public bool Send(string message)
        {
            if(this.connected)
            {
                NetworkStream stream = this.socket.GetStream();
                Byte[] data = System.Text.Encoding.ASCII.GetBytes(message);
                try
                {
                    stream.Write(data, 0, data.Length);
                } catch(Exception e) 
                {
                    throw e;
                    return false;
                }
                return true;

            }
            return false;
        }

        public string Recv(int size)
        {
            if (this.connected)
            {
                NetworkStream stream = this.socket.GetStream();
                Byte[] data = new Byte[size];
                try
                {
                    int readSize = stream.Read(data, 0, data.Length);
                    string responseData = System.Text.Encoding.ASCII.GetString(data, 0, readSize);
                    return responseData;
                } catch (Exception e)
                {
                    throw e;
                    return null;
                }
            }
            return null;
        }
    }
}
