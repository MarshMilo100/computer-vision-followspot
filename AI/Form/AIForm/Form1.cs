using AIForm.network;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace AIForm
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
            button1.Visible = false;
            button1.Enabled = false;

            TcpClientWrapper client = new TcpClientWrapper("192.168.1.129", 9807);
            client.Connect();
            client.Send("Hello");
            string response = client.Recv(150);
            statusLabel.Text = response;
        }

    }
}
