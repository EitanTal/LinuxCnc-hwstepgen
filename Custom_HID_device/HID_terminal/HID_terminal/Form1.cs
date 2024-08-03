using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using HidSharp;


namespace HID_terminal
{
    public partial class Form1 : Form
    {
         HidDeviceLoader loader;
         HidDevice device;
         HidStream stream;
         byte []bytes;
        public Form1()
        {
            InitializeComponent();
            loader = new HidDeviceLoader();
            device = loader.GetDevices(1155, 22352).FirstOrDefault();
            if (device == null)
            {
                Console.WriteLine("Failed to open device.");
                MessageBox.Show("Failed to open device.", "Error",
                MessageBoxButtons.OK, MessageBoxIcon.Error);
                Environment.Exit(1);
            }
            
        }
 

        private void button_send_Click(object sender, EventArgs e)
        {
            if (!device.TryOpen(out stream))
            {
                Console.WriteLine("Failed to open device."); MessageBox.Show("Failed to open device.", "Error",
MessageBoxButtons.OK, MessageBoxIcon.Error); Environment.Exit(1);
            }
            
            using (stream)
            {
                int n = 0;

                    bytes = new byte[device.MaxInputReportLength];
                    int count=0;
                    var message = new byte[64];
                    ASCIIEncoding.ASCII.GetBytes(textBoxSend.Text, 0, textBoxSend.Text.Length, message, 2);
                    message[0] = 0;
                    message[1] = (byte)textBoxSend.Text.Length;
                    stream.Write(message, 0, 2+textBoxSend.Text.Length);
                    try
                    {
                        count = stream.Read(bytes);
                    }
                    catch (TimeoutException)
                    {
                        Console.WriteLine("Read timed out.");
                    }

                    if (count > 0)
                    {
                        this.BeginInvoke(new EventHandler(DoUpdate));
                    }
           
        }
        }
        private void DoUpdate(object sender, System.EventArgs e)
        {
            string s = Encoding.UTF8.GetString(bytes, 2, 2+bytes[1]);
            textBoxRecieved.AppendText(s);
            textBoxRecieved.AppendText("\n");

        }
    }
}
