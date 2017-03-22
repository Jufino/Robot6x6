using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace webcam
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        bool cameraGrab = true;
        socketWebcam socketWebcam = new socketWebcam();
        private void button1_Click(object sender, EventArgs e)
        {
            socketWebcam.Open("192.168.2.6", "1212");
            System.Threading.Thread grabThread = new System.Threading.Thread(new System.Threading.ThreadStart(cameraGrabFunction));
            grabThread.Start();

        }
        
        private void cameraGrabFunction()
        {
            while (cameraGrab)
            {
                socketWebcam.SendString("imgK\n");
                Bitmap img = socketWebcam.recv_picture();
                if (img != null)
                {
                    pictureBox1.Image = new Bitmap(img);
                }
                socketWebcam.SendString("depK\n");
                img = socketWebcam.recv_picture();
                if (img != null)
                {
                    pictureBox2.Image = new Bitmap(img);
                }
                System.Threading.Thread.Sleep(33);
            }
            socketWebcam.Close();
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            cameraGrab = false;   
        }
    }
}
