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
            socketWebcam.Open("192.168.10.1", "1212");
            System.Threading.Thread grabThread = new System.Threading.Thread(new System.Threading.ThreadStart(cameraGrabFunction));
            grabThread.Start();

        }
        
        private void cameraGrabFunction()
        {
            while (cameraGrab)
            {
                socketWebcam.SendString("depK\n");
                Bitmap img = socketWebcam.recv_picture();
                if (img != null)
                {
                    pictureBox2.Image = new Bitmap(img, pictureBox2.Size);
                }
                socketWebcam.SendString("map\n");
                img = socketWebcam.recv_picture();
                if (img != null)
                {
                    pictureBox3.Image = new Bitmap(img,pictureBox3.Size);
                }
                socketWebcam.SendString("rgbO\n");
                img = socketWebcam.recv_picture();
                if (img != null)
                {
                    pictureBox4.Image = new Bitmap(img, pictureBox4.Size);
                }
                socketWebcam.SendString("greenO\n");
                img = socketWebcam.recv_picture();
                if (img != null)
                {
                    pictureBox6.Image = new Bitmap(img, pictureBox6.Size);
                }
                socketWebcam.SendString("orangeO\n");
                img = socketWebcam.recv_picture();
                if (img != null)
                {
                    pictureBox7.Image = new Bitmap(img, pictureBox7.Size);
                }
                socketWebcam.SendString("depthO\n");
                img = socketWebcam.recv_picture();
                if (img != null)
                {
                    pictureBox8.Image = new Bitmap(img, pictureBox8.Size);
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
