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
        SocketWebcam socketWebcam = new SocketWebcam();
        private void button1_Click(object sender, EventArgs e)
        {
            socketWebcam.Open("192.168.10.1", "1212");
            System.Threading.Thread grabThread = new System.Threading.Thread(new System.Threading.ThreadStart(cameraGrabFunction));
            grabThread.Start();

        }

        int poc = 0;

        private void cameraGrabFunction()
        {
            int pocetZlyhani = 0;
            while (cameraGrab)
            {
                Bitmap img = socketWebcam.recv_picture("depK");
                if (img != null)
                    pictureBox2.Image = new Bitmap(img, pictureBox2.Size);
                else
                    pocetZlyhani++;
                img = socketWebcam.recv_picture("map");
                if (img != null)
                    pictureBox3.Image = new Bitmap(img,pictureBox3.Size);
                else
                    pocetZlyhani++;
                img = socketWebcam.recv_picture("rgbO");
                if (img != null)
                    pictureBox4.Image = new Bitmap(img, pictureBox4.Size);
                else
                    pocetZlyhani++;
                img = socketWebcam.recv_picture("greenO");
                if (img != null)
                    pictureBox6.Image = new Bitmap(img, pictureBox6.Size);
                else
                    pocetZlyhani++;
                img = socketWebcam.recv_picture("orangeO");
                if (img != null)
                    pictureBox7.Image = new Bitmap(img, pictureBox7.Size);
                else
                    pocetZlyhani++;
                img = socketWebcam.recv_picture("depVK");
                if (img != null)
                    pictureBox1.Image = new Bitmap(img, pictureBox1.Size);
                else
                    pocetZlyhani++;
                System.Threading.Thread.Sleep(10);
                if (pocetZlyhani > 40)
                    cameraGrab = false;
                else if (pocetZlyhani <= 2) pocetZlyhani = 0;

                switch(poc){
                    case 0:
                    socketWebcam.SendString("iLowHG;" + minGreenH.Value + "\n");
                        poc++;
                        break;
                    case 1:
                        socketWebcam.SendString("iLowSG;" + minGreenS.Value + "\n");
                        poc++;
                        break;
                    case 2:
                        socketWebcam.SendString("iLowVG;" + minGreenV.Value + "\n");
                        poc++;
                        break;
                    case 3:
                        socketWebcam.SendString("iLowHO;" + minOrangeH.Value + "\n");
                        poc++;
                        break;
                    case 4:
                        socketWebcam.SendString("iLowSO;" + minOrangeS.Value + "\n");
                        poc++;
                        break;
                    case 5:
                        socketWebcam.SendString("iLowVO;" + minOrangeV.Value + "\n");
                        poc++;
                        break;
                    case 6:
                        socketWebcam.SendString("iHighHG;" + maxGreenH.Value + "\n");
                        poc++;
                        break;
                    case 7:
                        socketWebcam.SendString("iHighSG;" + maxGreenS.Value + "\n");
                        poc++;
                        break;
                    case 8:
                        socketWebcam.SendString("iHighVG;" + maxGreenV.Value + "\n");
                        poc++;
                        break;
                    case 9:
                        socketWebcam.SendString("iHighHO;" + maxOrangeH.Value + "\n");
                        poc++;
                        break;
                    case 10:
                        socketWebcam.SendString("iHighSO;" + maxOrangeS.Value + "\n");
                        poc++;
                        break;
                    case 11:
                        socketWebcam.SendString("erodeO;" + erodeO.Value + "\n");
                        poc++;
                        break;
                    case 12:
                        socketWebcam.SendString("erodeG;" + erodeG.Value + "\n");
                        poc++;
                        break;
                    case 13:
                        socketWebcam.SendString("dilateO;" + dilateO.Value + "\n");
                        poc++;
                        break;
                    case 14:
                        socketWebcam.SendString("dilateG;" + dilateG.Value + "\n");
                        poc++;
                        break;
                    case 15:
                        socketWebcam.SendString("iHighVO;" + maxOrangeV.Value + "\n");
                        poc = 0;
                        break;
                    default: poc = 0; break;
                }
            }
            socketWebcam.Close();
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            cameraGrab = false;   
        }
    }
}
