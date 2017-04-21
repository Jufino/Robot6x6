using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Text.RegularExpressions;

namespace webcam
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        bool sensorGrab = true;
        SocketWebcam socketWebcam = new SocketWebcam();
        private void button1_Click(object sender, EventArgs e)
        {
            socketWebcam.Open("192.168.10.1", "1213");
            System.Threading.Thread grabThread = new System.Threading.Thread(new System.Threading.ThreadStart(sensorGrabFunction));
            grabThread.Start();

        }

        private String speed = "0";
        private String dir = "S";
        private void sensorGrabFunction()
        {
            while (sensorGrab)
            {
                String buttons = socketWebcam.recv_data("butt");
                String roll = socketWebcam.recv_data("roll");
                String pitch = socketWebcam.recv_data("pitch");
                String yaw = socketWebcam.recv_data("yaw");
                String x = socketWebcam.recv_data("x");
                String y = socketWebcam.recv_data("y");
                String z = socketWebcam.recv_data("z");
                String volts = socketWebcam.recv_data("voltage");
                String capacityPercent = socketWebcam.recv_data("voltPer");
                String leds = socketWebcam.recv_data("leds");
                String ledK = socketWebcam.recv_data("ledK");

                if (sensorGrab)
                {
                    try
                    {
                        this.Invoke(new Action<string>(setLedKinect), new object[] { ledK });
                        this.Invoke(new Action<string>(setLeds), new object[] { leds });
                        this.Invoke(new Action<string>(setButtons), new object[] { buttons });
                        this.Invoke(new Action<string, string, string>(setAxis), new object[] { x, y, z });
                        this.Invoke(new Action<string, string, string>(setAngles), new object[] { roll, pitch, yaw });
                        this.Invoke(new Action<string, string>(setVoltage), new object[] { volts, capacityPercent });
                    }
                    catch { };
                    System.Threading.Thread.Sleep(33);
                }
            }
            socketWebcam.Close();
        }

        private void setButtons(String hodnota)
        {
            int intHodnota = Convert.ToInt16(hodnota);
            buttonUp.Checked = ((intHodnota & 1) == 1);
            buttonMiddle.Checked = ((intHodnota & 2) == 2);
            buttonDown.Checked = ((intHodnota & 4) == 4);
        }

        private void setAxis(String x, String y, String z)
        {
            xPos.Text = x;
            yPos.Text = y;
            zPos.Text = z;
        }

        private void setVoltage(String volts, String capacityPercent)
        {
            this.volts.Text = volts;
            this.capacityPercent.Text = capacityPercent;
        }

        private void setLeds(String leds)
        {
            int intLeds = Convert.ToInt16(leds);
            redUp.Checked = ((intLeds & (1 << 1)) == (1 << 1));
            greenUp.Checked = ((intLeds & (1 << 2)) == (1 << 2));
            redMiddle.Checked = ((intLeds & (1 << 3)) == (1 << 3));
            greenMiddle.Checked = ((intLeds & (1 << 4)) == (1 << 4));
            redDown.Checked = ((intLeds & (1 << 5)) == (1 << 5));
            greenDown.Checked = ((intLeds & (1 << 6)) == (1 << 6));
        }

        private void setLedKinect(String ledKinect)
        {
            int intLedKinect = Convert.ToInt16(ledKinect);
            redKinect.Checked = ((intLedKinect & (1 << 1)) == (1 << 1));
            greenKinect.Checked = ((intLedKinect & (1 << 2)) == (1 << 2));
            orangeKinect.Checked = ((intLedKinect & (1 << 3)) == (1 << 3));
            blinkRedOrangeKinect.Checked = ((intLedKinect & (1 << 4)) == (1 << 4));
            blinkGreenKinect.Checked = ((intLedKinect & (1 << 5)) == (1 << 5));
            blinkOrangeKinect.Checked = ((intLedKinect & (1 << 6)) == (1 << 6));
        }

        private void setAngles(String roll, String pitch, String yaw)
        {
            roll = Regex.Replace(roll, "((?![0-9.-]).)+", "").Replace(".", ",");
            pitch = Regex.Replace(pitch, "((?![0-9.-]).)+", "").Replace(".", ",");
            yaw = Regex.Replace(yaw, "((?![0-9.-]).)+", "").Replace(".", ",");
            double rollDegree = Convert.ToDouble(roll) * (180 / Math.PI);
            double pitchDegree = Convert.ToDouble(pitch) * (180 / Math.PI);
            double yawDegree = Convert.ToDouble(yaw) * (180 / Math.PI);

            rollPos.Text = roll;
            pitchPos.Text = pitch;
            yawPos.Text = yaw;
            rollPosDegree.Text = rollDegree.ToString();
            pitchPosDegree.Text = pitchDegree.ToString();
            yawPosDegree.Text = yawDegree.ToString();
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            sensorGrab = false;
        }

        private void motorUp_Click(object sender, EventArgs e)
        {
            dir = "F";
            socketWebcam.SendString("dir;" + dir + "\n");

        }

        private void motorStop_Click(object sender, EventArgs e)
        {
            dir = "S";
            socketWebcam.SendString("dir;" + dir + "\n");
        }

        private void motorClockwise_Click(object sender, EventArgs e)
        {
            dir = "C";
            socketWebcam.SendString("dir;" + dir + "\n");
        }

        private void motorDown_Click(object sender, EventArgs e)
        {
            dir = "B";
            socketWebcam.SendString("dir;" + dir + "\n");
        }

        private void motorAnticlockwise_Click(object sender, EventArgs e)
        {
            dir = "A";
            socketWebcam.SendString("dir;" + dir + "\n");
        }

        private void speedbox_ValueChanged(object sender, EventArgs e)
        {
            speed = speedbox.Value.ToString();
            socketWebcam.SendString("speed;" + speed + "\n");
        }
    }
}
