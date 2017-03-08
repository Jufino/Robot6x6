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
        gafuso.gafuso test = new gafuso.gafuso();
        private void button1_Click(object sender, EventArgs e)
        {
            test.Open("192.168.1.113", "1212");
            test.Add("img");
            System.Threading.Thread a = new System.Threading.Thread(new System.Threading.ThreadStart(testing));
            a.Start();

        }
        private void testing()
        {
            while (true)
            {
                test.Send();
                pictureBox1.Image = test.recv_picture();
                //System.Threading.Thread.Sleep(100);
            }
        }
    }
}
