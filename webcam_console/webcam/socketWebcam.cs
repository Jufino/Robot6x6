using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Drawing;
using System.Threading;
using System.IO;

    public class socketWebcam
    {
//-------------------------------------------------------------
        protected Socket socket_client;
        public Socket socketCon
        {
            get { return this.socket_client; }
        }
//-------------------------------------------------------------
        public bool Open(String IP, String Port)
        {
            try
            {
                socket_client = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                int alPort = System.Convert.ToInt16(Port, 10);
                System.Net.IPAddress remoteIPAddress = System.Net.IPAddress.Parse(IP);
                System.Net.IPEndPoint remoteEndPoint = new System.Net.IPEndPoint(remoteIPAddress, alPort);
                socket_client.ReceiveBufferSize = 100000;
                socket_client.Connect(remoteEndPoint);
                return true;

            }
            catch
            {
                return false;
            }

        }
        public bool Close()
        {
            try
            {
                socket_client.Close();
                return true;
            }
            catch
            {
                return false;
            }
        }
        public string ReceiveASCII(long length)
        {
            byte[] buffer = ReceiveBytes(length);
            return System.Text.Encoding.ASCII.GetString(buffer, 0, buffer.Length);
        }
        public byte[] ReceiveBytes(long length)
        {
            byte[] buffer = new byte[length];
            //while (socket_client.Available == 0) ;
            try
            {
                long size = 0;
                while (length > size)
                    size += socket_client.Receive(buffer, (int)size, (int)(length - size), SocketFlags.None);
            }
            catch { }
            return buffer;
        }
        public void SendString(string Data)
        {
            byte[] byData = System.Text.Encoding.ASCII.GetBytes(Data+"\0");
            SendByte(byData);
        }
        public void SendByte(byte[] Data)
        {
           /* Thread send = new Thread(delegate()
            {
                lock (zamok)
                {
            */
            try
            {
                socket_client.Send(Data, SocketFlags.None);
            }
            catch{}
             /*   }
            });
            send.Start();
            send.Join();
              */
        }
//-------------------------------------------------------------

    public enum ImageFormat
    {
        bmp,
        jpeg,
        gif,
        tiff,
        png,
        unknown
    }

    public static ImageFormat GetImageFormat(byte[] bytes)
    {
        // see http://www.mikekunz.com/image_file_header.html  
        var bmp = Encoding.ASCII.GetBytes("BM");     // BMP
        var gif = Encoding.ASCII.GetBytes("GIF");    // GIF
        var png = new byte[] { 137, 80, 78, 71 };    // PNG
        var tiff = new byte[] { 73, 73, 42 };         // TIFF
        var tiff2 = new byte[] { 77, 77, 42 };         // TIFF
        var jpeg = new byte[] { 255, 216, 255, 224 }; // jpeg
        var jpeg2 = new byte[] { 255, 216, 255, 225 }; // jpeg canon

        if (bmp.SequenceEqual(bytes.Take(bmp.Length)))
            return ImageFormat.bmp;

        if (gif.SequenceEqual(bytes.Take(gif.Length)))
            return ImageFormat.gif;

        if (png.SequenceEqual(bytes.Take(png.Length)))
            return ImageFormat.png;

        if (tiff.SequenceEqual(bytes.Take(tiff.Length)))
            return ImageFormat.tiff;

        if (tiff2.SequenceEqual(bytes.Take(tiff2.Length)))
            return ImageFormat.tiff;

        if (jpeg.SequenceEqual(bytes.Take(jpeg.Length)))
            return ImageFormat.jpeg;

        if (jpeg2.SequenceEqual(bytes.Take(jpeg2.Length)))
            return ImageFormat.jpeg;

        return ImageFormat.unknown;
    }

    public Bitmap recv_picture()
        {
            long sizeFromSystem = -1;
            long obrSize = -1;
            try
            {
                sizeFromSystem = Convert.ToInt32(ReceiveASCII(20));
                if (sizeFromSystem > 0)
                {
                    byte[] obr = ReceiveBytes(sizeFromSystem);
                    obrSize = obr.LongLength;
                    ImageFormat formatObr = GetImageFormat(obr);
                    if (formatObr != ImageFormat.unknown)
                    {
                        /*ImageConverter ic = new ImageConverter();
                        Image img = (Image)ic.ConvertFrom(obr);
                        return new Bitmap(img, _width, _height);*/
                        MemoryStream ms = new MemoryStream(obr);
                        return (new Bitmap(ms));
                    }
                    else
                    {
                        Console.WriteLine("Neznámy formát obrázka.\n");
                    }
                }
            }
            catch {
                Console.WriteLine("Problem s konverziou obrazka.\nVelkost so systemu:"+sizeFromSystem+"\nVelkost obrazka:"+obrSize+"\n");
            }
            return null;
        }
}