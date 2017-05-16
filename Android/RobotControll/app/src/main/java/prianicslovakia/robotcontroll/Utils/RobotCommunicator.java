package prianicslovakia.robotcontroll.Utils;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Matrix;

import java.io.BufferedWriter;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.charset.StandardCharsets;
import java.text.DecimalFormat;

public class RobotCommunicator {
    private Socket socket;
    private int SERVERPORT = 1212;
    private String SERVERIP = "192.168.100.1";

    public RobotCommunicator(String RobotIP, int RobotPort) {
        this.SERVERIP = RobotIP;
        this.SERVERPORT = RobotPort;
    }
    public void connect() {
        Thread connectThread = new Thread(new Runnable() {
            @Override
            public void run() {
                InetAddress serverAddr = null;
                try {
                    serverAddr = InetAddress.getByName(SERVERIP);
                    socket = new Socket(serverAddr, SERVERPORT);
                } catch (UnknownHostException e) {
                    socket = null;
                    e.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
        connectThread.start();
    }

    public boolean isConnected() {
        if (socket != null) return socket.isConnected();
        else return false;
    }

    public void sendData(String data) throws IOException {
        new PrintWriter(new BufferedWriter(new OutputStreamWriter(socket.getOutputStream())), true).println(data);
    }

    public byte[] receiveBytes(int byteCount) throws IOException {
        byte buffer[] = new byte[byteCount];
        int count = 0;
        while ((count += socket.getInputStream().read(buffer, 0, byteCount)) < byteCount) ;
        return buffer;
    }

    public String receiveString(int byteCount) throws IOException {
        return new String(receiveBytes(byteCount), StandardCharsets.UTF_8);
    }

    public Boolean disconnect() throws IOException {
        if(socket.isConnected()) socket.close();
        return socket.isClosed();
    }
}
