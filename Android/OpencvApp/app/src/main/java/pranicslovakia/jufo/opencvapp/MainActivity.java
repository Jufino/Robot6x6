package pranicslovakia.jufo.opencvapp;

import java.io.IOException;
import java.util.Arrays;
import java.util.logging.Handler;
import java.util.logging.LogRecord;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.imgproc.Imgproc;

import android.app.Activity;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.os.Message;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.WindowManager;
import android.widget.ImageView;
import android.widget.TextView;

import Utils.RobotBitmap;
import Utils.RobotCommunicator;

import static org.opencv.android.Utils.matToBitmap;

public class MainActivity extends Activity  {
    RobotCommunicator robotCommunicator;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        robotCommunicator = new RobotCommunicator("192.168.0.102",1212);
        robotBitmap = new RobotBitmap(robotCommunicator,"img");
        robotBitmap1 = new RobotBitmap(robotCommunicator,"img1");
    }

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            if (status == LoaderCallbackInterface.SUCCESS ) {
                try {
                    robotCommunicator.connect();
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            helloworld();
                        }
                    }).start();

                } catch (IOException e) {
                    e.printStackTrace();
                }
            } else {
                super.onManagerConnected(status);
            }
        }
    };

    @Override
    public void onResume() {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0,this, mLoaderCallback);
    }
    RobotBitmap robotBitmap;
    RobotBitmap robotBitmap1;
    public void helloworld() {
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        while(robotCommunicator.isConnected()) {
            robotBitmap.receiveBitmap();
            handleBitmap.sendEmptyMessage(0);
            robotBitmap1.receiveBitmap();
            handleBitmap1.sendEmptyMessage(0);
        }
    }
    private android.os.Handler handleBitmap = new android.os.Handler() {
        @Override
        public void handleMessage(Message msg) {
            ImageView iv = (ImageView) findViewById(R.id.imageView);
            iv.setImageBitmap(robotBitmap.getBitmap());
            TextView textView = (TextView) findViewById(R.id.textView);
            textView.setText(robotBitmap.getFps().toString());
        }
    };
    private android.os.Handler handleBitmap1 = new android.os.Handler() {
        @Override
        public void handleMessage(Message msg) {
            ImageView iv1 = (ImageView) findViewById(R.id.imageView2);
            iv1.setImageBitmap(robotBitmap1.getBitmap());
        }
    };
}