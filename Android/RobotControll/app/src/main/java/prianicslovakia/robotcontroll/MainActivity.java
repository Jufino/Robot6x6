package prianicslovakia.robotcontroll;

import android.app.usage.UsageEvents;
import android.os.Handler;
import android.os.Looper;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.math.BigDecimal;
import java.util.EventListener;

import prianicslovakia.robotcontroll.Utils.RobotCommunicator;

import static java.lang.Thread.sleep;

public class MainActivity extends AppCompatActivity {

    RobotCommunicator robotCommunicator = new RobotCommunicator("192.168.100.1", 1213);

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        robotCommunicator.connect();

        Button upButton = (Button) findViewById(R.id.upButton);
        Button downButton = (Button) findViewById(R.id.downButton);
        Button leftButton = (Button) findViewById(R.id.leftBotton);
        Button rightButton = (Button) findViewById(R.id.rightButton);
        final Switch isOnFollow = (Switch) findViewById(R.id.isOnFollow);
        SeekBar legBar = (SeekBar) findViewById(R.id.legBar);
        SeekBar distanceBar = (SeekBar) findViewById(R.id.distanceBar);
        final TextView legView = (TextView) findViewById(R.id.legVIew);
        final TextView distanceView = (TextView) findViewById(R.id.distanceView);
        final int speed_mm_s = 1000;


        upButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent motionEvent) {
                if (motionEvent.getAction() == MotionEvent.ACTION_DOWN) {
                    try {
                        if (!robotCommunicator.isConnected()) robotCommunicator.connect();
                        if (robotCommunicator.isConnected()) {
                            robotCommunicator.sendData("dir;F");
                            robotCommunicator.sendData("speed;" + String.valueOf(speed_mm_s));
                        } else {
                            Toast.makeText(getApplicationContext(), "Robot not connect. Please check WIFI connection.", Toast.LENGTH_SHORT).show();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                        try {
                            Toast.makeText(getApplicationContext(), "Robot reconnecting...", Toast.LENGTH_SHORT).show();
                            robotCommunicator.disconnect();
                            robotCommunicator.connect();
                        } catch (IOException e1) {
                            e1.printStackTrace();
                        }
                    }
                    return true;
                } else if (motionEvent.getAction() == MotionEvent.ACTION_UP) {
                    try {
                        if (!robotCommunicator.isConnected()) robotCommunicator.connect();
                        if (robotCommunicator.isConnected()) {
                            robotCommunicator.sendData("dir;S");
                        } else {
                            Toast.makeText(getApplicationContext(), "Robot not connect. Please check WIFI connection.", Toast.LENGTH_SHORT).show();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                        try {
                            Toast.makeText(getApplicationContext(), "Robot reconnecting...", Toast.LENGTH_SHORT).show();
                            robotCommunicator.disconnect();
                            robotCommunicator.connect();
                        } catch (IOException e1) {
                            e1.printStackTrace();
                        }
                    }
                    return true;
                }
                return false;
            }
        });

        downButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent motionEvent) {
                if (motionEvent.getAction() == MotionEvent.ACTION_DOWN) {
                    try {
                        if (!robotCommunicator.isConnected()) robotCommunicator.connect();
                        if (robotCommunicator.isConnected()) {
                            robotCommunicator.sendData("dir;B");
                            robotCommunicator.sendData("speed;" + String.valueOf(speed_mm_s));
                        } else {
                            Toast.makeText(getApplicationContext(), "Robot not connect. Please check WIFI connection.", Toast.LENGTH_SHORT).show();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                        try {
                            Toast.makeText(getApplicationContext(), "Robot reconnecting...", Toast.LENGTH_SHORT).show();
                            robotCommunicator.disconnect();
                            robotCommunicator.connect();
                        } catch (IOException e1) {
                            e1.printStackTrace();
                        }
                    }
                    return true;
                } else if (motionEvent.getAction() == MotionEvent.ACTION_UP) {
                    try {
                        if (!robotCommunicator.isConnected()) robotCommunicator.connect();
                        if (robotCommunicator.isConnected()) {
                            robotCommunicator.sendData("dir;S");
                        } else {
                            Toast.makeText(getApplicationContext(), "Robot not connect. Please check WIFI connection.", Toast.LENGTH_SHORT).show();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                        try {
                            Toast.makeText(getApplicationContext(), "Robot reconnecting...", Toast.LENGTH_SHORT).show();
                            robotCommunicator.disconnect();
                            robotCommunicator.connect();
                        } catch (IOException e1) {
                            e1.printStackTrace();
                        }
                    }
                    return true;
                }
                return false;
            }
        });

        leftButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent motionEvent) {
                if (motionEvent.getAction() == MotionEvent.ACTION_DOWN) {
                    try {
                        if (!robotCommunicator.isConnected()) robotCommunicator.connect();
                        if (robotCommunicator.isConnected()) {
                            robotCommunicator.sendData("dir;A");
                            robotCommunicator.sendData("speed;" + String.valueOf(speed_mm_s));
                        } else {
                            Toast.makeText(getApplicationContext(), "Robot not connect. Please check WIFI connection.", Toast.LENGTH_SHORT).show();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                        try {
                            Toast.makeText(getApplicationContext(), "Robot reconnecting...", Toast.LENGTH_SHORT).show();
                            robotCommunicator.disconnect();
                            robotCommunicator.connect();
                        } catch (IOException e1) {
                            e1.printStackTrace();
                        }
                    }
                    return true;
                } else if (motionEvent.getAction() == MotionEvent.ACTION_UP) {
                    try {
                        if (!robotCommunicator.isConnected()) robotCommunicator.connect();
                        if (robotCommunicator.isConnected()) {
                            robotCommunicator.sendData("dir;S");
                        } else {
                            Toast.makeText(getApplicationContext(), "Robot not connect. Please check WIFI connection.", Toast.LENGTH_SHORT).show();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                        try {
                            Toast.makeText(getApplicationContext(), "Robot reconnecting...", Toast.LENGTH_SHORT).show();
                            robotCommunicator.disconnect();
                            robotCommunicator.connect();
                        } catch (IOException e1) {
                            e1.printStackTrace();
                        }
                    }
                    return true;
                }
                return false;
            }
        });

        rightButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent motionEvent) {
                if (motionEvent.getAction() == MotionEvent.ACTION_DOWN) {
                    try {
                        if (!robotCommunicator.isConnected()) robotCommunicator.connect();
                        if (robotCommunicator.isConnected()) {
                            robotCommunicator.sendData("dir;C");
                            robotCommunicator.sendData("speed;" + String.valueOf(speed_mm_s));
                        } else {
                            Toast.makeText(getApplicationContext(), "Robot not connect. Please check WIFI connection.", Toast.LENGTH_SHORT).show();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                        try {
                            Toast.makeText(getApplicationContext(), "Robot reconnecting...", Toast.LENGTH_SHORT).show();
                            robotCommunicator.disconnect();
                            robotCommunicator.connect();
                        } catch (IOException e1) {
                            e1.printStackTrace();
                        }
                    }
                    return true;
                } else if (motionEvent.getAction() == MotionEvent.ACTION_UP) {
                    try {
                        if (!robotCommunicator.isConnected()) robotCommunicator.connect();
                        if (robotCommunicator.isConnected()) {
                            robotCommunicator.sendData("dir;S");
                        } else {
                            Toast.makeText(getApplicationContext(), "Robot not connect. Please check WIFI connection.", Toast.LENGTH_SHORT).show();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                        try {
                            Toast.makeText(getApplicationContext(), "Robot reconnecting...", Toast.LENGTH_SHORT).show();
                            robotCommunicator.disconnect();
                            robotCommunicator.connect();
                        } catch (IOException e1) {
                            e1.printStackTrace();
                        }
                    }
                    return true;
                }
                return false;
            }
        });
        isOnFollow.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                try {
                    if (!robotCommunicator.isConnected()) robotCommunicator.connect();
                    if (robotCommunicator.isConnected()) {
                        if (isOnFollow.isChecked()) {
                            robotCommunicator.sendData("sfol;ON");
                        } else {
                            robotCommunicator.sendData("sfol;OFF");
                        }
                    } else {
                        Toast.makeText(getApplicationContext(), "Robot not connect. Please check WIFI connection.", Toast.LENGTH_SHORT).show();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                    try {
                        Toast.makeText(getApplicationContext(), "Robot reconnecting...", Toast.LENGTH_SHORT).show();
                        robotCommunicator.disconnect();
                        robotCommunicator.connect();
                    } catch (IOException e1) {
                        e1.printStackTrace();
                    }
                }
            }
        });
        final double minLegVal = 0.4;
        final double maxLegVal = 1.2;
        legView.setText(new BigDecimal((double) legBar.getProgress() * ((maxLegVal - minLegVal) / 100) + minLegVal).setScale(2, BigDecimal.ROUND_HALF_UP).toString());
        legBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                legView.setText(new BigDecimal((double) i * ((maxLegVal - minLegVal) / 100) + minLegVal).setScale(2, BigDecimal.ROUND_HALF_UP).toString());
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                try {
                    if (!robotCommunicator.isConnected()) robotCommunicator.connect();
                    if (robotCommunicator.isConnected()) {
                        robotCommunicator.sendData("sleg;" + new BigDecimal((double) seekBar.getProgress() * ((maxLegVal - minLegVal) / 100) + minLegVal).multiply(new BigDecimal(1000)).setScale(0, BigDecimal.ROUND_HALF_UP).toString());
                    } else {
                        Toast.makeText(getApplicationContext(), "Robot not connect. Please check WIFI connection.", Toast.LENGTH_SHORT).show();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                    try {
                        Toast.makeText(getApplicationContext(), "Robot reconnecting...", Toast.LENGTH_SHORT).show();
                        robotCommunicator.disconnect();
                        robotCommunicator.connect();
                    } catch (IOException e1) {
                        e1.printStackTrace();
                    }
                }
            }
        });
        final double minDistanceVal = 0.4;
        final double maxDistanceVal = 1.2;
        distanceView.setText(new BigDecimal((double) distanceBar.getProgress() * ((maxDistanceVal - minDistanceVal) / 100) + minDistanceVal).setScale(2, BigDecimal.ROUND_HALF_UP).toString());
        distanceBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                distanceView.setText(new BigDecimal((double) i * ((maxDistanceVal - minDistanceVal) / 100) + minDistanceVal).setScale(2, BigDecimal.ROUND_HALF_UP).toString());
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                try {
                    if (!robotCommunicator.isConnected()) robotCommunicator.connect();
                    if (robotCommunicator.isConnected()) {
                        robotCommunicator.sendData("sdbrao;" + new BigDecimal((double) seekBar.getProgress() * ((maxDistanceVal - minDistanceVal) / 100) + minDistanceVal).multiply(new BigDecimal(1000)).setScale(0, BigDecimal.ROUND_HALF_UP).toString());
                    } else {
                        Toast.makeText(getApplicationContext(), "Robot not connect. Please check WIFI connection.", Toast.LENGTH_SHORT).show();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                    try {
                        Toast.makeText(getApplicationContext(), "Robot reconnecting...", Toast.LENGTH_SHORT).show();
                        robotCommunicator.disconnect();
                        robotCommunicator.connect();
                    } catch (IOException e1) {
                        e1.printStackTrace();
                    }
                }
            }
        });
    }
}
