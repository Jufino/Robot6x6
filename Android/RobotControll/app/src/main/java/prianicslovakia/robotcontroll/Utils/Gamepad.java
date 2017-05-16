package prianicslovakia.robotcontroll.Utils;

import android.os.Handler;
import android.os.Message;
import android.view.KeyEvent;

import java.util.Timer;
import java.util.TimerTask;

public class Gamepad {
    public interface GamepadChangeListener {
        public void onGamepadChangeEvent(String GamepadKey);
    }

    public interface GamepadEveryChangeListener {
        public void onGamepadEveryChangeEvent(String GamepadKey);
    }

    public interface GamepadChangeButtonListener {
        public void onGamepadChangeButtonEvent(String GamepadKey);
    }

    public interface GamepadEveryChangeButtonListener {
        public void onGamepadEveryChangeButtonEvent(String GamepadKey);
    }

    private GamepadChangeListener gamepadChangeListener;
    private GamepadChangeButtonListener gamepadChangeButtonListener;
    private GamepadEveryChangeListener gamepadEveryChangeListener;
    private GamepadEveryChangeButtonListener gamepadEveryChangeButtonListener;

    public GamepadChangeListener getGamepadChangeListener() {
        return gamepadChangeListener;
    }

    public void addGamepadChangeListener(GamepadChangeListener gamepadChangeListener) {
        this.gamepadChangeListener = gamepadChangeListener;
    }

    public GamepadChangeButtonListener getGamepadChangeButtonListener() {
        return gamepadChangeButtonListener;
    }

    public void addGamepadChangeButtonListener(GamepadChangeButtonListener gamepadChangeButtonListener) {
        this.gamepadChangeButtonListener = gamepadChangeButtonListener;
    }

    public GamepadEveryChangeListener getGamepadEveryChangeListener() {
        return gamepadEveryChangeListener;
    }

    public void addGamepadEveryChangeListener(GamepadEveryChangeListener gamepadEveryChangeListener) {
        this.gamepadEveryChangeListener = gamepadEveryChangeListener;
    }

    public GamepadEveryChangeButtonListener getGamepadEveryChangeButtonListener() {
        return gamepadEveryChangeButtonListener;
    }

    public void addGamepadEveryChangeButtonListener(GamepadEveryChangeButtonListener gamepadEveryChangeButtonListener) {
        this.gamepadEveryChangeButtonListener = gamepadEveryChangeButtonListener;
    }

    public Gamepad() {
        gamepadChangeButtonListener = null;
        gamepadChangeListener = null;
        gamepadEveryChangeButtonListener = null;
        gamepadEveryChangeListener = null;
    }

    int KEYCODE_GAMEPAD_DOWN = KeyEvent.KEYCODE_DPAD_RIGHT;
    int KEYCODE_GAMEPAD_UP = KeyEvent.KEYCODE_DPAD_LEFT;
    int KEYCODE_GAMEPAD_LEFT = KeyEvent.KEYCODE_DPAD_DOWN;
    int KEYCODE_GAMEPAD_RIGHT = KeyEvent.KEYCODE_DPAD_UP;
    int KEYCODE_GAMEPAD_IOS = KeyEvent.KEYCODE_DEL;
    int KEYCODE_GAMEPAD_A = KeyEvent.KEYCODE_BUTTON_B;
    int KEYCODE_GAMEPAD_X = KeyEvent.KEYCODE_DPAD_CENTER;
    int KEYCODE_GAMEPAD_TRIANGLE = KeyEvent.KEYCODE_BUTTON_Y;
    int KEYCODE_GAMEPAD_START = KeyEvent.KEYCODE_BUTTON_START;
    public static String GAMEPAD_DOWN = "DOWN";
    public static String GAMEPAD_UP = "UP";
    public static String GAMEPAD_LEFT = "LEFT";
    public static String GAMEPAD_RIGHT = "RIGHT";
    public static String GAMEPAD_UP_LEFT = GAMEPAD_UP + "-" + GAMEPAD_LEFT;
    public static String GAMEPAD_UP_RIGHT = GAMEPAD_UP + "-" + GAMEPAD_RIGHT;
    public static String GAMEPAD_DOWN_LEFT = GAMEPAD_DOWN + "-" + GAMEPAD_LEFT;
    public static String GAMEPAD_DOWN_RIGHT = GAMEPAD_DOWN + "-" + GAMEPAD_RIGHT;
    public static String GAMEPAD_A = "A";
    public static String GAMEPAD_IOS = "IOS";
    public static String GAMEPAD_X = "X";
    public static String GAMEPAD_TRIANGLE = "TRINGLE";
    public static String GAMEPAD_START = "START";
    public static String GAMEPAD_NOCLICK = "NOCLICK";

    private int last_keyCode = -1;
    private String PadClick = GAMEPAD_NOCLICK;
    private String lastPadClick = GAMEPAD_NOCLICK;
    private String ButtonCLick = GAMEPAD_NOCLICK;
    private String lastButtonClick = GAMEPAD_NOCLICK;
    Timer timerNoClick;
    Timer timerButtonNoClick;
    int timeReset = 500;
    int timeButtonReset = 500;

    public boolean onKeyDown(int keyCode, KeyEvent event) {
        if (keyCode == KEYCODE_GAMEPAD_DOWN) {
            setPadClick(GAMEPAD_DOWN);
            if (last_keyCode == KEYCODE_GAMEPAD_LEFT) {
                setPadClick(getPadClick() + "-" + GAMEPAD_LEFT);
            } else if (last_keyCode == KEYCODE_GAMEPAD_RIGHT) {
                setPadClick(getPadClick() + "-" + GAMEPAD_RIGHT);
            }
            last_keyCode = keyCode;
            if (gamepadEveryChangeListener != null) {
                onGamepadEveryChangeEvent.sendEmptyMessage(0);
            }
            if (gamepadChangeListener != null) {
                if (!lastPadClick.equals(getPadClick())) {
                    onGamepadChangeEvent.sendEmptyMessage(0);
                }
            }
            if (timerNoClick != null) timerNoClick.cancel();
            timerNoClick = new Timer();
            timerNoClick.schedule(new GamepadNoClick(), timeReset);
            return true;
        }
        if (keyCode == KEYCODE_GAMEPAD_UP) {
            setPadClick(GAMEPAD_UP);
            if (last_keyCode == KEYCODE_GAMEPAD_LEFT) {
                setPadClick(getPadClick() + "-" + GAMEPAD_LEFT);
            } else if (last_keyCode == KEYCODE_GAMEPAD_RIGHT) {
                setPadClick(getPadClick() + "-" + GAMEPAD_RIGHT);
            }
            last_keyCode = keyCode;
            if (gamepadEveryChangeListener != null) {
                onGamepadEveryChangeEvent.sendEmptyMessage(0);
            }
            if (gamepadChangeListener != null) {
                if (!lastPadClick.equals(getPadClick())) {
                    onGamepadChangeEvent.sendEmptyMessage(0);
                }
            }
            if (timerNoClick != null) timerNoClick.cancel();
            timerNoClick = new Timer();
            timerNoClick.schedule(new GamepadNoClick(), timeReset);
            return true;
        }
        if (keyCode == KEYCODE_GAMEPAD_RIGHT) {
            setPadClick("");
            if (last_keyCode == KEYCODE_GAMEPAD_UP) {
                setPadClick(GAMEPAD_UP + "-");
            } else if (last_keyCode == KEYCODE_GAMEPAD_DOWN) {
                setPadClick(GAMEPAD_DOWN + "-");
            }
            setPadClick(getPadClick() + GAMEPAD_RIGHT);
            last_keyCode = keyCode;
            if (gamepadEveryChangeListener != null) {
                onGamepadEveryChangeEvent.sendEmptyMessage(0);
            }
            if (gamepadChangeListener != null) {
                if (!lastPadClick.equals(getPadClick())) {
                    onGamepadChangeEvent.sendEmptyMessage(0);
                }
            }
            if (timerNoClick != null) timerNoClick.cancel();
            timerNoClick = new Timer();
            timerNoClick.schedule(new GamepadNoClick(), timeReset);
            return true;
        }
        if (keyCode == KEYCODE_GAMEPAD_LEFT) {
            setPadClick("");
            if (last_keyCode == KEYCODE_GAMEPAD_UP) {
                setPadClick(GAMEPAD_UP + "-");
            } else if (last_keyCode == KEYCODE_GAMEPAD_DOWN) {
                setPadClick(GAMEPAD_DOWN + "-");
            }
            setPadClick(getPadClick() + GAMEPAD_LEFT);
            last_keyCode = keyCode;
            if (gamepadEveryChangeListener != null) {
                onGamepadEveryChangeEvent.sendEmptyMessage(0);
            }
            if (gamepadChangeListener != null) {
                if (!lastPadClick.equals(getPadClick())) {
                    onGamepadChangeEvent.sendEmptyMessage(0);
                }
            }
            if (timerNoClick != null) timerNoClick.cancel();
            timerNoClick = new Timer();
            timerNoClick.schedule(new GamepadNoClick(), timeReset);
            return true;
        }

        if (keyCode == KEYCODE_GAMEPAD_IOS) {
            setButtonCLick(GAMEPAD_IOS);
            if (gamepadEveryChangeButtonListener != null) {
                onGamepadEveryChangeButtonEvent.sendEmptyMessage(0);
            }
            if (gamepadChangeButtonListener != null) {
                if (!lastButtonClick.equals(getButtonCLick())) {
                    onGamepadChangeButtonEvent.sendEmptyMessage(0);
                }
            }
            if (timerButtonNoClick != null) timerButtonNoClick.cancel();
            timerButtonNoClick = new Timer();
            timerButtonNoClick.schedule(new GamepadButtonNoClick(), timeButtonReset);
            return true;
        }
        if (keyCode == KEYCODE_GAMEPAD_TRIANGLE) {
            setButtonCLick(GAMEPAD_TRIANGLE);
            if (gamepadEveryChangeButtonListener != null) {
                onGamepadEveryChangeButtonEvent.sendEmptyMessage(0);
            }
            if (gamepadChangeButtonListener != null) {
                if (!lastButtonClick.equals(getButtonCLick())) {
                    onGamepadChangeButtonEvent.sendEmptyMessage(0);
                }
            }
            if (timerButtonNoClick != null) timerButtonNoClick.cancel();
            timerButtonNoClick = new Timer();
            timerButtonNoClick.schedule(new GamepadButtonNoClick(), timeButtonReset);
            return true;
        }
        if (keyCode == KEYCODE_GAMEPAD_A) {
            setButtonCLick(GAMEPAD_A);
            if (gamepadEveryChangeButtonListener != null) {
                onGamepadEveryChangeButtonEvent.sendEmptyMessage(0);
            }
            if (gamepadChangeButtonListener != null) {
                if (!lastButtonClick.equals(getButtonCLick())) {
                    onGamepadChangeButtonEvent.sendEmptyMessage(0);
                }
            }
            if (timerButtonNoClick != null) timerButtonNoClick.cancel();
            timerButtonNoClick = new Timer();
            timerButtonNoClick.schedule(new GamepadButtonNoClick(), timeButtonReset);
            return true;
        }
        if (keyCode == KEYCODE_GAMEPAD_X) {
            setButtonCLick(GAMEPAD_X);
            if (gamepadEveryChangeButtonListener != null) {
                onGamepadEveryChangeButtonEvent.sendEmptyMessage(0);
            }
            if (gamepadChangeButtonListener != null) {
                if (!lastButtonClick.equals(getButtonCLick())) {
                    onGamepadChangeButtonEvent.sendEmptyMessage(0);
                }
            }
            if (timerButtonNoClick != null) timerButtonNoClick.cancel();
            timerButtonNoClick = new Timer();
            timerButtonNoClick.schedule(new GamepadButtonNoClick(), timeButtonReset);
            return true;
        }
        if (keyCode == KEYCODE_GAMEPAD_START) {
            setButtonCLick(GAMEPAD_START);
            if (gamepadEveryChangeButtonListener != null) {
                onGamepadEveryChangeButtonEvent.sendEmptyMessage(0);
            }
            if (gamepadChangeButtonListener != null) {
                if (!lastButtonClick.equals(getButtonCLick())) {
                    onGamepadChangeButtonEvent.sendEmptyMessage(0);
                }
            }
            if (timerButtonNoClick != null) timerButtonNoClick.cancel();
            timerButtonNoClick = new Timer();
            timerButtonNoClick.schedule(new GamepadButtonNoClick(), timeButtonReset);
            return true;
        }
        return false;
    }

    public String getPadClick() {
        return PadClick;
    }

    public void setPadClick(String padClick) {
        PadClick = padClick;
    }

    public String getButtonCLick() {
        return ButtonCLick;
    }

    public void setButtonCLick(String buttonCLick) {
        ButtonCLick = buttonCLick;
    }

    class GamepadNoClick extends TimerTask {
        public void run() {
            setPadClick(GAMEPAD_NOCLICK);
            if (gamepadChangeListener != null) onGamepadChangeEvent.sendEmptyMessage(0);
            if (gamepadEveryChangeListener != null) onGamepadEveryChangeEvent.sendEmptyMessage(0);
            last_keyCode = -1;
        }
    }

    class GamepadButtonNoClick extends TimerTask {
        public void run() {
            setButtonCLick(GAMEPAD_NOCLICK);
            if (gamepadEveryChangeButtonListener != null)
                onGamepadEveryChangeButtonEvent.sendEmptyMessage(0);
            if (gamepadChangeButtonListener != null) onGamepadChangeButtonEvent.sendEmptyMessage(0);
        }
    }

    private Handler onGamepadChangeEvent = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            gamepadChangeListener.onGamepadChangeEvent(getPadClick());
        }
    };
    private Handler onGamepadEveryChangeEvent = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            gamepadEveryChangeListener.onGamepadEveryChangeEvent(getPadClick());
        }
    };
    private Handler onGamepadChangeButtonEvent = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            gamepadChangeButtonListener.onGamepadChangeButtonEvent(getButtonCLick());
        }
    };
    private Handler onGamepadEveryChangeButtonEvent = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            gamepadEveryChangeButtonListener.onGamepadEveryChangeButtonEvent(getButtonCLick());
        }
    };

}
