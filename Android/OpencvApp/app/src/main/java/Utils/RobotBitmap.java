package Utils;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Matrix;

import java.io.ByteArrayInputStream;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.nio.charset.StandardCharsets;

public class RobotBitmap{
    private Bitmap bitmap;
    private Float fps = 0F;
    private float size=0;
    String nazovImg="";
    RobotCommunicator robotCommunicator;

    public RobotBitmap(RobotCommunicator robotCommunicator,String nazovImg){
        this.nazovImg = nazovImg;
        this.robotCommunicator = robotCommunicator;
    }

    public Bitmap getBitmap() {
        return bitmap;
    }

    public void setBitmap(Bitmap bitmap) {
        this.bitmap = bitmap;
    }

    public Float getFps() {
        return fps;
    }

    public void setFps(Float fps) {
        this.fps = fps;
    }
    public Bitmap receiveBitmap() {
        try {
            long lastTime = System.nanoTime();
            robotCommunicator.sendData(nazovImg);
            String length = new String(robotCommunicator.receiveBytes(8), StandardCharsets.UTF_8);
            Thread.sleep(10);
            int lengthInt = Integer.parseInt(length);
            byte[] obrByte = robotCommunicator.receiveBytes(lengthInt);
            ByteArrayInputStream input = new ByteArrayInputStream(obrByte);
            Bitmap decodedBitmap = BitmapFactory.decodeStream(input);
            if (decodedBitmap != null) bitmap = decodedBitmap;
            /*vypocet fps*/
            long now = System.nanoTime();
            double fps = 1D/((now - lastTime) / 1000000000D);
            setFps(new BigDecimal(Double.toString(fps)).setScale(2, RoundingMode.HALF_UP).floatValue());
            /**/
            setSize(lengthInt/1024);
            Thread.sleep(40);
            return bitmap;
        } catch (Exception x) {
            x.printStackTrace();
        }
        return bitmap;
    }
    private Bitmap getResizedBitmap(Bitmap bm, int newWidth, int newHeight) {
        int width = bm.getWidth();
        int height = bm.getHeight();
        float scaleWidth = ((float) newWidth) / width;
        float scaleHeight = ((float) newHeight) / height;
        Matrix matrix = new Matrix();
        matrix.postScale(scaleWidth, scaleHeight);
        Bitmap resizedBitmap = Bitmap.createBitmap(
                bm, 0, 0, width, height, matrix, false);
        bm.recycle();
        return resizedBitmap;
    }

    public float getSize() {
        return size;
    }

    public void setSize(float size) {
        this.size = size;
    }
}