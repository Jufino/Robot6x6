#include <GY87.h>

GY87 gy87;

#define alfa 0.95
#define debug 1
#define pocMaxValue 10
#define callibrateCompass 0
#define offsetCompassX -87
#define offsetCompassY -56
int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;

float roll = 0;
float pitch = 0;
float yaw = 0;
int poc = 0;

unsigned long timer = 0;
float timeStep = 0.01;

void setup()
{
  if (debug || callibrateCompass) {
    Serial.begin(9600);
  }

  if (debug) {
    Serial.println("Starting IMU.");
    Serial.print("Inicialization GY87: ");
  }
  while (!gy87.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    if (debug) Serial.println("problem");
    delay(500);
  }
  if (debug) Serial.println("ok");
  // Enable bypass mode

  // Set measurement range
  gy87.setRangeCompass(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  gy87.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  gy87.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  gy87.setSamples(HMC5883L_SAMPLES_8);

  if (debug) Serial.println("Compass calibrate.");

  if (callibrateCompass)
    gy87.setOffset(0, 0);
  else
    gy87.setOffset(offsetCompassX, offsetCompassY);

  if (debug) Serial.println("Gyroscope calibrate.");
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  gy87.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  gy87.setThreshold(3);
  if (debug) Serial.println("Start measuring...");
}

// Tilt compensation
float tiltCompensate(Vector mag, float roll, float pitch)
{
  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  {
    return -10;
  }

  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // Tilt compensation
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;

  float heading = atan2(Yh, Xh);

  return heading;
}

// Correct angle
float correctAngle(float heading)
{
  if (heading < 0) {
    heading += 2 * PI;
  }
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }
  return heading;
}

void loop()
{
  timer = millis();
//    if (debug) Serial.println("Read gyro...");
  Vector gy = gy87.readNormalizeGyro();
//      if (debug) Serial.println("Read compass...");
  Vector mag = gy87.readNormalizeCompass();
//        if (debug) Serial.println("Read accelerometer...");
  Vector acc = gy87.readScaledAccel();
  float pitchAcc = atan2f(acc.YAxis, acc.ZAxis);
  float rollAcc = atan2f(acc.XAxis, acc.ZAxis);

  roll += gy.XAxis * timeStep * (PI / 180);
  pitch += gy.YAxis * timeStep * (PI / 180);
  yaw -= gy.ZAxis * timeStep * (PI / 180);

  pitch = pitch * alfa + pitchAcc * (1-alfa);
  roll = roll * alfa + rollAcc * (1-alfa);

  float compassVal = tiltCompensate(mag, roll, pitch);
  if (compassVal != -10) {
    compassVal += (4.0 + (26.0 / 60.0)) / (180 / M_PI);
    compassVal = correctAngle(compassVal);
    yaw = correctAngle(yaw);
    yaw = yaw * alfa + compassVal * (1-alfa);
  }

  if (callibrateCompass && poc++ == pocMaxValue) {
    Vector magRaw = gy87.readRawCompass();

    if (magRaw.XAxis < minX) minX = magRaw.XAxis;
    if (magRaw.XAxis > maxX) maxX = magRaw.XAxis;
    if (magRaw.YAxis < minY) minY = magRaw.YAxis;
    if (magRaw.YAxis > maxY) maxY = magRaw.YAxis;

    offX = (maxX + minX) / 2;
    offY = (maxY + minY) / 2;

    Serial.print(mag.XAxis);
    Serial.print(":");
    Serial.print(mag.YAxis);
    Serial.print(":");
    Serial.print(minX);
    Serial.print(":");
    Serial.print(maxX);
    Serial.print(":");
    Serial.print(minY);
    Serial.print(":");
    Serial.print(maxY);
    Serial.print(":");
    Serial.print(offX);
    Serial.print(":");
    Serial.print(offY);
    Serial.print("\n");
    poc = 0;
  }
  else if (debug && poc++ == pocMaxValue) {
    Serial.print("roll:");
    Serial.print(roll * 180 / M_PI);
    Serial.print("; pitch:");
    Serial.print(pitch * 180 / M_PI);
    Serial.print("; yaw:");
    Serial.println(yaw * 180 / M_PI);
    poc = 0;
  }
//delay(0.01);
  delay((timeStep * 1000) - (millis() - timer));
}

