#include <Wire.h>
#include <GY87.h>
#include <EEPROM.h>

#define addressI2C 0x11
GY87 gy87;

#define alfaAccelAndGyro 0.95 //pomer zberu dat medzi accelerometrom a gyroscopom
#define alfaCompass 0.80      //dolnopriepustny filter

#define debug 1
#define I2CDebug 1
#define printDegree 1
#define printCalibrate 1
#define pocMaxValue 10
bool callibrateCompassOn = true;
bool callibrateGyOn = false;
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

volatile char c = -1;
byte rollToSend[2];
byte pitchToSend[2];
byte yawToSend[2];
byte addressI2CToSend[] = {addressI2C};

void setup()
{
  delay(500);
  if (debug) {
    Serial.begin(9600);
    Serial.println("Starting IMU.");
    Serial.print("Inicialization GY87: ");
  }
  while (!gy87.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    if (debug) Serial.println("problem");
    delay(500);
  }
  if (debug) Serial.println("ok");

  gy87.setRangeCompass(HMC5883L_RANGE_1_3GA);
  gy87.setMeasurementMode(HMC5883L_CONTINOUS);
  gy87.setDataRate(HMC5883L_DATARATE_30HZ);
  gy87.setSamples(HMC5883L_SAMPLES_8);

  if (EEPROM.length() > 0) {
    offX = EEPROM.read(0);
    offY = EEPROM.read(1);
  }
  if (debug) {
    Serial.print("Compass set offset: X:");
    Serial.print(offX);
    Serial.print(", Y:");
    Serial.println(offY);
  }
  gy87.setOffset(offX, offY);

  if (debug) Serial.println("Gyroscope calibrate.");
  // Kalibracia gyroskopu, musi byt v klude
  gy87.calibrateGyro();

  // Nastavenie citlivosti, odstranenie zasumenia.
  gy87.setThreshold(3);

  Wire.begin(addressI2C);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  if (debug) Serial.println("Start measuring...");
}

float tiltCompensate(Vector mag, float roll, float pitch)
{
  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  {
    return -10; // ak je compas moc nakloneny, tak hadze hodnotu mimo rozsahu
  }

  //Predpocitanie cosinusov a sinusov, kedze sa pouzivaju viackrat
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // Kompenzacia naklonenia
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;

  float heading = atan2(Yh, Xh);

  return heading;
}

// Korekcia uhla v rozpeti 0 az 2*PI
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

/*--------------------------------------------------*/
void receiveEvent(int howMany) {
  while (1 < Wire.available()) {
    Wire.read();
  }
  c = Wire.read();
  switch (c) {
    case 99: // calibrate compass
      if (callibrateCompassOn) {
        EEPROM.write(0, offX);
        EEPROM.write(1, offY);
        gy87.setOffset(offX, offY);
        if (printCalibrate) {
          Serial.print("LOG/Offset compass X:");
          Serial.print(offX);
          Serial.print(", Y:");
          Serial.println(offY);
        }
        callibrateCompassOn = false;
      }
      else {
        callibrateCompassOn = true;
      }
      break;
    case 100: // calibrate gyroscope
      callibrateGyOn = true;
      break;
  }
}
/*--------------------------------------------------*/
void requestEvent() {
  switch (c) {
    case 1:
      Wire.write(rollToSend, 2);
      break;
    case 2:
      Wire.write(rollToSend, 2);
      break;
    case 3:
      Wire.write(yawToSend, 2);
      break;
    case 111:
      Wire.write(addressI2CToSend, 1);
      break;
  }
  if (I2CDebug) {
    Serial.println(c, DEC);
  }
  c = -1;
}

void loop()
{
  timer = millis();
  //-------------------------------------------------------
  //vypocet vsetkych uhlov zariadenia
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
  //yaw -= gy.ZAxis * timeStep * (PI / 180);

  pitch = pitch * alfaAccelAndGyro + pitchAcc * (1 - alfaAccelAndGyro);
  roll = roll * alfaAccelAndGyro + rollAcc * (1 - alfaAccelAndGyro);

  float compassVal = tiltCompensate(mag, roll, pitch);
  if (compassVal != -10) {
    compassVal += (4.0 + (26.0 / 60.0)) / (180 / M_PI);
    compassVal = correctAngle(compassVal);
    yaw = yaw * alfaCompass + compassVal * (1 - alfaCompass);
    yaw = correctAngle(yaw);
    signed int  yawNormToSend = yaw * 100;
    yawToSend[0] = (yawNormToSend & 0xFF);
    yawToSend[1] = (yawNormToSend >> 8) & 0xFF;
  }
  else {
    signed int  yawNormToSend = -10;
    yawToSend[0] = (yawNormToSend & 0xFF);
    yawToSend[1] = (yawNormToSend >> 8) & 0xFF;
  }
  //-------------------------------------------------------
  //priprav data na odoslanie
  signed int rollNormToSend = roll * 100;
  rollToSend[0] = (rollNormToSend & 0xFF);
  rollToSend[1] = (rollNormToSend >> 8) & 0xFF;

  signed int pitchNormToSend = pitch * 100;
  pitchToSend[0] = (pitchNormToSend & 0xFF);
  pitchToSend[1] = (pitchNormToSend >> 8) & 0xFF;

  //-------------------------------------------------------
  //kalibracia stredu kompasu
  if (callibrateCompassOn) {
    Vector magRaw = gy87.readRawCompass();

    if (magRaw.XAxis < minX) minX = magRaw.XAxis;
    if (magRaw.XAxis > maxX) maxX = magRaw.XAxis;
    if (magRaw.YAxis < minY) minY = magRaw.YAxis;
    if (magRaw.YAxis > maxY) maxY = magRaw.YAxis;

    offX = (maxX + minX) / 2;
    offY = (maxY + minY) / 2;
  }
  //-------------------------------------------------------
  //kalibracia zasumenia gyroskopu
  if (callibrateGyOn) {
    gy87.calibrateGyro();
    callibrateGyOn = false;
  }
  //-------------------------------------------------------
  //debugovanie cez seriovy port
  if (debug && poc++ == pocMaxValue) {
    if (printDegree) {
      Serial.print("roll:");
      Serial.print(roll * 180 / M_PI);
      Serial.print("; pitch:");
      Serial.print(pitch * 180 / M_PI);
      Serial.print("; yaw:");
      Serial.println(yaw * 180 / M_PI);
    }
    if (printCalibrate) {
      Serial.print("Offset compass X:");
      Serial.print(offX);
      Serial.print(", Y:");
      Serial.println(offY);
    }
    poc = 0;
  }
  //-------------------------------------------------------
  delay((timeStep * 1000) - (millis() - timer)); //dobehnutie casu, ktory ostal
}

