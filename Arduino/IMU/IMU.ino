#include <Wire.h>
#include <GY87.h>
#include <EEPROM.h>

#define addressI2C 0x11
GY87 gy87;

#define alfaAccelAndGyro 0.95 //pomer zberu dat medzi accelerometrom a gyroscopom
#define alfaCompass 0.99      //dolnopriepustny filter
#define alfaFilterCompass 0.5

#define debug 1
#define I2CDebug 0
#define printDegree 1
#define printCalibrate 0
#define printVoltage 0
#define pocMaxValue 10
bool callibrateCompassOn = false || printCalibrate;
bool callibrateGyOn = false;
float minX = 0;
float maxX = 0;
float minY = 0;
float maxY = 0;
float minZ = 0;
float maxZ = 0;
float offX = 0;
float offY = 0;
float offZ = 0;
float xMagneticFiltered = 0;
float yMagneticFiltered = 0;
float zMagneticFiltered = 0;


float roll = 0;
float pitch = 0;
float yaw = 0;
int poc = 0;
int sumVoltage = 0;
int pocVoltage = 0;
unsigned long timer = 0;
float timeStep = 0.01;

volatile char c = -1;
byte rollToSend[2];
byte pitchToSend[2];
byte yawToSend[2];
byte voltageToSend[2];
byte addressI2CToSend[] = {addressI2C};

void setup()
{
  Wire.begin(addressI2C);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

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

  if (!callibrateCompassOn) {
    offX = 46;
    offY = -214;
    offZ = -318;
    EEPROM.update(0, (int)offX);
    EEPROM.update(1, (int)offX >> 8);
    EEPROM.update(2, (int)offY);
    EEPROM.update(3, (int)offY >> 8);
    EEPROM.update(4, (int)offZ);
    EEPROM.update(5, (int)offZ >> 8);
    if (EEPROM.length() > 0) {
      offX = EEPROM.read(0) | (EEPROM.read(1) << 8);
      offY = EEPROM.read(2) | (EEPROM.read(3) << 8);
      offZ = EEPROM.read(4) | (EEPROM.read(5) << 8);
    }
    gy87.setOffset(offX, offY, offZ);
  }
  else {
    gy87.setOffset(0, 0, 0);
  }

  if (debug) {
    Serial.print("Compass set offset: X:");
    Serial.print(offX);
    Serial.print(", Y:");
    Serial.print(offY);
    Serial.print(", Z:");
    Serial.println(offZ);
  }


  if (debug) Serial.println("Gyroscope calibrate.");
  // Kalibracia gyroskopu, musi byt v klude
  gy87.calibrateGyro();

  // Nastavenie citlivosti, odstranenie zasumenia.
  gy87.setThreshold(3);

  if (debug) Serial.println("Start measuring...");
  delay(500);
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
      gy87.setOffset(0, 0, 0);
      callibrateCompassOn = true;
      break;
    case 100:
      if (callibrateCompassOn) {
        EEPROM.update(0, (int)offX);
        EEPROM.update(1, (int)offX >> 8);
        EEPROM.update(2, (int)offY);
        EEPROM.update(3, (int)offY >> 8);
        EEPROM.update(4, (int)offZ);
        EEPROM.update(5, (int)offZ >> 8);
        gy87.setOffset((int)offX, (int)offY, (int)offZ);
        if (printCalibrate) {
          Serial.print("LOG/Offset compass X:");
          Serial.print(offX);
          Serial.print(", Y:");
          Serial.print(offY);
          Serial.print(", Z:");
          Serial.println(offZ);
        }
        callibrateCompassOn = false;
      }
      break;
    case 101: // calibrate gyroscope
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
      Wire.write(pitchToSend, 2);
      break;
    case 3:
      Wire.write(yawToSend, 2);
      break;
    case 4:
      Wire.write(voltageToSend, 2);
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
  noInterrupts();
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
  float rollAcc = atan2f(-acc.XAxis, acc.ZAxis);

  roll += gy.XAxis * timeStep * (PI / 180);
  pitch += gy.YAxis * timeStep * (PI / 180);
  yaw -= gy.ZAxis * timeStep * (PI / 180);

  pitch = pitch * alfaAccelAndGyro + pitchAcc * (1 - alfaAccelAndGyro);
  roll = roll * alfaAccelAndGyro + rollAcc * (1 - alfaAccelAndGyro);

  //float compassVal = atan2(mag.YAxis, mag.XAxis);
  float compassVal =  tiltCompensate(mag, rollAcc, pitchAcc);
  if (compassVal != -10) {
    compassVal += (4.0 + (26.0 / 60.0)) / (180 / M_PI);
    yaw = yaw * alfaCompass + compassVal * (1 - alfaCompass);
    yaw = correctAngle(compassVal);
    int16_t  yawNormToSend = (int16_t)(yaw * 10000);
    yawToSend[0] = yawNormToSend;
    yawToSend[1] = yawNormToSend >> 8;
  }
  else {
    int16_t  yawNormToSend = -65000;
    yawToSend[0] = yawNormToSend;
    yawToSend[1] = yawNormToSend >> 8;
  }
  //-------------------------------------------------------
  //priprav data na odoslanie
  int16_t rollNormToSend = (int16_t)(roll * 10000);
  rollToSend[0] = rollNormToSend;
  rollToSend[1] = rollNormToSend >> 8;

  int16_t pitchNormToSend = (int16_t)(pitch * 10000);
  pitchToSend[0] = pitchNormToSend;
  pitchToSend[1] = pitchNormToSend >> 8;

  //-------------------------------------------------------
  //kalibracia stredu kompasu
  if (callibrateCompassOn) {
    Vector magRaw = gy87.readRawCompass();

    xMagneticFiltered = xMagneticFiltered * alfaFilterCompass + magRaw.XAxis * (1 - alfaFilterCompass);
    yMagneticFiltered = yMagneticFiltered * alfaFilterCompass + magRaw.YAxis * (1 - alfaFilterCompass);
    zMagneticFiltered = zMagneticFiltered * alfaFilterCompass + magRaw.ZAxis * (1 - alfaFilterCompass);

    if (xMagneticFiltered < minX) minX = xMagneticFiltered;
    if (xMagneticFiltered > maxX) maxX = xMagneticFiltered;
    if (yMagneticFiltered < minY) minY = yMagneticFiltered;
    if (yMagneticFiltered > maxY) maxY = yMagneticFiltered;
    if (zMagneticFiltered < minZ) minZ = zMagneticFiltered;
    if (zMagneticFiltered > maxZ) maxZ = zMagneticFiltered;

    offX = (maxX + minX) / 2;
    offY = (maxY + minY) / 2;
    offZ = (maxZ + minZ) / 2;
  }
  //-------------------------------------------------------
  //kalibracia zasumenia gyroskopu
  if (callibrateGyOn) {
    gy87.calibrateGyro();
    callibrateGyOn = false;
  }

  //bateria napatie
  uint16_t voltageNormToSend = 0;
  if (pocVoltage < 30) {
    sumVoltage += analogRead(A6);
    pocVoltage++;
  }
  else {
    voltageNormToSend = sumVoltage / pocVoltage;
    voltageToSend[0] = voltageNormToSend;
    voltageToSend[1] = voltageNormToSend >> 8;
    pocVoltage = 0;
    sumVoltage = 0;
  }


  interrupts();
  //-------------------------------------------------------
  //debugovanie cez seriovy port
  if (debug && poc++ >= pocMaxValue) {
    if (printDegree) {
      Serial.print("roll:");
      Serial.print(roll * 180 / M_PI);
      Serial.print("; pitch:");
      Serial.print(pitch * 180 / M_PI);
      Serial.print("; yaw:");
      Serial.println(yaw * 180 / M_PI);
    }
    if (printCalibrate && callibrateCompassOn) {
      Serial.print("Magnetic X:");
      Serial.print(xMagneticFiltered);
      Serial.print(", Y:");
      Serial.print(yMagneticFiltered);
      Serial.print(", Z:");
      Serial.print(zMagneticFiltered);
      Serial.print("; Offset compass X:");
      Serial.print(offX);
      Serial.print(", Y:");
      Serial.print(offY);
      Serial.print(", Z:");
      Serial.println(offZ);
    }
    if (printVoltage) {
      Serial.println(voltageNormToSend);
    }
    poc = 0;
  }
  //-------------------------------------------------------
  int16_t ostatokCasu = (timeStep * 1000) - (millis() - timer);
  if (ostatokCasu > 0)
    delay(ostatokCasu); //dobehnutie casu, ktory ostal
}

