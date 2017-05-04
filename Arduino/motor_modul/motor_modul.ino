#include <Wire.h>
/*I2C adresa*/
#define I2CDebug false
#define speedAndCurrentDebug false
#define addressI2C 0x01 //motory cislovane  z lava vzadu 
/*nastavenia merania */
#define measureCurrentAction 50
#define measureSpeedAction 25
#define measureDebugCurrent false    //pri merani nastaviť na true, meranie rýchlosti nastaviť na false
#define measureDebugSpeed false       //pri merani nastaviť na true, meranie prúdu nastaviť na false
#define numberOfMeasureData 600
#define numberOfMeasureDataCurrent 600 //600*0.0015=0.9s
#define numberOfMeasureDataSpeed 20 //20*0.25=5s
unsigned int measureData[numberOfMeasureData];
unsigned int measureIndex = 0;
bool measureStart = false;
/*--------------------------------------------------*/
/* ostatné nastavenia*/
#define numberOfTimer 3              //pocet casovacov
#define periodVoltageMeasure 1000000L //perioda pre prudovy regulator
#define periodCurrentRegulator 1500L //perioda pre prudovy regulator
#define periodSpeedRegulator 100000L //perioda pre rychlostny regulator
/* filter */
#define LPF_Beta 0.025 //konštanta dolnopriepustného filtra - https://kiritchatterjee.wordpress.com/2014/11/10/a-simple-digital-low-pass-filter-in-c/
#define useLPF true   //spustenie alebo vypnutie filtrácie prúdu
/* konštanty prúdového regulátora */
#define Pcurrent 2
#define Icurrent 0.8
#define Kbcurrent 0.2
volatile bool onCurrentReg = true;
/*konštanty rýchlostného regulátora */
#define Pspeed 0.30
#define Ispeed 0.27
#define Kbspeed 0.5
volatile bool onSpeedReg = true;
#define maxUspeddReg 500
/*--------------------------------------------------*/
/*pomocné premenné*/
unsigned long last_time = 0;
unsigned long sumCurrent = 0;
unsigned long timesCurrentMeasure = 0;

double regCurrentIsum = 0;
double regSpeedIsum = 0;
double pozadCurrent = 0;
double pozadSpeed = 0;
unsigned long time_integral[numberOfTimer];
long numberTickLast = 0;
volatile bool periodDone = false;
volatile long numberTick = 0;
volatile int numberTickForI2C = 0;
volatile unsigned char combLast = 0;
volatile char c = -1;
volatile int measureSpeed = 0;
volatile double averageMeasureCurrent = 0;
volatile unsigned int measureVoltage = 0;
byte measureSpeedToSend[2];
byte currentToSend[2];
byte numberTickToSend[2];
byte voltageToSend[2];
byte addressI2CToSend[] = {addressI2C};
/*--------------------------------------------------*/
void encoderMotor() {
  char pinA = digitalRead(2);
  char pinB = digitalRead(3);
  char comb = (pinB | (pinA << 1));
  if (combLast == 3) {
    if (comb == 2) {
      numberTick++;
      numberTickForI2C++;
      combLast = comb;
    }
    else if (comb == 1) {
      numberTick--;
      numberTickForI2C--;
      combLast = comb;
    }
  }
  else if (combLast == 1) {
    if (comb == 3) {
      numberTick++;
      numberTickForI2C++;
      combLast = comb;
    }
    else if (comb == 0) {
      numberTick--;
      numberTickForI2C--;
      combLast = comb;
    }
  }
  else if (combLast == 0) {
    if (comb == 1) {
      numberTick++;
      numberTickForI2C++;
      combLast = comb;
    }
    else if (comb == 2) {
      numberTick--;
      numberTickForI2C--;
      combLast = comb;
    }
  }
  else if (combLast == 2) {
    if (comb == 0) {
      numberTick++;
      numberTickForI2C++;
      combLast = comb;
    }
    else if (comb == 3) {
      numberTick--;
      numberTickForI2C--;
      combLast = comb;
    }
  }
  else {
    combLast = comb;
  }
  numberTickToSend[0] = (numberTickForI2C & 0xFF);
  numberTickToSend[1] = (numberTickForI2C >> 8) & 0xFF;
}
/*--------------------------------------------------*/
SIGNAL(TIMER1_OVF_vect)
{
  periodDone = true; //ak prejde perioda PWM signalu
}
/*--------------------------------------------------*/
void motor(int pwm) {
  if (pwm > 0) {
    digitalWrite(12, HIGH);
    digitalWrite(11, LOW);
  }
  else if (pwm < 0) {
    digitalWrite(12, LOW);
    digitalWrite(11, HIGH);
    pwm = pwm * -1;
  }
  else {
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    pwm = 255;
  }
  OCR1B = pwm;
}
/*--------------------------------------------------*/
void setup() {
  if (measureDebugCurrent || measureDebugSpeed || I2CDebug || speedAndCurrentDebug)
    Serial.begin(9600);

  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(A0, OUTPUT);
  digitalWrite(A2, INPUT);
  digitalWrite(A3, INPUT);
  digitalWrite(2, INPUT);
  digitalWrite(3, INPUT);

  attachInterrupt(digitalPinToInterrupt(2), encoderMotor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encoderMotor, CHANGE);
  char pinA = digitalRead(2);
  char pinB = digitalRead(3);
  combLast = (pinA | (pinB << 1));

  TIMSK1 = (1 << TOIE2);   // enable timer overflow interrupt
  TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0) | (0 << WGM11) | (1 << WGM10); //fast pwm,top=255
  TCCR1B = (0 << WGM13) | (1 << WGM12) | (0 << CS12) | (1 << CS11) | (1 << CS10); // 16Mhz/64/255=980hz
  motor(0);

  for (int i = 0; i < numberOfTimer; i++) {
    time_integral[i] = 0;
  }

  digitalWrite(13, HIGH);
  digitalWrite(A0, HIGH);

  Wire.begin(addressI2C);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  motor(0);
}
/*--------------------------------------------------*/
void receiveEvent(int howMany) {
  byte d = 0;
  byte e = 0;
  while (3 < Wire.available()) {
    Wire.read();
  }
  c = Wire.read();
  switch (c) {
    case 99:
      d = Wire.read();
      e = Wire.read();
      onSpeedReg = false;
      onCurrentReg = true;
      pozadCurrent = ((d << 8) | e);
      break;
    case 100:
      d = Wire.read();
      e = Wire.read();
      onSpeedReg = true;
      onCurrentReg = true;
      pozadSpeed = (d << 8) | e;
      break;
    case 101:
      d = Wire.read();
      e = Wire.read();
      onSpeedReg = false;
      onCurrentReg = false;
      motor((d << 8) | e);
      break;
  }
  if (I2CDebug) {
    Serial.print(d, DEC);
    Serial.print("/");
    Serial.print(e, DEC);
    Serial.print("/");
    Serial.println((d << 8) | e);
  }
}
/*--------------------------------------------------*/
void requestEvent() {
  switch (c) {
    case 1:
      Wire.write(measureSpeedToSend, 2);
      break;
    case 2:
      Wire.write(currentToSend, 2);
      break;
    case 3:
      Wire.write(numberTickToSend, 2);
      Serial.print(numberTickForI2C, DEC);
      Serial.print("/");
      numberTickForI2C = 0;
      numberTickToSend[0] = 0;
      numberTickToSend[1] = 0;
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
double uLastSpeed = 0;
double uLastCurrent = 0;
void loop() {
  /*--------------------------------------------------*/
  /*prikazy pouzitelne cez seriovy port pre meranie*/
  if ((measureDebugCurrent || measureDebugSpeed) && Serial.available() > 0) {
    byte byteSerial = Serial.read();
    if (byteSerial == 'g') {
      if ((numberOfMeasureDataCurrent <= measureIndex && measureDebugCurrent) || (numberOfMeasureDataSpeed <= measureIndex && measureDebugSpeed)) {
        Serial.println("E");
      }
      else {
        Serial.println(measureData[measureIndex++]);
      }
    }
    else if (byteSerial == 'r') {
      measureIndex = 0;
      measureStart = false;
    }
    else if (byteSerial == 's') {
      measureIndex = 0;
      measureStart = true;
      if (measureDebugSpeed)
        pozadSpeed = measureSpeedAction;
      else if (measureDebugCurrent) {
        pozadCurrent = measureCurrentAction;
        //motor(255); //prud pri merani
      }
    }
  }
  /*--------------------------------------------------*/
  /*spracovanie casu, aby sme robili kvazy presne periody pre regulatory*/
  noInterrupts();
  unsigned long actual_time = micros();
  int delta_time;
  if (actual_time > last_time)
    delta_time = actual_time - last_time;
  else
    delta_time = last_time - actual_time;
  for (int i = 0; i < numberOfTimer; i++)
    time_integral[i] += (long)delta_time;
  last_time = actual_time;
  /*--------------------------------------------------*/
  /*integrovanie prudu*/
  sumCurrent += analogRead(A3);
  timesCurrentMeasure++;
  interrupts();
  /*--------------------------------------------------*/
  /*v kazdej periode pridame hodnotu prudu cez dolnopriepustny filter, aby sme dostranili zasumenie*/
  if (periodDone) {
    noInterrupts();
    if (useLPF) {
      double raw_data = ((double)(sumCurrent / timesCurrentMeasure)) * (5 / 1.023);
      averageMeasureCurrent = (double)averageMeasureCurrent - (LPF_Beta * ((double)averageMeasureCurrent - (double)raw_data));
    }
    else {
      averageMeasureCurrent = ((double)(sumCurrent / timesCurrentMeasure)) * (5 / 1.023);
    }
    //priprava dat pre i2c
    currentToSend[0] = ((unsigned int)averageMeasureCurrent & 0xFF);
    currentToSend[1] = ((unsigned int)averageMeasureCurrent >> 8) & 0xFF;
    //-----------------
    sumCurrent = 0;
    timesCurrentMeasure = 0;
    periodDone = false;
    interrupts();
  }
  /*--------------------------------------------------*/
  /*regulator rychlosti*/
  if (time_integral[0] >= periodSpeedRegulator) {
    noInterrupts();
    /*--------------------------------------------------*/
    /*ziskana rychlost derivovanim polohy*/
    double speedWheel = numberTick - numberTickLast;
    numberTickLast = numberTick;
    measureSpeed = (int)speedWheel;
    //priprava dat pre i2c
    measureSpeedToSend[0] = (measureSpeed & 0xFF);
    measureSpeedToSend[1] = (measureSpeed >> 8) & 0xFF;
    //------
    if (speedAndCurrentDebug) {
      Serial.print(averageMeasureCurrent);
      Serial.print("/");
      Serial.println(speedWheel);
    }
    /*--------------------------------------------------*/
    if (onSpeedReg) {
      /*PI regulator rychlosti*/
      double e = pozadSpeed - speedWheel;
      regSpeedIsum += Ispeed * e;
      if (pozadSpeed >= 0) {
        if (uLastSpeed > maxUspeddReg) regSpeedIsum += -uLastSpeed + maxUspeddReg;
        else if (uLastSpeed < 0) regSpeedIsum += -uLastSpeed + 0;
      }
      else {
        if (uLastSpeed < -maxUspeddReg) regSpeedIsum += -uLastSpeed - maxUspeddReg;
        else if (uLastSpeed > 0) regSpeedIsum += -uLastSpeed + 0;
      }
      if (e == 0 && pozadSpeed == 0) regSpeedIsum = 0; //ak je pozadovana 0 a tiez odchylka 0 chceme, aby bol aj nulovy prud
      double u = e * Pspeed + regSpeedIsum;
      uLastSpeed = u;
      if (pozadSpeed >= 0) {
        if (u > maxUspeddReg) u = maxUspeddReg;
        else if (u < 0) u = 0;
      }
      else {
        if (u < -maxUspeddReg) u = -maxUspeddReg;
        else if (u > 0) u = 0;
      }
      pozadCurrent = u;

    }
    /*--------------------------------------------------*/
    /*sluzi iba pri merani*/
    if (measureStart && measureDebugSpeed) {
      if (numberOfMeasureDataSpeed <= measureIndex) {
        measureStart = false;
        measureIndex = 0;
        pozadSpeed = 0;
      }
      else {
        measureData[measureIndex++] = (unsigned char)speedWheel;
      }
    }

    /*--------------------------------------------------*/
    time_integral[0] = 0;
    interrupts();
  }
  /*--------------------------------------------------*/
  /*regulator prudu*/
  if (time_integral[1] >= periodCurrentRegulator) {
    noInterrupts();
    if (onCurrentReg) {
      /*--------------------------------------------------*/
      /*PI regulator prúdu*/
      double e;
      if (pozadCurrent > 0)
        e = pozadCurrent - averageMeasureCurrent;
      else
        e = -pozadCurrent - averageMeasureCurrent;
      regCurrentIsum += Icurrent * e;
      if (uLastCurrent > 255) regCurrentIsum += -uLastCurrent + 255;
      if (uLastCurrent < 0) regCurrentIsum += -uLastCurrent + 0;
      int u = (int)(e * Pcurrent + regCurrentIsum);
      uLastCurrent = u;
      if (u > 255) u = 255;
      else if (u < 0) u = 0;
      if (pozadCurrent > 0)
        motor(u);
      else
        motor(-u);

    }
    /*--------------------------------------------------*/
    /*sluzi iba pri merani*/
    if (measureStart && measureDebugCurrent) {
      if (numberOfMeasureDataCurrent <= measureIndex) {
        measureStart = false;
        measureIndex = 0;
        pozadCurrent = 0;
        //motor(0);
      }
      else {
        measureData[measureIndex++] = (unsigned int)averageMeasureCurrent;
      }
    }

    /*--------------------------------------------------*/
    time_integral[1] = 0;
    interrupts();
  }
  /*--------------------------------------------------*/
  /*meranie napatia*/
  if (time_integral[2] >= periodVoltageMeasure) {
    noInterrupts();
    measureVoltage = analogRead(A2);
    voltageToSend[0] = (measureVoltage & 0xFF);
    voltageToSend[1] = (measureVoltage >> 8) & 0xFF;
    time_integral[2] = 0;
    interrupts();
  }
  /*--------------------------------------------------*/
}
