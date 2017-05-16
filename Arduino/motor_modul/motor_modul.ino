#include <Wire.h>
/*I2C adresa*/
#define I2CDebug false
#define speedAndCurrentDebug false
#define addressI2C 0x06 //motory cislovane  z lava vzadu 
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
#define calcKp(p) (p)
#define calcKi(p,i,perioda) (p*(perioda/i))
#define calcKd(p,d,perioda) (p*(d/perioda))
//vypocet inkrementalnej casti regulatora
double calcQ0(double p, double d, double perioda) {
  return (calcKp(p) + calcKd(p, d, perioda));
}
double calcQ1(double p, double i, double d, double perioda) {
  return (-calcKp(p) - 2 * calcKd(p, d, perioda) + calcKi(p, i, perioda));
}
double calcQ2(double p, double d, double perioda) {
  return calcKd(p, d, perioda) ;
}
double calcP1(double p, double i, double d, double perioda) {
  return ((calcKi(p, i, perioda) - calcKd(p, d, perioda)) / (calcKp(p) + calcKi(p, i, perioda) + calcKd(p, d, perioda)));
}
double calcP2(double p, double i, double d, double perioda) {
  return ((calcKd(p, d, perioda)) / (calcKp(p) + calcKi(p, i, perioda) + calcKd(p, d, perioda)));
}
/*--------------------------------------------------*/
/* ostatné nastavenia*/
#define numberOfTimer 3              //pocet casovacov
#define periodVoltageMeasure 1000000L //perioda pre prudovy regulator [uS]
#define periodCurrentRegulator 1500L //perioda pre prudovy regulator [uS]
#define periodSpeedRegulator 150000L //perioda pre rychlostny regulator [uS]
/* filter */
#define LPF_Beta 0.025 //konštanta dolnopriepustného filtra - https://kiritchatterjee.wordpress.com/2014/11/10/a-simple-digital-low-pass-filter-in-c/
#define useLPF true   //spustenie alebo vypnutie filtrácie prúdu
/* premenne prúdového regulátora */
double pCurrent = 0.12;
double iCurrent = 0.006;
double dCurrent = 0;
#define uCurrentSaturationMax 255
#define uCurrentSaturationMin 0
volatile double q0Current;
volatile double q1Current;
volatile double q2Current;
volatile double p1Current;
volatile double p2Current;

void calcAllCurrentRegulatorParameters(double p, double i, double d, double perioda) {
  q0Current = calcQ0(p, d, perioda);
  q1Current = calcQ1(p, i, d, perioda);
  q2Current = calcQ2(p, d, perioda);
  p1Current = calcP1(p, i, d, perioda);
  p2Current = calcP2(p, i, d, perioda);
}

double umRegCurrent = 0;
double ueRegCurrent[] = {0, 0, 0};
double uRegCurrent[] = {0, 0};
double eRegCurrent[] = {0, 0, 0};
double pozadRegCurrent = 0;
volatile bool onCurrentReg = true;
/* premenne rychlostneho regulátora */
double pSpeed = 1.5;
double iSpeed = 1.1;
double dSpeed = 0;
#define uSpeedSaturationMax 600
#define uSpeedSaturationMin -600
volatile double q0Speed;
volatile double q1Speed;
volatile double q2Speed;
volatile double p1Speed;
volatile double p2Speed;

void calcAllSpeedRegulatorParameters(double p, double i, double d, double perioda) {
  q0Speed = calcQ0(p, d, perioda);
  q1Speed = calcQ1(p, i, d, perioda);
  q2Speed = calcQ2(p, d, perioda);
  p1Speed = calcP1(p, i, d, perioda);
  p2Speed = calcP2(p, i, d, perioda);
}

double umRegSpeed = 0;
double ueRegSpeed[] = {0, 0, 0};
double uRegSpeed[] = {0, 0};
double eRegSpeed[] = {0, 0, 0};
double pozadRegSpeed = 0;
volatile bool onSpeedReg = true;

/*pomocné premenné*/
unsigned long last_time = 0;
unsigned long sumCurrent = 0;
unsigned long timesCurrentMeasure = 0;
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

  calcAllCurrentRegulatorParameters(pCurrent, iCurrent, dCurrent, (double)periodCurrentRegulator / 1000000);
  calcAllSpeedRegulatorParameters(pSpeed, iSpeed, dSpeed, (double)periodSpeedRegulator / 1000000);

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
      pozadRegCurrent = ((d << 8) | e);
      break;
    case 100:
      d = Wire.read();
      e = Wire.read();
      onSpeedReg = true;
      onCurrentReg = true;
      pozadRegSpeed = (d << 8) | e;
      break;
    case 101:
      d = Wire.read();
      e = Wire.read();
      onSpeedReg = false;
      onCurrentReg = false;
      motor((d << 8) | e);
      break;
    case 102:
      d = Wire.read();
      e = Wire.read();
      pCurrent = ((double)((unsigned int)((d << 8) | e))) / 10000;
      if (I2CDebug) {
        Serial.print("PCurrent: ");
        Serial.println(pCurrent);
      }
      break;
    case 103:
      d = Wire.read();
      e = Wire.read();
      iCurrent = ((double)((unsigned int)((d << 8) | e))) / 10000;
      if (I2CDebug) {
        Serial.print("ICurrent: ");
        Serial.println(iCurrent);
      }
      break;
    case 104:
      d = Wire.read();
      e = Wire.read();
      dCurrent = ((double)((unsigned int)((d << 8) | e))) / 10000;
      if (I2CDebug) {
        Serial.print("DCurrent: ");
        Serial.println(dCurrent);
      }
      break;
    case 105:
      d = Wire.read();
      e = Wire.read();
      pSpeed = ((double)((unsigned int)((d << 8) | e))) / 10000;
      if (I2CDebug) {
        Serial.print("PSpeed: ");
        Serial.println(pSpeed);
      }
      break;
    case 106:
      d = Wire.read();
      e = Wire.read();
      iSpeed = ((double)((unsigned int)((d << 8) | e))) / 10000;
      if (I2CDebug) {
        Serial.print("ISpeed: ");
        Serial.println(iSpeed);
      }
      break;
    case 107:
      d = Wire.read();
      e = Wire.read();
      dSpeed = ((double)((unsigned int)((d << 8) | e))) / 10000;
      if (I2CDebug) {
        Serial.print("DSpeed: ");
        Serial.println(dSpeed);
      }
      break;
    case 108:
      calcAllCurrentRegulatorParameters(pCurrent, iCurrent, dCurrent, periodCurrentRegulator / 1000000);
      uRegCurrent[2] = 0;
      uRegCurrent[1] = 0;
      uRegCurrent[0] = 0;
      if (I2CDebug) {
        Serial.println("Calculate parameters current regulator");
      }
      break;
    case 109:
      calcAllSpeedRegulatorParameters(pSpeed, iSpeed, dSpeed, periodSpeedRegulator / 1000000);
      uRegSpeed[2] = 0;
      uRegSpeed[1] = 0;
      uRegSpeed[0] = 0;
      if (I2CDebug) {
        Serial.println("Calculate parameters speed regulator");
      }
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
      //Serial.print(numberTickForI2C, DEC);
      //Serial.print("/");
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
        pozadRegSpeed = measureSpeedAction;
      else if (measureDebugCurrent) {
        //pozadRegCurrent = measureCurrentAction;
        motor(200); //prud pri merani
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
      Serial.print(uRegCurrent[0]);
      Serial.print("/");
      Serial.print(pozadRegCurrent);
      Serial.print("/");
      Serial.print(averageMeasureCurrent);
      Serial.print("/");
      Serial.println(speedWheel);
    }
    /*--------------------------------------------------*/
    if (onSpeedReg) {
      /*Regulator rýchlosti*/
      eRegSpeed[2] = eRegSpeed[1]; //e(k-2)
      eRegSpeed[1] = eRegSpeed[0]; //e(k-1)
      eRegSpeed[0] = pozadRegSpeed - speedWheel; //e(k)

      uRegSpeed[1] = uRegSpeed[0];
      uRegSpeed[0] = uRegSpeed[1] + q0Speed * eRegSpeed[0] + q1Speed * eRegSpeed[1] + q2Speed * eRegSpeed[2] - p1Speed * ueRegSpeed[1] - p2Speed * ueRegSpeed[2];

      if (pozadRegSpeed > 0) {
        if (uRegSpeed[0] < 0)
          umRegSpeed = 0;
        else if (uRegSpeed[0] > uSpeedSaturationMax)
          umRegSpeed = uSpeedSaturationMax;
        else
          umRegSpeed = uRegSpeed[0];
      }
      else
      {
        if (uRegSpeed[0] < uSpeedSaturationMin)
          umRegSpeed = uSpeedSaturationMin;
        else if (uRegSpeed[0] > 0)
          umRegSpeed = 0;
        else
          umRegSpeed = uRegSpeed[0];
      }

      ueRegSpeed[2] = ueRegSpeed[1];
      ueRegSpeed[1] = ueRegSpeed[0];
      ueRegSpeed[0] = uRegSpeed[0] - umRegSpeed;
      if (pozadRegSpeed > 0) {
        if (uRegSpeed[0] < 0)
          uRegSpeed[0] = 0;
        else if (uRegSpeed[0] > uSpeedSaturationMax)
          uRegSpeed[0] = uSpeedSaturationMax;
        else
          uRegSpeed[0] = uRegSpeed[0];
      }
      else
      {
        if (uRegSpeed[0] < uSpeedSaturationMin)
          uRegSpeed[0] = uSpeedSaturationMin;
        else if (uRegSpeed[0] > 0) {
          uRegSpeed[0] = 0;
        }
        else
          uRegSpeed[0] = uRegSpeed[0];
      }

      if (pozadRegSpeed == 0) {
        uRegSpeed[2] = 0;
        uRegSpeed[1] = 0;
        uRegSpeed[0] = 0; // vynulujeme, pretoze chceme pozadovanu rychlost 0lovu, avsak koleso sa netoci, ale na motore je este stale prud, co je zbytocne, tak aby pri nulovej rychlosti sme sa dostali k nulovemu prudu
      }

      pozadRegCurrent = uRegSpeed[0];
    }
    /*--------------------------------------------------*/
    /*sluzi iba pri merani*/
    if (measureStart && measureDebugSpeed) {
      if (numberOfMeasureDataSpeed <= measureIndex) {
        measureStart = false;
        measureIndex = 0;
        pozadRegSpeed = 0;
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
      eRegCurrent[2] = eRegCurrent[1]; //e(k-2)
      eRegCurrent[1] = eRegCurrent[0]; //e(k-1)
      if (pozadRegCurrent > 0)
        eRegCurrent[0] = pozadRegCurrent - averageMeasureCurrent; //e(k)
      else
        eRegCurrent[0] = -pozadRegCurrent - averageMeasureCurrent; //e(k)

      uRegCurrent[1] = uRegCurrent[0];
      uRegCurrent[0] = uRegCurrent[1] + q0Current * eRegCurrent[0] + q1Current * eRegCurrent[1] + q2Current * eRegCurrent[2] - p1Current * ueRegCurrent[1] - p2Current * ueRegCurrent[2];

      if (uRegCurrent[0] < uCurrentSaturationMin)
        umRegCurrent = uCurrentSaturationMin;
      else if (uRegCurrent[0] > uCurrentSaturationMax)
        umRegCurrent = uCurrentSaturationMax;
      else
        umRegCurrent = uRegCurrent[0];

      ueRegCurrent[2] = ueRegCurrent[1];
      ueRegCurrent[1] = ueRegCurrent[0];
      ueRegCurrent[0] = uRegCurrent[0] - umRegCurrent;

      if (uRegCurrent[0] < uCurrentSaturationMin)
        uRegCurrent[0] = uCurrentSaturationMin;
      else if (uRegCurrent[0] > uCurrentSaturationMax)
        uRegCurrent[0] = uCurrentSaturationMax;
      else
        uRegCurrent[0] = uRegCurrent[0];

      if (pozadRegCurrent > 0)
        motor((int)uRegCurrent[0]);
      else
        motor(-(int)uRegCurrent[0]);
    }
    /*--------------------------------------------------*/
    /*sluzi iba pri merani*/
    if (measureStart && measureDebugCurrent) {
      if (numberOfMeasureDataCurrent <= measureIndex) {
        measureStart = false;
        measureIndex = 0;
        pozadRegCurrent = 0;
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
