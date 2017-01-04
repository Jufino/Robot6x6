/*nastavenia merania */
#define measureDebugCurrent false    //pri merani nastaviť na true, meranie rýchlosti nastaviť na false
#define measureDebugSpeed true       //pri merani nastaviť na true, meranie prúdu nastaviť na false
#define numberOfMeasureData 600
#define numberOfMeasureDataCurrent 600 //600*0.0015=0.9s
#define numberOfMeasureDataSpeed 20 //20*0.25=5s
unsigned int measureData[numberOfMeasureData];
unsigned int measureIndex = 0;
bool measureStart = false;
/*--------------------------------------------------*/
/* ostatné nastavenia*/
#define numberOfTimer 2              //pocet casovacov
#define periodCurrentRegulator 1500L //perioda pre prudovy regulator
#define periodSpeedRegulator 250000L //perioda pre rychlostny regulator
#define LPF_Beta 0.025 //konštanta dolnopriepustného filtra - https://kiritchatterjee.wordpress.com/2014/11/10/a-simple-digital-low-pass-filter-in-c/
#define useLPF true   //spustenie alebo vypnutie filtrácie prúdu
/* konštanty prúdového regulátora */
#define Pcurrent 3.02
#define Icurrent 0.6
/*konštanty rýchlostného regulátora */
#define Pspeed 0.3
#define Ispeed 0.35
/*--------------------------------------------------*/
/*pomocné premenné*/
unsigned long last_time = 0;
unsigned long sumCurrent = 0;
unsigned long timesCurrentMeasure = 0;
double averageMeasureCurrent = 0;
double regCurrentIsum = 0;
double regSpeedIsum = 0;
double pozadCurrent = 0;
double pozadSpeed = 0;
unsigned long time_integral[numberOfTimer];
long numberTickLast = 0;
volatile bool periodDone = false;
volatile long numberTick = 0;
volatile unsigned char combLast = 0;
/*--------------------------------------------------*/
void encoderMotor() {
  char pinA = digitalRead(2);
  char pinB = digitalRead(3);
  char comb = (pinB | (pinA << 1));
  if (combLast == 3) {
    if (comb == 2) {
      numberTick++;
      combLast = comb;
    }
    else if (comb == 1) {
      numberTick--;
      combLast = comb;
    }
  }
  else if (combLast == 1) {
    if (comb == 3) {
      numberTick++;
      combLast = comb;
    }
    else if (comb == 0) {
      numberTick--;
      combLast = comb;
    }
  }
  else if (combLast == 0) {
    if (comb == 1) {
      numberTick++;
      combLast = comb;
    }
    else if (comb == 2) {
      numberTick--;
      combLast = comb;
    }
  }
  else if (combLast == 2) {
    if (comb == 0) {
      numberTick++;
      combLast = comb;
    }
    else if (comb == 3) {
      numberTick--;
      combLast = comb;
    }
  }
  else {
    combLast = comb;
  }
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
  if (measureDebugCurrent || measureDebugSpeed)
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
}
/*--------------------------------------------------*/
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
        pozadSpeed = 150; //rychlost pri merani
      else if (measureDebugCurrent)
        pozadCurrent = 200; //prud pri merani
    }
  }
  /*--------------------------------------------------*/
  /*spracovanie casu, aby sme robili kvazy presne periody pre regulatory*/
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
  /*--------------------------------------------------*/
  /*v kazdej periode pridame hodnotu prudu cez dolnopriepustny filter, aby sme dostranili zasumenie*/
  if (periodDone) {
    if (useLPF) {
      double raw_data = ((double)(sumCurrent / timesCurrentMeasure)) * (5 / 1.023);
      averageMeasureCurrent = (double)averageMeasureCurrent - (LPF_Beta * ((double)averageMeasureCurrent - (double)raw_data));
    }
    else {
      averageMeasureCurrent = ((double)(sumCurrent / timesCurrentMeasure)) * (5 / 1.023);
    }
    sumCurrent = 0;
    timesCurrentMeasure = 0;
    periodDone = false;
  }
  /*--------------------------------------------------*/
  /*regulator rychlosti*/
  if (time_integral[0] >= periodSpeedRegulator) {
    /*--------------------------------------------------*/
    /*ziskana rychlost derivovanim polohy*/
    double speedWheel = numberTick - numberTickLast;
    numberTickLast = numberTick;
    /*--------------------------------------------------*/
    /*PI regulator rychlosti*/
    double e = pozadSpeed - speedWheel;
    regSpeedIsum += Ispeed * e;
    if (regSpeedIsum > 500) regSpeedIsum = 500;
    else if (regSpeedIsum < -500) regSpeedIsum = -500;
    else if (e == 0 && pozadSpeed == 0) regSpeedIsum = 0; //ak je pozadovana 0 a tiez odchylka 0 chceme, aby bol aj nulovy prud
    double u = e * Pspeed + regSpeedIsum;
    if (pozadSpeed >= 0) {
      if (u > 500) u = 500;
      else if (u < 0) u = 0;
    }
    else {
      if (u < -500) u = -500;
      else if (u > 0) u = 0;
    }
    pozadCurrent = u;
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
  }
  /*--------------------------------------------------*/
  /*regulator prudu*/
  if (time_integral[1] >= periodCurrentRegulator) {
    /*--------------------------------------------------*/
    /*PI regulator prúdu*/
    double e = pozadCurrent - averageMeasureCurrent;
    regCurrentIsum += Icurrent * e;
    if (regCurrentIsum > 255) regCurrentIsum = 255;
    else if (regCurrentIsum < -255) regCurrentIsum = -255;
    int u = (int)(e * Pcurrent + regCurrentIsum);
    if (pozadCurrent >= 0) {
      if (u > 255) u = 255;
      else if (u < 0) u = 0;
    }
    else {
      if (u < -255) u = -255;
      else if (u > 0) u = 0;
    }
    motor(u);
    /*--------------------------------------------------*/
    /*sluzi iba pri merani*/
    if (measureStart && measureDebugCurrent) {
      if (numberOfMeasureDataCurrent <= measureIndex) {
        measureStart = false;
        measureIndex = 0;
        pozadCurrent = 0;
      }
      else {
        measureData[measureIndex++] = (char)averageMeasureCurrent;
      }
    }
    /*--------------------------------------------------*/
    time_integral[1] = 0;
  }
  /*--------------------------------------------------*/
}
