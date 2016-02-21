#include <Wire.h>
#include <avr/wdt.h>
volatile boolean stopUprava = false;
volatile unsigned int pocetTikov1 = 0;
volatile unsigned int pocetTikov4 = 0;
volatile char c = -1;
int lastRychlost1 = 0;
int lastRychlost4 = 0;
int lastVzdialenost1 = 0;
int lastVzdialenost4 = 0;
int lastNapetie = 0;
int lastPrud = 0;
char smer1 = 0;
char smer4 = 0;
volatile boolean onReg1 = true;
volatile boolean onReg4 = true;
volatile int setRychlost1=0;
volatile int setRychlost4=0;
volatile int vykonajOK = false;

//pin 10,11 +,- 12V
//pin 2 - // otackomer motor 1
//pin 3 - // otackomer motor 4
//LED A3 - cervena
//LED A2 - zelena
//LED 13 - modra
//A0 napetie
//A1 prud
#define LCervena A3
#define LZelena A2
#define LModra 13
#define Napetie A0
#define Prud A1
#define watchdogStatus 0
#define serialPortStatus 0

void napajanie12V(boolean stav) {
  if (stav == true) {
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
  }
  else {
    digitalWrite(10, HIGH);
    digitalWrite(11, HIGH);
  }

}
void motor(int motor, char smer, int rychlost) {
  if (motor == 1) {
    if (smer < 0) {
      digitalWrite(4, HIGH);
      digitalWrite(7, LOW);
      smer1 = smer;
    }
    else if (smer == 0) {
      digitalWrite(4, LOW);
      digitalWrite(7, LOW);
    }
    else {
      digitalWrite(4, LOW);
      digitalWrite(7, HIGH);
      smer1 = smer;
    }
    analogWrite(5, rychlost);
  }
  else {
    if (smer < 0) {
      digitalWrite(8, LOW);
      digitalWrite(9, HIGH);
      smer4 = smer;
    }
    else if (smer == 0) {
      digitalWrite(8, LOW);
      digitalWrite(9, LOW);
    }
    else {
      digitalWrite(8, HIGH);
      digitalWrite(9, LOW);
      smer4 = smer;
    }

    analogWrite(6, rychlost);

  }
}
void setup() {
  for (int pin = 4; pin <= 11; pin++) {
    pinMode(pin, OUTPUT);
  }
  pinMode(Napetie, INPUT);
  pinMode(Prud, INPUT);
  pinMode(LZelena, OUTPUT);
  pinMode(LCervena, OUTPUT);
  pinMode(LModra, OUTPUT);
  digitalWrite(LModra, LOW);
  digitalWrite(LCervena, LOW);
  digitalWrite(LZelena, LOW);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), otackomerMotor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), otackomerMotor4, CHANGE);
  napajanie12V(false);
  motor(1, 0, 255);
  motor(4, 0, 255);
  Wire.begin(8);                // nastavenie komunikacnej adresy na 0x08
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  if(serialPortStatus)  Serial.begin(9600);
  if(watchdogStatus) wdt_enable(WDTO_1S);
}

void otackomerMotor1() {
  if (smer1 > 0) pocetTikov1++;
  else if(smer1 < 0) pocetTikov1--;
}

void otackomerMotor4() {
  if (smer4 > 0) pocetTikov4++;
  else if (smer4 < 0) pocetTikov4--;
}

void loop() {
  digitalWrite(LModra, LOW);
    lastRychlost1 = pocetTikov1;
    lastRychlost4 = pocetTikov4;
    lastVzdialenost1 += pocetTikov1;
    lastVzdialenost4 += pocetTikov4;
    lastNapetie = analogRead(Napetie);
    lastPrud = analogRead(Prud);
//Serial.print(lastNapetie,DEC);
//Serial.print('\n');
  pocetTikov1 = 0;
  pocetTikov4 = 0;
  delay(100);
  if(watchdogStatus) wdt_reset();
}

void receiveEvent(int howMany) {
  char d = 0;
  digitalWrite(LModra, HIGH);
  while (2 < Wire.available()) {
    Wire.read();
  }
  c = Wire.read();
  d = Wire.read();
  switch (c) {
    case 100:
      lastVzdialenost1 = 0;
      break;
    case 99:
      lastVzdialenost4 = 0;
      break;
    case 98:
      lastVzdialenost1 = 0;
      lastVzdialenost4 = 0;
      break;
    case 97:
      if (d == 0)       digitalWrite(LZelena, LOW);
      else if (d == 1)  digitalWrite(LZelena, HIGH);
      break;
    case 96:
      if (d == 0)       digitalWrite(LCervena, LOW);
      else if (d == 1)  digitalWrite(LCervena, HIGH);
      break;
    case 95:
      if (d == 0)       napajanie12V(false);
      else if (d == 1)  napajanie12V(true);
      break;
    case 94:
      onReg1 = true;
      setRychlost1 = d;
      break;
    case 93:
      onReg1 = true;
      setRychlost1 = -d;
      break;
    case 92:
      onReg1 = false;
      motor(1,1,d);
      break;
    case 91:
      onReg1 = false;
      motor(1,0,d);
      break;
    case 90:
      onReg1 = false;
      motor(1,-1,d);
      break;
    case 89:
      onReg4 = true;
      setRychlost4 = d;
      break;
    case 88:
      onReg4 = true;
      setRychlost4 = -d;
      break;
    case 87:
      onReg4 = false;
      motor(4,1,d);
      break;
    case 86:
      onReg4 = false;
      motor(4,0,d);
      break;
    case 85:
      onReg4 = false;
      motor(4,-1,d);
      break;
  }
}
void requestEvent() {
  byte data[2];
  switch (c) {
    case 1:
      data[1] = (lastRychlost1 & 0xFF);
      data[0] = (lastRychlost1 >> 8) & 0xFF;
      Wire.write(data, 2);
      break;
    case 2:
      data[1] = (lastRychlost4 & 0xFF);
      data[0] = (lastRychlost4 >> 8) & 0xFF;
      Wire.write(data, 2);
      break;
    case 3:
      data[1] = (lastVzdialenost1 & 0xFF);
      data[0] = (lastVzdialenost1 >> 8) & 0xFF;
      Wire.write(data, 2);
      break;
    case 4:
      data[1] = (lastVzdialenost4 & 0xFF);
      data[0] = (lastVzdialenost4 >> 8) & 0xFF;
      Wire.write(data, 2);
      break;
    case 5:
      data[1] = (lastNapetie & 0xFF);
      data[0] = (lastNapetie >> 8) & 0xFF;
      Wire.write(data, 2);
      break;
    case 6:
      data[1] = (lastPrud & 0xFF);
      data[0] = (lastPrud >> 8) & 0xFF;
      Wire.write(data, 2);
      break;
  }
  c = -1;
}
