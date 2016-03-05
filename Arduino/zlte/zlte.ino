#include <Wire.h>
#include <avr/wdt.h>
volatile unsigned int pocetTikov5 = 0;
volatile unsigned int pocetTikov6 = 0;
volatile char c = -1;
volatile int lastRychlost5 = 0;
volatile int lastRychlost6 = 0;
volatile int lastVzdialenost5 = 0;
volatile int lastVzdialenost6 = 0;
char smer5 = 1;
char smer6 = 1;
boolean onReg5 = true;
boolean onReg6 = true;
volatile int setRychlost5 = 0;
volatile int setRychlost6 = 0;

//pin 2 - // otackomer motor 6
//pin 3 - // otackomer motor 5
//LED A3 - cervena
//LED A2 - zelena
//LED 13 - modra
#define LCervena A2
#define LZelena A3
#define LModra 13
#define watchdogStatus 1
#define serialPortStatus 0
#define test 300

void motor(int motor, char smer, int rychlost) {
  if (motor == 6) {
    if (smer < 0) {
      digitalWrite(4, HIGH);
      digitalWrite(7, LOW);
      smer6 = smer;
    }
    else if (smer == 0) {
      digitalWrite(4, LOW);
      digitalWrite(7, LOW);
    }
    else {
      digitalWrite(4, LOW);
      digitalWrite(7, HIGH);
      smer6 = smer;
    }
    analogWrite(5, rychlost);
  }
  else {
    if (smer < 0) {
      digitalWrite(8, LOW);
      digitalWrite(9, HIGH);
      smer5 = smer;
    }
    else if (smer == 0) {
      digitalWrite(8, LOW);
      digitalWrite(9, LOW);
    }
    else {
      digitalWrite(8, HIGH);
      digitalWrite(9, LOW);
      smer5 = smer;
    }

    analogWrite(6, rychlost);

  }
}
void setup() {
  for (int pin = 4; pin <= 11; pin++) {
    pinMode(pin, OUTPUT);
  }
  pinMode(A3, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(LModra, LOW);
  digitalWrite(LCervena, LOW);
  digitalWrite(LZelena, LOW);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), otackomerMotor6, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), otackomerMotor5, CHANGE);
  motor(5, 0, 255);
  motor(6, 0, 255);
  Wire.begin(9);                // nastavenie komunikacnej adresy na 0x09
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  if (serialPortStatus)  Serial.begin(9600);
  if (watchdogStatus) wdt_enable(WDTO_1S);
}

void otackomerMotor6() {
  if (smer6 > 0){ 
    pocetTikov6++;
    lastVzdialenost6++;
  }
  else if (smer6 < 0){
    pocetTikov6--;
    lastVzdialenost6--;
  }
}

void otackomerMotor5() {
  if (smer5 > 0){
    pocetTikov5++;
    lastVzdialenost5++;
  }
  else if (smer5 < 0){
    pocetTikov5--;
    lastVzdialenost5--;
  }
}

void loop() {
  digitalWrite(LModra, LOW);
  lastRychlost5 = pocetTikov5;
  lastRychlost6 = pocetTikov6;
  pocetTikov5 = 0;
  pocetTikov6 = 0;
  delay(100);
  if (watchdogStatus) wdt_reset();
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
      lastVzdialenost6 = 0;
      break;
    case 99:
      lastVzdialenost5 = 0;
      break;
    case 98:
      lastVzdialenost5 = 0;
      lastVzdialenost6 = 0;
      break;
    case 97:
      if (d == 0)       digitalWrite(LZelena, LOW);
      else if (d == 1)  digitalWrite(LZelena, HIGH);
      break;
    case 96:
      if (d == 0)       digitalWrite(LCervena, LOW);
      else if (d == 1)  digitalWrite(LCervena, HIGH);
      break;
    case 94:
      onReg6 = true;
      setRychlost6 = d;
      break;
    case 93:
      onReg6 = true;
      setRychlost6 = -d;
      break;
    case 92:
      onReg6 = false;
      motor(6, 1, d);
      break;
    case 91:
      onReg6 = false;
      motor(6, 0, d);
      break;
    case 90:
      onReg6 = false;
      motor(6, -1, d);
      break;
    case 89:
      onReg5 = true;
      setRychlost5 = d;
      break;
    case 88:
      onReg5 = true;
      setRychlost5 = -d;
      break;
    case 87:
      onReg5 = false;
      motor(5, 1, d);
      break;
    case 86:
      onReg5 = false;
      motor(5, 0, d);
      break;
    case 85:
      onReg5 = false;
      motor(5, -1, d);
      break;
  }
}
void requestEvent() {
  byte data[2];
  switch (c) {
    case 1:
      data[1] = (lastRychlost6 & 0xFF);
      data[0] = (lastRychlost6 >> 8) & 0xFF;
      Wire.write(data, 2);
      break;
    case 2:
      data[1] = (lastRychlost5 & 0xFF);
      data[0] = (lastRychlost5 >> 8) & 0xFF;
      Wire.write(data, 2);
      break;
    case 3:
      data[1] = (lastVzdialenost6 & 0xFF);
      data[0] = (lastVzdialenost6 >> 8) & 0xFF;
      Wire.write(data, 2);
      break;
    case 4:
      data[1] = (lastVzdialenost5 & 0xFF);
      data[0] = (lastVzdialenost5 >> 8) & 0xFF;
      Wire.write(data, 2);
      break;
    case 5:
      data[1] = (lastVzdialenost6 & 0xFF);
      data[0] = (lastVzdialenost6 >> 8) & 0xFF;
      Wire.write(data, 2);
      lastVzdialenost6 = 0;
      break;
    case 6:
      data[1] = (lastVzdialenost5 & 0xFF);
      data[0] = (lastVzdialenost5 >> 8) & 0xFF;
      Wire.write(data, 2);
      lastVzdialenost5  = 0;
      break;
    case 127:
      data[1] = (test & 0xFF);
      data[0] = (test >> 8) & 0xFF;
      Wire.write(data, 2);
      break;
  }
  c = -1;
}
