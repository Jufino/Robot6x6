#include <Wire.h>
#include <Servo.h>
#include <avr/wdt.h>
volatile boolean stopUprava = false;
volatile unsigned int pocetTikov2 = 0;
volatile unsigned int pocetTikov3 = 0;
volatile int c = -1;
int lastRychlost2 = 0;
int lastRychlost3 = 0;
int lastVzdialenost2 = 0;
int lastVzdialenost3 = 0;
char smer2 = 1;
char smer3 = 1;
long duration;
int distance;
int poziciaServo;
volatile boolean onReg2 = true;
volatile boolean onReg3 = true;
volatile int setRychlost2=0;
volatile int setRychlost3=0;

//pin 2 - // otackomer motor 3
//pin 3 - // otackomer motor 2
//LED A3 - cervena
//LED A2 - zelena
//LED 13 - modra
#define LCervena A2
#define LZelena A3
#define LModra 13
#define trigPin A0
#define echoPin A1
#define servoPin 10
#define watchdogStatus 1
#define serialPortStatus 0

void motor(int motor, char smer, int rychlost) {
    Serial.print(floatTempC,DEC);
  if (motor == 3) {
    if (smer < 0) {
      digitalWrite(4, HIGH);
      digitalWrite(7, LOW);
      smer3 = smer;
    }
    else if (smer == 0) {
      digitalWrite(4, LOW);
      digitalWrite(7, LOW);
    }
    else {
      digitalWrite(4, LOW);
      digitalWrite(7, HIGH);
      smer3 = smer;
    }
    analogWrite(5, rychlost);
  }
  else {
    if (smer < 0) {
      digitalWrite(8, LOW);
      digitalWrite(9, HIGH);
      smer2 = smer;
    }
    else if (smer == 0) {
      digitalWrite(8, LOW);
      digitalWrite(9, LOW);
    }
    else {
      digitalWrite(8, HIGH);
      digitalWrite(9, LOW);
      smer2 = smer;
    }
    analogWrite(6, rychlost);
  }
}
Servo servo1;
void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
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
  attachInterrupt(digitalPinToInterrupt(3), otackomerMotor3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), otackomerMotor2, CHANGE);
  motor(2, 0, 255);
  motor(3, 0, 255);
  Wire.begin(10);            
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  servo1.attach(servoPin);
  servo1.write(90);
  if(serialPortStatus)  Serial.begin(9600);
  if(watchdogStatus) wdt_enable(WDTO_500MS);
}

void otackomerMotor3() {
  if (smer3 > 0) pocetTikov3++;
  else if(smer3 < 0) pocetTikov3--;
}

void otackomerMotor2() {
  if (smer2 > 0) pocetTikov2++;
  else if (smer2 < 0) pocetTikov2--;
}
int v=10;
void loop() {
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  while (stopUprava == true);
  stopUprava = true;
    digitalWrite(LModra, LOW);
    distance = (duration/2) / 29.1;
    lastRychlost2 = pocetTikov2;
    lastRychlost3 = pocetTikov3;
    lastVzdialenost2 += pocetTikov2;
    lastVzdialenost3 += pocetTikov3;
  stopUprava = false;
  pocetTikov2 = 0;
  pocetTikov3 = 0;
  delay(88);
  if(watchdogStatus) wdt_reset();
}

void receiveEvent(int howMany) {
  int d = 0;
  digitalWrite(LModra, HIGH);
  while (2 < Wire.available()) {
    Wire.read();
  }
  c = Wire.read();
  d = Wire.read();
  if(c != -1){
          Serial.print("d: ");
          Serial.print(c);
        Serial.print("\nc: ");
                  Serial.print(d);
        Serial.print("\n");
  }
  switch (c) {
    case 100:
      while (stopUprava == true);
      stopUprava = true;
      lastVzdialenost3 = 0;
      stopUprava = false;
      break;
    case 99:
      while (stopUprava == true);
      stopUprava = true;
      lastVzdialenost2 = 0;
      stopUprava = false;
      break;
    case 98:
      while (stopUprava == true);
      stopUprava = true;
      lastVzdialenost2 = 0;
      lastVzdialenost3 = 0;
      stopUprava = false;
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
      onReg3 = true;
      setRychlost3 = d;
      break;
    case 93:
      onReg3 = true;
      setRychlost3 = -d;
      break;
    case 92:
      onReg3 = false;
      motor(3,1,d);
      break;
    case 91:
      onReg3 = false;
      motor(3,0,d);
      break;
    case 90:
      onReg3 = false;
      motor(3,-1,d);
      break;
    case 89:
      onReg2 = true;
      setRychlost2 = d;
      break;
    case 88:
      onReg2 = true;
      setRychlost2 = -d;
      break;
    case 87:
      onReg2 = false;
      motor(2,1,d);
      break;
    case 86:
      onReg2 = false;
      motor(2,0,d);
      break;
    case 85:
      onReg2 = false;
      motor(2,-1,d);
      break;
    case 84:
      if(d <= 181 && d >= 1){
        poziciaServo=d;
        servo1.write(poziciaServo);
        Serial.print(poziciaServo);
        Serial.print("\n");
      }
      break;
  }
  
}
void requestEvent() {
  byte data[2];
  switch (c) {
    case 1:
      while (stopUprava == true);
      stopUprava = true;
      data[1] = (lastRychlost3 & 0xFF);
      data[0] = (lastRychlost3 >> 8) & 0xFF;
      stopUprava = false;
      Wire.write(data, 2);
      break;
    case 2:
      while (stopUprava == true);
      stopUprava = true;
      data[1] = (lastRychlost2 & 0xFF);
      data[0] = (lastRychlost2 >> 8) & 0xFF;
      stopUprava = false;
      Wire.write(data, 2);
      break;
    case 3:
      while (stopUprava == true);
      stopUprava = true;
      data[1] = (lastVzdialenost3 & 0xFF);
      data[0] = (lastVzdialenost3 >> 8) & 0xFF;
      stopUprava = false;
      Wire.write(data, 2);
      break;
    case 4:
      while (stopUprava == true);
      stopUprava = true;
      data[1] = (lastVzdialenost2 & 0xFF);
      data[0] = (lastVzdialenost2 >> 8) & 0xFF;
      stopUprava = false;
      Wire.write(data, 2);
      break;
    case 5:
      while (stopUprava == true);
      stopUprava = true;
      data[0] = (poziciaServo & 0xFF);
      stopUprava = false;
      Wire.write(data, 1);
      break;
    case 6:
      while (stopUprava == true);
      stopUprava = true;
      data[1] = (distance & 0xFF);
      data[0] = (distance >> 8) & 0xFF;
      stopUprava = false;
      Wire.write(data, 2);
      break;
  }
  c = -1;
}