#include <Arduino.h>
#include <Servo.h>

#define DECODE_NEC

#define IR_USE_AVR_TIMER4
#include <IRremote.hpp>

#include "Wheels.h"
#include "LiquidCrystal_I2C.h"
#include "PinChangeInterrupt.h"

#define INTINPUT0 A0
#define INTINPUT1 A1

#define TRIG A2
#define ECHO A3

#define IR_RECEIVE_PIN 2

#define SERVO 3
Servo serwo;

volatile int cnt0, cnt1;
volatile int counter;

byte LCDAddress = 0x27;

LiquidCrystal_I2C lcd(LCDAddress, 16, 2);


#define SET_MOVEMENT(side,f,b) digitalWrite( side[0], f);\
  digitalWrite( side[1], b)




Wheels::Wheels()
{

}

unsigned int tellDistance() {

  unsigned long tot;      // czas powrotu (time-of-travel)
  unsigned int distance;

  digitalWrite(TRIG, HIGH);
  delay(10);
  digitalWrite(TRIG, LOW);
  tot = pulseIn(ECHO, HIGH);
  /* prędkość dźwięku = 340m/s => 1 cm w 29 mikrosekund
     droga tam i z powrotem, zatem:
  */
  distance = tot / 58;
  return distance;
}
bool scan() {
  lcd.clear();
  serwo.write(30);
  unsigned int right = tellDistance();
  lcd.print(right);
  delay(1000);
  lcd.clear();
  serwo.write(150);
  unsigned int left = tellDistance();
  lcd.print(left);
  delay(1000);
  serwo.write(90);
  if (right > left) {
    return true;
  }
  return false;
}

void increment() {
  if (digitalRead(INTINPUT0))
    cnt0++;
  else if (digitalRead(INTINPUT1))
    cnt1++;
}

void Wheels::attachRight(int pF, int pB, int pS)
{
  pinMode(pF, OUTPUT);
  pinMode(pB, OUTPUT);
  pinMode(pS, OUTPUT);
  this->pinsRight[0] = pF;
  this->pinsRight[1] = pB;
  this->pinsRight[2] = pS;
}


void Wheels::attachLeft(int pF, int pB, int pS)
{
  pinMode(pF, OUTPUT);
  pinMode(pB, OUTPUT);
  pinMode(pS, OUTPUT);
  this->pinsLeft[0] = pF;
  this->pinsLeft[1] = pB;
  this->pinsLeft[2] = pS;
}

void Wheels::setSpeedRight(uint8_t s)
{
  analogWrite(this->pinsRight[2], s);
}

void Wheels::setSpeedLeft(uint8_t s)
{
  analogWrite(this->pinsLeft[2], s);
}

void Wheels::setSpeed(uint8_t s)
{
  setSpeedLeft(s);
  setSpeedRight(s);
}

void Wheels::attach(int pRF, int pRB, int pRS, int pLF, int pLB, int pLS)
{
  // lcd setup
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("H3llo, w0rld!");
  // wheel sensors
  pinMode(INTINPUT0, INPUT);
  pinMode(INTINPUT1, INPUT);
  cnt0 = 0;
  cnt1 = 0;

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  attachPCINT(digitalPinToPCINT(INTINPUT0), increment, CHANGE);
  attachPCINT(digitalPinToPCINT(INTINPUT1), increment, CHANGE);

  pinMode(TRIG, OUTPUT);    // TRIG startuje sonar
  pinMode(ECHO, INPUT);

  serwo.attach(SERVO);

  this->attachRight(pRF, pRB, pRS);
  this->attachLeft(pLF, pLB, pLS);
}

void Wheels::forwardLeft()
{
  SET_MOVEMENT(pinsLeft, HIGH, LOW);
}

void Wheels::forwardRight()
{
  SET_MOVEMENT(pinsRight, HIGH, LOW);
}

void Wheels::backLeft()
{
  SET_MOVEMENT(pinsLeft, LOW, HIGH);
}

void Wheels::backRight()
{
  SET_MOVEMENT(pinsRight, LOW, HIGH);
}

void Wheels::forward()
{
  this->forwardLeft();
  this->forwardRight();
}

void Wheels::back()
{
  this->backLeft();
  this->backRight();

}

void Wheels::stopLeft()
{
  SET_MOVEMENT(pinsLeft, LOW, LOW);
}

void Wheels::stopRight()
{
  SET_MOVEMENT(pinsRight, LOW, LOW);
}

void Wheels::stop()
{
  this->stopLeft();
  this->stopRight();
}

void Wheels::goForward(int cm) {
  this->setSpeedRight(100);
  this->setSpeedLeft(115);
  float count = (21.0 / 40.0) * cm;
  lcd.clear();
  cnt0 = 0;
  cnt1 = 0;
  this->forward();
  while (cnt0 < count && cnt1 < count ) {
  }
  this->stop();
  lcd.clear();
  delay(200);
}



void Wheels::goBack(int cm) {
  this->setSpeedRight(100);
  this->setSpeedLeft(115);
  lcd.clear();
  cnt0 = 0;
  cnt1 = 0;
  this->back();
  counter = 0;
  while (counter < cm / 21 ) {
    if (cnt0 > 40) {
      cnt0 = 0;
      counter++;
      lcd.clear();
      lcd.print(counter);
    }
  }
  this->stop();
  lcd.clear();
  delay(200);
}

void Wheels::turn(int deg) {
  int check = (26 * abs(deg)) / 90;
  this->setSpeedRight(100);
  this->setSpeedLeft(115);

  if (deg > 0) {
    this->forwardRight();
    this->backLeft();
  } else {
    this->forwardLeft();
    this->backRight();
  }

  cnt0 = 0;
  cnt1 = 0;
  while (cnt0 < check && cnt1 < check) {
  }
  this->stop();
  lcd.clear();
  lcd.print(cnt0);
  lcd.print(" | ");
  lcd.print(cnt1);
}

void Wheels::IDoWhatIAmTold() {
  bool moveing = false;
  bool near_wall = false;
  bool avoid = true;
  bool printed = false;
  bool turn_right = false;
  bool scaned = false;
  uint16_t msg = 0;
  this->setSpeedRight(100);
  this->setSpeedLeft(120);
  lcd.clear();
  moveing = true;
  this->forward();
  while(true) {
    near_wall = tellDistance() < 20;
    if (IrReceiver.decode()) {
        lcd.clear();
        msg = IrReceiver.decodedIRData.command;
        IrReceiver.resume();
        lcd.print(msg);
    }
    if (near_wall && msg == 28) {
      this->stop();
      moveing = false;
    }
    if (!near_wall && !moveing) {
      printed = false;
      this->forward();
      moveing = true;
    }
    if (near_wall && msg == 24) {
      this -> stop();
      moveing = false;
      turn_right = scan();
        if (turn_right) {
    this->turn(70);
    serwo.write(180);
  } else {
    this->turn(-70);
    serwo.write(0);
  }

  lcd.clear();
  lcd.print("forward and look to side");
  delay(1000);
  this->forward();
  bool d = 0;
  while (near_wall) {
    d = tellDistance();
    lcd.print(d);
    if (d > 40) {
      near_wall = false;
    }
    delay(50);
    lcd.clear();
  }
  delay(500);
 
  this -> stop();
  if (turn_right) {
    this->turn(-70);
    serwo.write(90);
  } else {
    this->turn(70);
    serwo.write(90);
  }
    }
  }
}

void Wheels::goAndStop() {
  this->setSpeedRight(200);
  this->setSpeedLeft(200);
  lcd.clear();
  this->forward();
  bool near_wall = false;
  unsigned int d = 2000;
  while (!near_wall) {
    d = tellDistance();
    lcd.print(d);
    if (d < 20) {
      near_wall = true;
    }
    delay(50);
    lcd.clear();
  }
  this -> stop();
  bool turn_right = scan();
  lcd.clear();
  lcd.print("turn and look");

  if (turn_right) {
    this->turn(70);
    serwo.write(180);
  } else {
    this->turn(-70);
    serwo.write(0);
  }

  d = 0;
  lcd.clear();
  lcd.print("forward and look to side");
  delay(1000);
  this->forward();
  while (near_wall) {
    d = tellDistance();
    lcd.print(d);
    if (d > 40) {
      near_wall = false;
    }
    delay(50);
    lcd.clear();
  }
  delay(500);
 
  this -> stop();
  if (turn_right) {
    this->turn(-70);
    serwo.write(90);
  } else {
    this->turn(70);
    serwo.write(90);
  }
}
