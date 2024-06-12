#include "Wheels.h"
//#include "TimerOne.h"


Wheels w;

bool moveing = false;
bool near_wall = false;
bool avoid_mode = false;

void setup() {
  // put your setup code here, to run once:
  // w.attach(8,4,10,7,5,6);
  w.attach(7, 8, 6, 10, 9, 11);
}

void loop() {
  w.IDoWhatIAmTold();
}
