
// pinout del robot
#include "BRCito_defines.h"

// General definitions
#define LW 0 // left wheel
#define RW 1 // right wheel

// Estos valores definen las velocidades de las ruedas
#define VEL_MIN 0
#define VEL_MAX 250//130
#define VEL_CURVA 230 // 110
#define MIN_ERR_THR 150 // mayor error 

long LW_counter=0,RW_counter=0;

void setup() {

  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(RW_C1), RW_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LW_C1), LW_ISR, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("LW: ");
  Serial.print(LW_counter);
  Serial.print("\tRW: ");
  Serial.println(RW_counter);
  
  delay(10);
}

void LW_ISR(){
  int8_t lwc2 = digitalRead(LW_C2);
  LW_counter += lwc2 ? 1 : -1;
}

void RW_ISR(){
  int8_t rwc2 = digitalRead(RW_C2);
  RW_counter += rwc2 ? -1 : 1;
}
