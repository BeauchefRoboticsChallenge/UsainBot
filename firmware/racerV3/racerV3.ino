/* Sketch UsainBot v2.0

   by Gonzalo Olave based on v1.0 by Matias Mattamala and G.O.
   v1.0 -> https://github.com/mmattamala/racer/blob/master/arduino/robot_brc/robot_brc.ino
   last update 26/4/2022

   This code is in the public domain and it is licensed under the BEER-JUICE-WARE licence

   ------------------------------------------------------------
   "THE BEERWARE LICENSE" (Revision 42):
   The authors wrote this code. As long as you retain this
   notice, you can do whatever you want with this stuff. If we
   meet someday, and you think this stuff is worth it, you can
   buy us a beer in return (or a juice).
   ------------------------------------------------------------
*/

// debug
//#define DEBUG_SENSORS

#define CALIBRATE
// serial debug
#define DEBUG

#define LINE_WHITE

#include "DebugUtils.h"
#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>

// Wheel pins
// Left wheel (LW)
#define PIN_LW_PWM 6
#define PIN_LW_A 13
#define PIN_LW_B 10

// Right wheel (RW)
#define PIN_RW_PWM 5
#define PIN_RW_A 11
#define PIN_RW_B 12

// Right sensor
#define PIN_RS A1
// Left sensor
#define PIN_LS A0

#define LED_PIN 9
#define BUTTON_PIN 7

// General definitions
#define LW 0 // left wheel
#define RW 1 // right wheel

#define TIMEOUT 2500

// speed
#define VEL_MIN 0
#define VEL_MAX 255
#define VEL_CURVA 180
#define MIN_ERR_THR 250


int r_sensor = 0;
int l_sensor = 0;
int st_sensor = 0;

float KP = 0.5; // 0.3
float KI = 0.0;
float KD = 12;

float prop_err, int_err, der_err, last_err;
float pid_out;

bool stdby_state = 0;

uint8_t lw_speed;
uint8_t rw_speed;

int th = 800;
float sum1 = 0;
float sum = 0;
float err = 0;
int s_min = TIMEOUT;
int s_max = 0;
float pos = 0;

int counter = 0;

uint16_t sensorSide[2];
uint16_t sensorLine[6];

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(2, LED_PIN, NEO_GRB + NEO_KHZ800);
QTRSensors qtr_line, qtr_side;

uint32_t BLACK = pixel.Color(0, 0, 0);
uint32_t RED = pixel.Color(255, 0, 0); // rojo
uint32_t MAGENTA = pixel.Color(255, 0, 200); // magenta
uint32_t GREEN = pixel.Color(0, 255, 0); // magenta
uint32_t YELLOW = pixel.Color(255, 140, 0); // magenta

//-------------------------------
// Setup functions
//-------------------------------
void setupWheels() {

  // configure left wheel
  pinMode(PIN_LW_PWM, OUTPUT);
  pinMode(PIN_LW_A, OUTPUT);
  pinMode(PIN_LW_B, OUTPUT);

  // configure right wheel
  pinMode(PIN_RW_PWM, OUTPUT);
  pinMode(PIN_RW_A, OUTPUT);
  pinMode(PIN_RW_B, OUTPUT);

  onWheels();
}

void setupHW() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void setupLeds() {
  pixel.begin();
}

void setupSensors() {
  qtr_side.setTypeRC();
  qtr_side.setSensorPins((const uint8_t[]) {
    PIN_LS, PIN_RS
  }, 2);
  qtr_line.setTypeAnalog();
  qtr_line.setSensorPins((const uint8_t[]) {
    A2, A3, A4, A5, A6, A7
  }, 6);

#ifdef CALIBRATE
  pixel.setPixelColor(0, RED);
  pixel.show();
  Serial.println("Calibrando...");
  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    qtr_line.calibrate();
    qtr_side.calibrate();
  }
  pixel.setPixelColor(0, BLACK);
  pixel.show();
#endif
}

void setup() {

  Serial.begin(115200); // for BT communication and debug

  setupWheels();
  setupLeds();
  setupHW();
  setupSensors();

  pixel.setBrightness(100);
  pixel.setPixelColor(0, YELLOW);
  pixel.show();
  delay(500);

  while(true){
    int boton = digitalRead(BUTTON_PIN);
    if(boton == 0) break;
  }
  delay(1000);
}

void loop() {
  //testBlink();

  onWheels();
  //testWheel(RW);
  //delay(1000);
  //testWheel(LW);
  //delay(1000);
  //handBrake();

#ifdef DEBUG_SENSORS
  readRawSensors();
  delay(10);
#endif
  
  lineFollowing();

  pixel.setPixelColor(0, GREEN);
  pixel.show();

  pixel.setPixelColor(1, MAGENTA);
  pixel.show();

}

//-------------------------------
// Line following functions
//-------------------------------

void lineFollowing() {
  // Step 1: Read Raw Sensors
  readRawSensors();
  // Step 2: Compute error
  computeError();
  // Step 3: Compute PID correction
  computePID();
  // Step 4: Map error correction to motor speeds
  computeMotorSpeeds();
  // Step 5: Apply speeds
  setMotorSpeeds();
}

void readRawSensors() {
#ifdef DEBUG_SENSORS
  qtr_line.readCalibrated(sensorLine);
  qtr_side.readCalibrated(sensorSide);
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(sensorLine[i]);
    Serial.print('\t');
  }
  for (uint8_t i = 0; i < 2; i++) {
    Serial.print(sensorSide[i]);
    Serial.print('\t');
  }
  Serial.println();

#ifdef LINE_WHITE
  pos = qtr_line.readLineWhite(sensorLine);
#else
  pos = qtr_line.readLineBlack(sensorLine);
#endif

  //DEBUG_PRINTLN(pos);
  l_sensor = sensorSide[0];
  r_sensor = sensorSide[1];

#else
  qtr_line.readCalibrated(sensorLine);
  qtr_side.readCalibrated(sensorSide);
#ifdef LINE_WHITE
  pos = qtr_line.readLineWhite(sensorLine);
#else
  pos = qtr_line.readLineBlack(sensorLine);
#endif

  l_sensor = sensorSide[0];
  r_sensor = sensorSide[1];
  DEBUG_PRINTLN(pos);
  //position = map(position, 0, 5000, -255, 255);
  //delay(500);
#endif

}

void computeError() {
  prop_err = pos - TIMEOUT;
  der_err = prop_err - last_err;
  int_err += prop_err;

  last_err = prop_err;
  sum1 = 0;
  sum = 0;

}

void computePID() {
  pid_out = prop_err * KP + int_err * KI + der_err * KD;

  //DEBUG_PRINTLN(pid_out);

}

void computeMotorSpeeds() {

  if ( pid_out > MIN_ERR_THR ) {
    lw_speed = map(pid_out, -2500, 2500, VEL_MIN, VEL_CURVA);
    rw_speed = VEL_MIN; //TODO: apagar motor
    //handBreakRight(); // creo que esto hace que se muera el robot cuando salga de la línea
  }
  else if ( pid_out < -MIN_ERR_THR ) {
    lw_speed = VEL_MIN;
    rw_speed = map(pid_out, -2500, 2500, VEL_CURVA, VEL_MIN);
    //handBreakLeft();
  }
  else {
    //onWheels();
    // TODO: arreglar este mapeo:
    lw_speed = map(pid_out, -2500, 2500, VEL_MAX, VEL_MIN);
    rw_speed = map(pid_out, -2500, 2500, VEL_MAX, VEL_MIN);
  }

  //DEBUG_PRINTLN("LW: " + String(lw_speed) + "\tRW: " + String(rw_speed));
}

void setMotorSpeeds() {
  analogWrite(PIN_LW_PWM, lw_speed);
  analogWrite(PIN_RW_PWM, rw_speed);

}

void handBrakeLeft() {
  digitalWrite(PIN_LW_A, HIGH);
  digitalWrite(PIN_LW_B, HIGH);
}

void handBrakeRight() {
  digitalWrite(PIN_RW_A, HIGH);
  digitalWrite(PIN_RW_B, HIGH);
}

void handBrake() {
  handBrakeLeft();
  handBrakeRight();
}

void onWheels() {
  // this depends on motor connections to driver
  digitalWrite(PIN_LW_A, LOW);
  digitalWrite(PIN_LW_B, HIGH);

  digitalWrite(PIN_RW_A, LOW);
  digitalWrite(PIN_RW_B, HIGH);
}


//-------------------------------
// Test functions
//-------------------------------
void testBlink() {
  /*
    uint32_t color = pixel.Color(255,0,255); // magenta
    pixel.setPixelColor(0,color);
    pixel.show();
  */
  PlasmaPulse(30);

}

void testWheel(int wheel) {

  int t = 20;
  if (wheel == RW) {
    for (int i = 0; i < 255; i += 5) {
      analogWrite(PIN_RW_PWM, i);
      Serial.println("Testing " + String(wheel == LW ? "left " : "right") + " wheel, power = " + String(i));
      delay(t);
    }
    for (int i = 255; i > 0; i -= 5) {
      analogWrite(PIN_RW_PWM, i);
      Serial.println("Testing " + String(wheel == LW ? "left " : "right") + " wheel, power = " + String(i));
      delay(t);
    }
  }
  else if (wheel == LW) {
    for (int i = 0; i < 255; i += 5) {
      analogWrite(PIN_LW_PWM, i);
      Serial.println("Testing " + String(wheel == LW ? "left " : "right") + " wheel, power = " + String(i));
      delay(t);
    }
    for (int i = 255; i > 0; i -= 5) {
      analogWrite(PIN_LW_PWM, i);
      Serial.println("Testing " + String(wheel == LW ? "left " : "right") + " wheel, power = " + String(i));
      delay(t);
    }
  }
}

void testBrake() {

  digitalWrite(PIN_RW_A, HIGH);
  digitalWrite(PIN_RW_B, LOW);
  digitalWrite(PIN_LW_A, HIGH);
  digitalWrite(PIN_LW_B, LOW);
  analogWrite(PIN_RW_PWM, 100);
  analogWrite(PIN_LW_PWM, 100);

  delay(2000);
  handBrake();

  delay(1000);
}

void PlasmaPulse(uint8_t wait) {
  uint16_t i, j;
  uint8_t brightness = 10;

  for (i = 0; i < pixel.numPixels(); i++) {
    pixel.setPixelColor(i, pixel.Color(255, 255, 255));
  }
  pixel.show();
  delay(wait);
  //Adjust 60 and 90 to the starting and ending colors you want to fade between.
  for (j = 170; j >= 135; --j) {
    for (i = 0; i < pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel((i + j) & 255));
    }
    pixel.show();
    brightness -= 6;
    pixel.setBrightness(brightness);
    delay(wait);
  }

  for (j = 135; j < 1170; j++) {
    for (i = 0; i < pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel((i + j) & 255));
    }
    pixel.show();
    brightness += 6;
    pixel.setBrightness(brightness);
    delay(wait);
  }

  for (i = 0; i < pixel.numPixels(); i++) {
    pixel.setPixelColor(i, pixel.Color(255, 255, 255));
  }
  pixel.show();
  delay(wait);

}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if (WheelPos < 85) {
    return pixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return pixel.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return pixel.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
