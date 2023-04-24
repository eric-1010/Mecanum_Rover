/*This Sketch is a modified version from DroneBotWorkshop.
   https://dronebotworkshop.com/mecanum/
   https://youtu.be/dZttHOxIoek
*/

/*  warning read the docs about max freq per resolution selected
    Per this forum post: https://esp32.com/viewtopic.php?t=6701
    The way to calculate max PWM frequency is  integer (log 2 (LEDC_APB_CLK / frequency))

    With a LEDC_APB_CLK == 80MHz, these are the following maximum values, in Hz:

    LEDC_TIMER_1_BIT, 40000000
    LEDC_TIMER_2_BIT, 20000000
    LEDC_TIMER_3_BIT, 10000000
    LEDC_TIMER_4_BIT,  5000000
    LEDC_TIMER_5_BIT,  2500000
    LEDC_TIMER_6_BIT,  1250000
    LEDC_TIMER_7_BIT,   625000
    LEDC_TIMER_8_BIT,   312500
    LEDC_TIMER_9_BIT,   156250
    LEDC_TIMER_10_BIT,   78125
    LEDC_TIMER_11_BIT,   39062
    LEDC_TIMER_12_BIT,   19531
    LEDC_TIMER_13_BIT,    9765
    LEDC_TIMER_14_BIT,    4882
    LEDC_TIMER_15_BIT,    2441
    LEDC_TIMER_16_BIT,    1220
    LEDC_TIMER_17_BIT,     610
    LEDC_TIMER_18_BIT,     305
    LEDC_TIMER_19_BIT,     152
    LEDC_TIMER_20_BIT,      76
*/

/*
   Sketch uses 1026389 bytes (78%) of program storage space.
   Maximum is 1310720 bytes.
   Global variables use 38780 bytes (11%) of dynamic memory,
   leaving 288900 bytes for local variables.
   Maximum is 327680 bytes.
*/

#include <PS4Controller.h>
/* https://github.com/aed3/PS4-esp32 read the info provided.
    side note on pairing a PS4 controller.
    i did not use the recomended sixaxis pair tool in the "read-me" file,
    instead i used a phone app to connect my PS4 controller to an unused phone
    and test the controller. After pairing to the phone i simply used the
    BlueTooth MAC address of the phone that i connected the controller to.
*/

//Front Right motor
#define FR_EN    18
#define FR_MP_0  5
#define FR_MP_1  17
//Front Left motor
#define FL_EN    25
#define FL_MP_0  26
#define FL_MP_1  27
//Rear Right motor
#define RR_EN    15
#define RR_MP_0  16
#define RR_MP_1   4
//Rear Left motor
#define RL_EN    13
#define RL_MP_0  12
#define RL_MP_1  14
// Servo pins
#define servoPanPin   23
#define servoTiltPin  19  /* testing purposes - change to any PWM capable GPIO pin */
// Light pin
#define Light_Pin     2  /* testing purposes - change to any PWM capable GPIO pin */
////////////// 60mm regular/mecanum wheels drive selection//////////
//#define REGULAR_WHEELS
#define MECANUM_WHEELS
int FR_PWM_SPD = 0;
int FL_PWM_SPD = 0;
int RR_PWM_SPD = 0;
int RL_PWM_SPD = 0;
/*
   Define Bytes to represent Mecannum Wheel Modes
   Individual bits define L298N motor driver module input states
   B7 = FR_MP_0
   B6 = FR_MP_1
   B5 = FL_MP_0
   B4 = FL_MP_1
   B3 = RR_MP_0
   B2 = RR_MP_1
   B1 = RL_MP_0
   B0 = RL_MP_1
*/
const byte MEC_STRAIGHT_FORWARD = B10101010;
const byte MEC_STRAIGHT_BACKWARD = B01010101;
const byte MEC_SIDEWAYS_RIGHT = B01101001;
const byte MEC_SIDEWAYS_LEFT = B10010110;
const byte MEC_DIAGONAL_45 = B00101000;
const byte MEC_DIAGONAL_135 = B10000010;
const byte MEC_DIAGONAL_225 = B00010100;
const byte MEC_DIAGONAL_315 = B01000001;
const byte MEC_PIVOT_RIGHT_FORWARD = B00100010;
const byte MEC_PIVOT_RIGHT_BACKWARD = B00010001;
const byte MEC_PIVOT_LEFT_FORWARD = B10001000;
const byte MEC_PIVOT_LEFT_BACKWARD = B01000100;
const byte MEC_ROTATE_CLOCKWISE = B01100110;
const byte MEC_ROTATE_COUNTERCLOCKWISE = B10011001;
const byte MEC_PIVOT_SIDEWAYS_FRONT_RIGHT = B01100000;
const byte MEC_PIVOT_SIDEWAYS_FRONT_LEFT = B10010000;
const byte MEC_PIVOT_SIDEWAYS_REAR_RIGHT = B00001001;
const byte MEC_PIVOT_SIDEWAYS_REAR_LEFT = B00000110;

float ServoTiltPosition;
int ServoPanPosition;

float ServoTiltPosition_Minimum  =  90;  /* Setting PWM properties */
float ServoTiltPosition_Center   = 360;  /* Setting PWM properties */
float ServoTiltPosition_Maximum  = 540;  /* Setting PWM properties */

const int PWMResolution = 12;        /* Setting PWM Resolution for servos and Light
                                        if changed servo postions will also need to
                                        be changed
*/
const int PWMResolution_motors = 8;  /* Setting PWM Resolution for motors
                                        dont change, sketch will not work.
*/

int Light_Brightness          = 0;  /* Setting PWM properties */
const int Brightness_Minimum  = 0;  /* Setting PWM properties */
const int Brightness_Maximum  = (int)(pow(2, PWMResolution) - 1);  /* Setting PWM properties */
const int Speed_Minimum       = (int)(pow(2, PWMResolution_motors) - 1);  /* use (-Speed_Minimum) */
const int Speed_Maximum       = (int)(pow(2, PWMResolution_motors) - 1);  /* Setting PWM properties */



const int FR_Channel     = 0;  /* Setting PWM Channel */
const int FL_Channel     = 1;  /* Setting PWM Channel */
const int RR_Channel     = 2;  /* Setting PWM Channel */
const int RL_Channel     = 3;  /* Setting PWM Channel */
const int Light_Channel  = 4;  /* Setting PWM Channel */
const int Tilt_Channel   = 5;  /* Setting PWM Channel */
const int Pan_Channel    = 6;  /* Setting PWM Channel */

const int Light_Freq    = 75;  /* Setting PWM freqency to 75Hz - issues will occur if changed */
const int Servo_Freq    = 50;  /* Setting PWM freqency to 50Hz */
const int Motor_Freq = 19000;  /* Setting PWM freqency to 19 KHz - set to eliminate motor whine */

void moveMotors(int speedFR, int speedFL, int speedRR, int speedRL, byte dircontrol) {

  // Moves all 4 motors
  // Directions specified in direction byte

  // Right Front Motor
  digitalWrite(FR_MP_0, bitRead(dircontrol, 7));
  digitalWrite(FR_MP_1, bitRead(dircontrol, 6));
  ledcWrite(FR_Channel, abs(speedFR));

  // Left Front Motor
  digitalWrite(FL_MP_0, bitRead(dircontrol, 5));
  digitalWrite(FL_MP_1, bitRead(dircontrol, 4));
  ledcWrite(FL_Channel, abs(speedFL));

  // Right Rear Motor
  digitalWrite(RR_MP_0, bitRead(dircontrol, 3));
  digitalWrite(RR_MP_1, bitRead(dircontrol, 2));
  ledcWrite(RR_Channel, abs(speedRR));

  // Left Rear Motor
  digitalWrite(RL_MP_0, bitRead(dircontrol, 1));
  digitalWrite(RL_MP_1, bitRead(dircontrol, 0));
  ledcWrite(RL_Channel, abs(speedRL));
}

void stopMotors() {

  // Stops all motors and motor controllers
  ledcWrite(FR_Channel, 0);
  ledcWrite(FL_Channel, 0);
  ledcWrite(RR_Channel, 0);
  ledcWrite(RL_Channel, 0);

  digitalWrite(FR_MP_0, 0);
  digitalWrite(FR_MP_1, 0);
  digitalWrite(FL_MP_0, 0);
  digitalWrite(FL_MP_1, 0);
  digitalWrite(RR_MP_0, 0);
  digitalWrite(RR_MP_1, 0);
  digitalWrite(RL_MP_0, 0);
  digitalWrite(RL_MP_1, 0);
}
void notify() {
  control_begin();
}

void setup()
{
  Serial.begin(115200);

  PS4.attach(notify);
  PS4.begin("00:00:00:00:00:00");// change to match your paired BluTooth MAC Address

  //Front Right motor
  pinMode(FR_EN   , OUTPUT);
  pinMode(FR_MP_0 , OUTPUT);
  pinMode(FR_MP_1 , OUTPUT);

  //Front Left motor
  pinMode(FL_EN   , OUTPUT);
  pinMode(FL_MP_0 , OUTPUT);
  pinMode(FL_MP_1 , OUTPUT);

  //Rear Right motor
  pinMode(RR_EN   , OUTPUT);
  pinMode(RR_MP_0 , OUTPUT);
  pinMode(RR_MP_1 , OUTPUT);

  //Rear Left motor
  pinMode(RL_EN   , OUTPUT);
  pinMode(RL_MP_0 , OUTPUT);
  pinMode(RL_MP_1 , OUTPUT);

  ledcSetup(Pan_Channel, Servo_Freq, PWMResolution);
  ledcSetup(Tilt_Channel, Servo_Freq, PWMResolution);
  ledcSetup(Light_Channel, Light_Freq, PWMResolution);
  ledcSetup(FR_Channel, Motor_Freq, PWMResolution_motors);
  ledcSetup(FL_Channel, Motor_Freq, PWMResolution_motors);
  ledcSetup(RR_Channel, Motor_Freq, PWMResolution_motors);
  ledcSetup(RL_Channel, Motor_Freq, PWMResolution_motors);

  ledcAttachPin(FR_EN, FR_Channel);          /* Attach the FR_Channel to the GPIO Pin */
  ledcAttachPin(FL_EN, FL_Channel);          /* Attach the FL_Channel to the GPIO Pin */
  ledcAttachPin(RR_EN, RR_Channel);          /* Attach the RR_Channel to the GPIO Pin */
  ledcAttachPin(RL_EN, RL_Channel);          /* Attach the RL_Channel to the GPIO Pin */
  ledcAttachPin(Light_Pin, Light_Channel);   /* Attach the Light_Channel to the GPIO Pin */
  ledcAttachPin(servoTiltPin, Tilt_Channel); /* Attach the Tilt_Channel to the GPIO Pin */
  ledcAttachPin(servoPanPin, Pan_Channel);   /* Attach the Pan_Channel to the GPIO Pin */

  ledcWrite(Light_Channel, Light_Brightness);/* Set Light to 0 brightness/OFF*/
}
void loop() {
}

void control_begin() {

  int ServoPanPosition;
  int servoPanLeftPosition  = map( PS4.L2Value(), 0, 255, 306, 548);    //L2 value 0-255
  int servoPanRightPosition = map( PS4.R2Value(), 0, 255, 306, 61);  //R2 value 0-255

  if (servoPanLeftPosition > 307) {
    ServoPanPosition = servoPanLeftPosition;
  }
  else if (servoPanRightPosition < 304) {
    ServoPanPosition = servoPanRightPosition;
  }
  else {
    ServoPanPosition = 305;
  }
  if (PS4.L1()) {
    ServoTiltPosition = ServoTiltPosition + 0.55;
  }
  if (PS4.R1()) {
    ServoTiltPosition = ServoTiltPosition - 0.55;
  }
  if (PS4.L3()) {
    ServoTiltPosition = ServoTiltPosition_Center;
  }
  if (ServoTiltPosition > ServoTiltPosition_Maximum) {
    ServoTiltPosition = ServoTiltPosition_Maximum;
  }
  if (ServoTiltPosition < ServoTiltPosition_Minimum) {
    ServoTiltPosition = ServoTiltPosition_Minimum;
  }
  if (PS4.Square()) {
    Light_Brightness = Light_Brightness + 2;
  }
  if (PS4.Circle()) {
    Light_Brightness = Light_Brightness - 2;
  }
  if (PS4.Cross()) {
    Light_Brightness = Brightness_Minimum;
  }
  if (PS4.Triangle()) {
    Light_Brightness = Brightness_Maximum;
  }
  if (Light_Brightness > Brightness_Maximum) {
    Light_Brightness = Brightness_Maximum;
  }
  if (Light_Brightness < Brightness_Minimum) {
    Light_Brightness = Brightness_Minimum;
  }

#if defined(REGULAR_WHEELS)
  int Speed = map( PS4.LStickY(), -128, 127, -Speed_Minimum, Speed_Maximum);
  int Speed1 = map( PS4.RStickY(), -128, 127, -Speed_Minimum, Speed_Maximum);

  if (Speed > 0 && Speed1 > 0) {
    FR_PWM_SPD = Speed1;
    FL_PWM_SPD = Speed;
    RR_PWM_SPD = Speed1;
    RL_PWM_SPD = Speed;
    moveMotors(FR_PWM_SPD, FL_PWM_SPD, RR_PWM_SPD, RL_PWM_SPD, MEC_STRAIGHT_FORWARD);
  }
  else if (Speed < 0 && Speed1 < 0) {
    FR_PWM_SPD = Speed1;
    FL_PWM_SPD = Speed;
    RR_PWM_SPD = Speed1;
    RL_PWM_SPD = Speed;
    moveMotors(FR_PWM_SPD, FL_PWM_SPD, RR_PWM_SPD, RL_PWM_SPD, MEC_STRAIGHT_BACKWARD);
  }
  else if (Speed < 0 && Speed1 > 0) {
    FR_PWM_SPD = Speed1;
    FL_PWM_SPD = Speed;
    RR_PWM_SPD = Speed1;
    RL_PWM_SPD = Speed;
    moveMotors(FR_PWM_SPD, FL_PWM_SPD, RR_PWM_SPD, RL_PWM_SPD, MEC_ROTATE_COUNTERCLOCKWISE);
  }
  else if (Speed > 0 && Speed1 < 0) {
    FR_PWM_SPD = Speed1;
    FL_PWM_SPD = Speed;
    RR_PWM_SPD = Speed1;
    RL_PWM_SPD = Speed;
    moveMotors(FR_PWM_SPD, FL_PWM_SPD, RR_PWM_SPD, RL_PWM_SPD, MEC_ROTATE_CLOCKWISE);
  }
  else {
    stopMotors();
  }


#elif defined(MECANUM_WHEELS)
  int Rx = map( PS4.RStickX(), -128, 127, -Speed_Minimum, Speed_Maximum); //Right stick - X axis
  int Ly = map( PS4.LStickY(), -128, 127, -Speed_Minimum, Speed_Maximum); //Left stick  - Y axis
  int Lx = map( PS4.LStickX(), -128, 127, -Speed_Minimum, Speed_Maximum); //Left stick  - X axis

  if (Ly > 50) {
    FR_PWM_SPD = Ly;
    FL_PWM_SPD = Ly;
    RR_PWM_SPD = Ly;
    RL_PWM_SPD = Ly;
    moveMotors(FR_PWM_SPD, FL_PWM_SPD, RR_PWM_SPD, RL_PWM_SPD, MEC_STRAIGHT_FORWARD);
  }
  else if (Ly < -50) {
    FR_PWM_SPD = Ly;
    FL_PWM_SPD = Ly;
    RR_PWM_SPD = Ly;
    RL_PWM_SPD = Ly;
    moveMotors(FR_PWM_SPD, FL_PWM_SPD, RR_PWM_SPD, RL_PWM_SPD, MEC_STRAIGHT_BACKWARD);
  }
  else if (Rx < -50) {
    FR_PWM_SPD = Rx;
    FL_PWM_SPD = Rx;
    RR_PWM_SPD = Rx;
    RL_PWM_SPD = Rx;
    moveMotors(FR_PWM_SPD, FL_PWM_SPD, RR_PWM_SPD, RL_PWM_SPD, MEC_ROTATE_COUNTERCLOCKWISE);
  }
  else if (Rx > 50) {
    FR_PWM_SPD = Rx;
    FL_PWM_SPD = Rx;
    RR_PWM_SPD = Rx;
    RL_PWM_SPD = Rx;
    moveMotors(FR_PWM_SPD, FL_PWM_SPD, RR_PWM_SPD, RL_PWM_SPD, MEC_ROTATE_CLOCKWISE);
  }
  else if (Lx < -50) {
    FR_PWM_SPD = Lx;
    FL_PWM_SPD = Lx;
    RR_PWM_SPD = Lx;
    RL_PWM_SPD = Lx;
    moveMotors(FR_PWM_SPD, FL_PWM_SPD, RR_PWM_SPD, RL_PWM_SPD, MEC_SIDEWAYS_LEFT);
  }
  else if (Lx > 50) {
    FR_PWM_SPD = Lx;
    FL_PWM_SPD = Lx;
    RR_PWM_SPD = Lx;
    RL_PWM_SPD = Lx;
    moveMotors(FR_PWM_SPD, FL_PWM_SPD, RR_PWM_SPD, RL_PWM_SPD, MEC_SIDEWAYS_RIGHT);
  }
  else {
    stopMotors();
  }
#endif
  ledcWrite(Pan_Channel   ,  ServoPanPosition)  ;
  ledcWrite(Light_Channel ,  Light_Brightness)  ;
  ledcWrite(Tilt_Channel  ,  ServoTiltPosition) ;
}
