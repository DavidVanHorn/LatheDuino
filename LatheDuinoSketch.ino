#include <AccelStepper.h>
#include <MultiStepper.h>


#include <avr/wdt.h>


#include <avr/interrupt.h>
#include <Arduino.h>
//#include <TMRpcm_PLDuino.h>
//#include <SPI.h>
//#include <SD.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_ILI9341.h>
#include <PLDuino.h>
//#include <PLDTouch.h>
//#include <PLDuinoGUI.h>
//#include <using_namespace_PLDuinoGUI.h>
//#include <DS3232RTC.h>
//#include <Time.h>
//#include <Wire.h>
//#include "utils.h"




// AVR data sheet
// http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf
// Motor controllers data sheet
// https://www.automationtechnologiesinc.com/wp-content/uploads/downloads/2014/05/KL-5056D.pdf
// 200kHz fastest step which equates to 2.5uS minimum pulse width.
// "For  reliable  response,  pulse  width  should  be longer than 2.5μs. "

// Timers:
// In Arduinoland, various timers get used fo different things.  This is intended for
// a 2560 board, if ported to some other board then use a free timer.
//
// 2560 timer allocations
// Timer0: runs delay(), millis() and micros(). PWM 4 and 13
// Timer1: Could be servo library or could be free  PWM 11 and 12
// Timer2: tone() PWM 9 and 10
// Timer3: 16 bit, free PWM 2,3,5
// Timer4: 16 bit, free PWM 6,7,8
// Timer5: 16 bit, used if servo library is used  PWM 44,45,46

// PLC I/O pins
// D46 LCD backlight
// D27 3.3V to LCD, Touch panel, and micro SD card
// D9  Audio by PWM
// D13 LED
// D26 power to wifi chip

// Joystick control inputs
#define StickPin1 34  // faster
#define StickPin2 36  // slower
#define StickPin3 35  // Twist +
#define StickPin4 37  // Twist -

// HeadStock Motor pins
#define HeadStepPin 44 // because timer 5 has PWM here
#define HeadEnablePin 38
#define HeadDirectionPin 43

// TailStock Motor pins
#define TailStepPin 45 // because timer 5 has PWM here
#define TailEnablePin 39
#define TailDirectionPin 41


// Headstock motor
AccelStepper HeadStock(AccelStepper::DRIVER, HeadStepPin, HeadDirectionPin); // Interface type, step pin, direction pin
// Tailstock motor
AccelStepper TailStock(AccelStepper::DRIVER, TailStepPin, TailDirectionPin);


// Somewhat arbitrary, there's really no "forward" on a glass lathe.
//#define MotorForward HIGH
//#define MotorReverse LOW

#define LEDPin 13

// Outputs from switch sensing, not conditioned
boolean Faster;                 // true if switch was closed on last scan
boolean Slower;                 // set to false once they have been processed
boolean TwistPlus;              //
boolean TwistMinus;             //
boolean new_stick;              // set to true when a stick scan has been performed


// This code executes once, setting up the I/O pins and variables.
void setup() {
  wdt_init();               // Init watchdog timer
  LED_init();               // inits LED to off.
  switches_init();          // init inputs and debounce for switches
  analog_inputs_init();     // init inputs for analog controls
  //display_init();         //
  steppers_init();          // init accelstepper functions
  HeadStock.setSpeed(0);    //
  TailStock.setSpeed(0);    //
  interrupts();             // enable interrupts before dropping into main
}

// This code loops forever while we are running.
void loop() {
  HeadStock.runSpeed();   // Execute these as fast as possible
  TailStock.runSpeed();   //
  check_digital_inputs(); // did any of the switches change state?
  HeadStock.runSpeed();   //
  TailStock.runSpeed();   //
  speed_adjust();         // Change the motor speed if it's time
  HeadStock.runSpeed();   //
  TailStock.runSpeed();   //
  sanity_check();         // Is everybody happy?
}
//****************************************************************************************
// Accelstepper init
//****************************************************************************************
#define MaxSpeed 4500.0   // steps per second. 6600 max on this CPU but motors quit at about 4500
                          // on low current drive
#define MaxAccel 1000.0   // Without load, max accel > 200000 @ 30V
#define Rotation (3600/9) // steps per rotation of motor  (1.8 deg/step)
#define MinStepPulse 0    // Microseconds low, added to 20uS min

void steppers_init(void){    
    HeadStock.setMaxSpeed(MaxSpeed);
    HeadStock.setAcceleration(MaxAccel);
    HeadStock.setPinsInverted(false, false, true); // Invert direction, step, enable
    HeadStock.setMinPulseWidth(MinStepPulse); // in microseconds
    
    TailStock.setMaxSpeed(MaxSpeed);
    TailStock.setAcceleration(MaxAccel);
    TailStock.setPinsInverted(true, false, true); // Invert direction, step, enable
    TailStock.setMinPulseWidth(MinStepPulse); // in microseconds
    }

void HeadStock_Jog(void){
  HeadStock.runToNewPosition(0);
  HeadStock.runToNewPosition(Rotation*3);
  HeadStock.runToNewPosition(0);
}

void TailStock_Jog(void){
  TailStock.runToNewPosition(0);
  TailStock.runToNewPosition(Rotation*3);
  TailStock.runToNewPosition(0);
}

//****************************************************************************************
// speed
//****************************************************************************************
//
// Desired speed is a variable that starts at zero (stopped) and increments to some max value.
// At the timer level, I need the inverse of this, bigger speed number is less time per step.
//
#define speed_increment 5 // how much speed change per loop
#define twist_increment 1

// If the stick was scanned then new_stick will be true 
void speed_adjust(void) {
  if (new_stick == false){
    return;
  }
  
  // One of these may be true, if so then do it.
  // If none is true, then just make sure head and tail are running the same speed.
  if (Faster){
    speed_up();
  } else if (Slower){
    speed_down();
  } else if (TwistPlus){
    head_twist();
  } else if (TwistMinus){
    tail_twist();
  } else {
   speed_sync(); // no stick command so sync speeds
  }
  Faster = false;    
  Slower = false;    
  TwistPlus = false;
  TwistMinus = false;
  new_stick = false;
}



// Goal: restore both motors to the same speed.
// This is done by slowing the fast motor down to match the slow motor.
void speed_sync(void){
  // Get the two speeds
  float head = HeadStock.speed();
  float tail = TailStock.speed();
  
  if (head == tail){
    return; // Nothing to do.
  }
  
  // If the head is going faster than the tail
  if (abs(head) > abs(tail)) {
    HeadStock.setSpeed(tail); // Slow the head down to match the tail
    return;
  } else {
    // we already handled the equal case.
    TailStock.setSpeed(head); // Slow the tail down to match the head
  }
}

// speed can be positive or negative.
// if spinning positive, then up means faster. (more positive)
// if spinning negative, then up means slower. (less negative)
void speed_up(void){
  
  float head = HeadStock.speed();
  head = head + speed_increment;
  if (head > MaxSpeed){
    return;
  }
  if (head < (-1*MaxSpeed)){
    return;
  }
  // Acceptable speed, set it.
  HeadStock.setSpeed(head);
  
  float tail = TailStock.speed();
  tail = tail + speed_increment;
  if (tail > MaxSpeed){
    return;
  }
  if (tail < (-1*MaxSpeed)){
    return;
  }
  TailStock.setSpeed(tail);
}

// speed can be positive or negative.
// if spinning positive then down means slower (less positive)
// if spinning negative then down means faster (more negative)
void speed_down(void){
  
  float head = HeadStock.speed();
  head = head - speed_increment;
  if (speed_test(head)){
    HeadStock.setSpeed(head);
  } 
  
  float tail = TailStock.speed();
  tail = tail - speed_increment;
  if (speed_test(tail)){
    TailStock.setSpeed(tail);
  } 
}

// Twist is always implemented by slowing down a motor
// This avoids any problems with running into max speed.
void head_twist(void){
  float head = HeadStock.speed();
  head = speed_slower(head);
  HeadStock.setSpeed(head);
}

void tail_twist(void){
  float tail = TailStock.speed();
  tail = speed_slower(tail);
  TailStock.setSpeed(tail);
}

// speed can be positive or negative so compare absolute values
// MaxSpeed is positive always
bool speed_test (float speed){
  if (abs(speed) < MaxSpeed){
    return(true);
  } else {
    return(false);
  }
}

float speed_slower (float speed){
  if (speed == 0){
    return speed;
  } else if (speed > 0){
    speed = speed - speed_increment;
  } else {
    speed = speed + speed_increment;
  }
  return speed;
}

//****************************************************************************************
// Switch Input Functions
//****************************************************************************************

// Decrement to zero to pace reading the stick
#define StickTime 10 // milliseconds between scans 
unsigned long StickTimer;

// Set up the I/O pins for the control switches.
void switches_init(void) {
  stick_init();
  // add other functions here
}

// This just handles the joystick
void stick_init(void) {
  pinMode(StickPin1, INPUT_PULLUP);
  pinMode(StickPin2, INPUT_PULLUP);
  pinMode(StickPin3, INPUT_PULLUP);
  pinMode(StickPin4, INPUT_PULLUP);
  Faster = false;
  Slower = false;
  TwistPlus = false;
  TwistMinus = false;
  StickTimer = millis();
}

// Is it time to check the stick inputs?
void check_digital_inputs(void) {
  if ((unsigned long)(millis() - StickTimer) >= StickTime) {
      StickTimer = millis();
      check_digital_inputs_go();
   }
}

// Did any switches change state?
// LOW = input active, set the flag to true if the input is low.
void check_digital_inputs_go(void){
  // If the switch is closed, then the input is LOW
  if (digitalRead (StickPin1)) {
    // High means switch is NOT closed.
    Faster = false;
  } else {
    // Low means switch IS closed
    Faster = true;
  }
  
  if (digitalRead (StickPin2)) {
    Slower = false;
  } else {
    Slower = true;
  }
  
  if (digitalRead (StickPin3)) {
    TwistPlus = false;
  } else {
    TwistPlus = true;
  }
  
  if (digitalRead (StickPin4)) {
    TwistMinus = false;
  } else {
    TwistMinus = true;
  }
// At this point, if any stick switch was closed, its corresponding boolean is true.
  new_stick = true; // semaphore that we have new stick data
}
//****************************************************************************************
// Motor Functions
//****************************************************************************************
//
// Twist is implemented by slowing down one or the other motors.
// Sync restores them to the same rate.
//
//

void motors_emergency_stop(void) {
  //motor_init();         // disable the motor controllers
//  motor_pause_timer = 5;  // let the load coast to a stop.
}

// Needs to initiate a decelerate to zero.
// Other motor commands locked out till motor stops and a
// short pause happens.

void motors_stop(void) {
  //motor_pause_timer = 3; // seconds
  //desired_speed = 0;
}

//****************************************************************************************
// LED
//****************************************************************************************
// LED output for diagnostics
// I/O pin low means LED is on.

void LED_init(void) {
  pinMode (LEDPin, OUTPUT);
  LED_off();
}

void LED_on(void) {
  digitalWrite(LEDPin, LOW);
}

void LED_off(void) {
  digitalWrite(LEDPin, HIGH);
}

void LED_toggle(void) {
  digitalWrite(LEDPin, digitalRead(LEDPin) ^ 1);
}

// Blocking delay used here
void LED_Blink(void) {
  LED_on();
  delay(1);
  LED_off();
  delay(1);
}

// Blocking delay used here
void LED_blink_slow(void) {
  LED_on();
  delay(20);
  LED_off();
  delay(20);
}
//****************************************************************************************
// Analog Functions
//****************************************************************************************

// Set up analog inputs
void analog_inputs_init(void) {

}

// Check voltages for changes
void check_analog_inputs(void) {

}



//****************************************************************************************
// Sanity Functions
//****************************************************************************************


// check variable values etc and only if all are ok, reset the WDT
void sanity_check(void) {

  // Normal exit point
Sanity_Pass:
  wdt_reset();
  return;

  // Something failed, can't trust anything else now either.
Sanity_Fail:
  motors_emergency_stop();
  wdt_enable(WDTO_250MS);
  while (1) {
    // hang here and wait for reboot.
  }
}
//****************************************************************************************
// Watchdog Functions
//****************************************************************************************
void wdt_init(void) {
  wdt_enable(WDTO_250MS);
}
//****************************************************************************************
// Display Functions
//****************************************************************************************

void display_init(void) {

}

void update_display(void) {

}

/*
  void showSlide (const char *picfile, const char *wavfile)
  {
  long start = millis();
  bmpDraw(tft, picfile, 0, 0);
  playSound(tmrpcm, wavfile);
  }

  #define NORMAL_TIMEOUT        60
  #define PRESENTATION_TIMEOUT   5

  void showInputs(unsigned long timeout);
  void showRelays(unsigned long timeout);

  void demo()
  {
  bool touched = true;
  for(;;)
  {
    tft.fillScreen(ILI9341_BLACK);
    showSlide("slide1.bmp", "part1.wav");
    showSlide("slide2.bmp", "part2.wav");
    showSlide("slide3.bmp", "part3.wav"); touched = waitForTouchOrTimeout(touch, touched? NORMAL_TIMEOUT : PRESENTATION_TIMEOUT);
    showInputs(touched? NORMAL_TIMEOUT : PRESENTATION_TIMEOUT);
    showSlide("slide5.bmp", "part5.wav"); touched = waitForTouchOrTimeout(touch, touched? NORMAL_TIMEOUT : PRESENTATION_TIMEOUT);
    showRelays(touched? NORMAL_TIMEOUT : PRESENTATION_TIMEOUT);
    showSlide("features.bmp", "part7.wav");
  }
  }
*/
