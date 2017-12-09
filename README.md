# LatheDuino
Arduino code for the Jordan Kube Open Lathe project

The basics are this: An Arduino compatible controller, with a few analog inputs and digital inputs for controls, and at an 
absolute minimum two digital outputs for driving the stepper motor controllers.
More I/O gives more features.

I am starting with the following hardware:
Ebay: 332406924434  Industrial-PLC-Open-Source-Arduino-Mega-2560-Kit-DIN-Rail-Mount-LAN-WiFi-RTC
Ebay: 271664191307  Proximity sensor for "home" indication on the chucks.
Sparkfun: Piezo Vibration Sensor - Large with Mass  for sensing excessive vibration.
SparkFun: Arcade Joystick - Short Handle for speed and "twist" commands
Sparkfun: Foot Pedal Switch - for "deadman" switch shutdown.

Sitting inbetween the Arduino and the stepper motors, a pair of industrial stepper drivers, and a 48V SMPS to supply power.

Basics:

The first order of business, after getting everything in hand, is to create a skeleton application, and implement some basic functions

Speed: Given that I want to adjust the motor speed independently to create "twist" effects, I need timer based output of two 
speed pulses that can be either in sync (normal) or slightly different speeds (twisting) I don't have to implement this on day one
but I need to consider it in the design.

From the Joystick, we have four normally open switches, which will go to digital inputs for "Faster", "Slower", "Twist+" and "Twist-"
We have the home sensor, which is also a digital input, and will be used to detect unplanned slippage.
We have the vibration sensor, which is an analog input, which will be used to slow or stop the lathe if high vibration occurs.
We have a number of button inputs possible, and the first one will be footswitch, which is an orderly deceleration to stopped.
"Sync" switch will restore both motors to the same speed.

Much will change, but here's how I see it running at this point:

Power switch on:
Display lights up and shows machine status as stopped, sanity check on all inputs. 
Optional: Require all inputs be tested before the machine goes active.

Idle loop:
"Faster" switch closes, and the machine accelerates slowly from stopped, terminating accel when "Faster" is released or max speed.
"Slower" switch closes, and the machine decelerates slowly from speed terminating decel when "Slower" is released, or speed=0.
"Twist+" switch closes, and if the speed is not zero, then the headstock motor slows down slightly.
"Twist-" switch closes, and if the speed is not zero, then the tailstock motor slows down slightly.
"FootSwitch" switch closes and if the speed is not zero the speed is reduced to zero.
"Sync" switch puts both motors at the same speed, by increasing the slow motor. If both are same speed no action.
Any speed change is vetted against maximum speed limit. Note that twist is always handled by slowing a motor.
Sanity check function will execute in the main loop, and this will be the only point where the processor watchdog timer is handled.

On interrupt from either of the proximity sensors, the step count is checked to determine if the stepper has slipped. 
Not sure how to handle this, except to decrease max speed and accel/decel rates.

