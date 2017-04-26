A slightly modified version of Brett Beauregard's Arduino PID library (original description in the end of this README). The changes are:

1. Timing in microseconds instead of milliseconds. This allows a faster update rate (> kHz). The sample time can be set to 0 so that the PID processes the data as fast as it can and measures the sample time internally.

2. "Controller direction" has been removed. Instead, the sign of the tunings (which is no longer forced to be positive) determines the response.

3. The summation of PID terms is now "out = Kp*err + Kd*derivative(err) + Ki*integral(err)". The original version has a minus on the differential term.

4. Integrator and last value can now be reset for cases of large changes of setpoint. Otherwise, integral and differential terms can become a problem.

***************************************************************
* Arduino PID Library - Version 1.1.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under a GPLv3 License
***************************************************************

 - For an ultra-detailed explanation of why the code is the way it is, please visit: 
   http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

 - For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary
