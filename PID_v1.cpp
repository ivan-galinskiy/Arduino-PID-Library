/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/
 
 // Additional changes by Ivan Galinskiy. Note: this library and the original
 // are NOT compatible

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_v1.h>

// Main class
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd)
{
	
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
	
	// Controller disabled by default
	inAuto = false;
	
	// Set output limits to Arduino PWM limits
	PID::SetOutputLimits(0, 255);

	// Default sample time is 0.1 seconds
    SampleTime = 1e6/10;
	
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = micros() - SampleTime;				
}
 
 
// Compute and set the output value. Returns true if a calculation has been performed, false otherwise. 
bool PID::Compute()
{
   unsigned long now = micros();
   unsigned long timeChange = (now - lastTime);
   
   if(!inAuto) 
	   return false;
   
   if(timeChange >= SampleTime)
   {
	  double input = *myInput;
	  
	  // Error value (as defined in most control literature
      double error = *mySetpoint - input;
	  
	  // Integral term with saturation
      ITerm += (ki * error);
      if (ITerm > outMax) 
		  ITerm = outMax;
      else if (ITerm < outMin) 
		  ITerm = outMin;
	  
	  // Differential term. Note that we are not differentiating the error,
	  // but the input (avoiding derivative kicks with setpoint changes)
      double dInput = (input - lastInput);
 
      // PID Output
      double output = kp * error + ITerm + kd * dInput;
      
	  if (output > outMax) 
		  output = outMax;
      else if (output < outMin) output = outMin;
	  *myOutput = output;
	  
      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
	  return true;
   }
   else return false;
}



void PID::SetTunings(double Kp, double Ki, double Kd)
{
   fullKp = Kp; 
   fullKi = Ki; 
   fullKd = Kd;
   
   double SampleTimeInSec = ((double)SampleTime)/1e6;  
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
  
// Set the sampling time (must be supplied in seconds)
void PID::SetSampleTime(double NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      SampleTime = (unsigned long) (NewSampleTime*1e6);
	  PID::SetTunings(fullKp, fullKi, fullKd);
   }
}
 
// Set the output limits of the PID output
void PID::SetOutputLimits(double Min, double Max)
{
	// Sanity check
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
	   // If the new limits saturate the current outputs, apply that
	   if (*myOutput > outMax) 
		   *myOutput = outMax;
	   else if (*myOutput < outMin) 
		   *myOutput = outMin;
	 
	   if (ITerm > outMax) 
		   ITerm= outMax;
	   else if (ITerm < outMin) 
		   ITerm= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
 // Important: before turning on the PID, set the Output variable to the
 // desired bias. Then, the transfer will be bumpless.
void PID::Initialize()
{
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  fullKp; }
double PID::GetKi(){ return  fullKi;}
double PID::GetKd(){ return  fullKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}

