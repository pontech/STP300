// chipKIT-core 2.1.0 - MPIDE Version 20140316 (untested)
#include <libSTP300.h>

//////////////////////////////////////////////////////////
// Setup Code
//////////////////////////////////////////////////////////
void setup() {
  setup_STP300();
  createHeartbeat(DEBUGLED, 2000);
}

//////////////////////////////////////////////////////////
// Loops
//////////////////////////////////////////////////////////

void loop() {
  loop_stp300_homing_task();
  loop_stp300_serial_parser();
}

//////////////////////////////////////////////////////////
// Stepper Motor Safe to move callback
//////////////////////////////////////////////////////////
// User function that returns true ONLY when it is
// safe to move the motor.
//////////////////////////////////////////////////////////

bool safeToMove(bool directionPositive)
{
  if(!ram.stophomingonly) //if motor is homing or homingonly not set
  {
    if(directionPositive)
    {
      //Serial.println((!digitalRead(posHome) && ram.homeswitchnc) || (digitalRead(posHome) && !ram.homeswitchnc),DEC);
      return (!digitalRead(posHome) && ram.homeswitchnc) || (digitalRead(posHome) && !ram.homeswitchnc);
    } else {
      //Serial.println((!digitalRead(negHome) && ram.homeswitchnc) || (digitalRead(negHome) && !ram.homeswitchnc),DEC);
      return (!digitalRead(negHome) && ram.homeswitchnc) || (digitalRead(negHome) && !ram.homeswitchnc);
    }
  } else {
    return true;
  }
}
