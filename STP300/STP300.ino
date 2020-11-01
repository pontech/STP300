// chipKIT-core 2.1.0 - MPIDE Version 20140316 (untested)
#include <libSTP300.h>

Cron cron(micros);

//////////////////////////////////////////////////////////
// Task to flash LED
//////////////////////////////////////////////////////////

void flash() {
  Cron::CronDetail *self = cron.self();

  digitalWrite(DEBUGLED, self->temp);

  self->temp ^= 1;
  self->yield = micros() + 1000000;
}

//////////////////////////////////////////////////////////
// Setup Code
//////////////////////////////////////////////////////////
void setup() {
  setup_STP300();
  cron.add(flash);
}

//////////////////////////////////////////////////////////
// Loops
//////////////////////////////////////////////////////////

void loop() {
  cron.scheduler(); // For flashing the LED
  loop_stp300_homing_task();
  loop_stp300_serial_parser();
}

//////////////////////////////////////////////////////////
// Stepper Motor Safe to move callback
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
