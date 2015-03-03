#include <SPI.h>
#include <dSPIN_L6472.h>

L6472 stepper(11, 12, 13, 6); //MOSI, MISO, SCK, SS

void setup(){
   Serial.begin(9600);
   
  if (stepper.init(1.5, 0.2) == 0)
    ; //Init Successfull
    
  stepper.setAcc(400); //set acceleration
  stepper.setMaxSpeed(1000);  
  
  stepper.setMinSpeed(10);
  stepper.setMicroSteps(16); //1,2,4,8,16
  stepper.setThresholdSpeed(1000);
  stepper.setOverCurrent(6000); //set overcurrent protection
//  stepper.setStallCurrent(1000);
  
  //stepper.run(1, 200);  
  
  //stepper.softStop();
  
  stepper.goTo(200);  
  
}

void loop(){  
  
  while(stepper.isBusy()){
     delay(10); 
  }
  delay(500);
  
  stepper.goTo(0);
  
  while(stepper.isBusy()){
     delay(10); 
  }
  
  delay(500);

  stepper.goTo(3200);


}