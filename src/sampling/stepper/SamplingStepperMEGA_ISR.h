/*
  ISR for handling the interrupt-driven sampling for 
  real-time control on the Arduino Mega 2560.
  
  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons

  Gergely Takacs, 2019
  Last update: 3.6.2019.
*/

#ifndef SAMPLINGSTEPPERMEGA_ISR_H
#define SAMPLINGSTEPPERMEGA_ISR_H

ISR(TIMER5_COMPA_vect)
{
 if (!SamplingStepper.fireFlag){                   // If not over the maximal resolution of the counter
  (SamplingStepper.getInterruptCallback())();      // Start the interrupt callback
 }                                          
 else if(SamplingStepper.fireFlag){                // else, if it is over the resolution of the counter
     SamplingStepper.fireCount++;                    // start counting
   if (SamplingStepper.fireCount>= SamplingStepper.getSamplingMicroseconds()/ SamplingStepper.fireResolution){ // If done with counting
       SamplingStepper.fireCount=0;                 // make the counter zero again
      (SamplingStepper.getInterruptCallback())();  // and start the interrupt callback
  } 
 }
}
#endif
