/*
  ISR for handling the interrupt-driven sampling for
  real-time control on the Arduino Zero and the
  Adafruit Metro M4 Express, e.g. SAMD device-based boards.

  This code is part of the AutomationShield hardware and software
  ecosystem. Visit http://www.automationshield.com for more
  details. This code is licensed under a Creative Commons

  Gergely Takacs, 2019
  Last update: 3.6.2019.
*/

#ifndef SAMPLINGSTEPPERSAMD_ISR_H
#define SAMPLINGSTEPPERSAMD_ISR_H

void TC4_Handler (void) {
 if (!SamplingStepper.fireFlag){                   // If not over the maximal resolution of the counter
   //Interrupt can fire before step is done!!!
   TC4->COUNT16.INTFLAG.bit.MC0 = 1;    	// Clear the interrupt
   (SamplingStepper.getInterruptCallback())();	    // Launch interrupt handler
 }
 else if(SamplingStepper.fireFlag){                // Else, if period is over the resolution of the counter
    //Interrupt can fire before step is done!!!
   SamplingStepper.fireCount++;                    // Start counting
   if (SamplingStepper.fireCount==SamplingStepper.getSamplingMicroseconds()/SamplingStepper.fireResolution){ // If done with counting
	SamplingStepper.fireCount=0;                   // Make the counter zero again
	(SamplingStepper.getInterruptCallback())();    // Launch interrupt handler
   }
   TC4->COUNT16.INTFLAG.bit.MC0 = 1;        //Clear the interrupt
 }
}
#endif