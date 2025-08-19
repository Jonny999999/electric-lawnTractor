#pragma once

#include <avr/io.h>

// Calculate PWM frequency for logging
#define PWM_FREQUENCY(PWM_PRESCALER, PWM_RESOLUTION) ((F_CPU) / (PWM_PRESCALER) / ((PWM_RESOLUTION) + 1))

// configure and initialize timer1 in Fast-pwm mode and output on pin PB1
// returns 1 if a parameter was not an allowed value
// Possible values resolution: 1023(10Bit), 511(9Bit), 255(8Bit)
// Possible values prescaler:  1, 8, 64, 256, 1024
int pwm_initFastPwmTimer1(uint16_t resolution, uint16_t prescaler);

//Pre-calculated possible combinations:
/*
|  Resolution  |  Prescaler  |  Resulting PWM-Freq @ 8MHz  |
   1023       	  1	            7813   
   1023       	  8	            977    
   1023       	  64	        122           
   1023       	  256	        31
   1023       	  1024	        8
   		
   511	           1	        15625 
   511	           8	        1953  
   511	           64	        244    
   511	           256	        61
   511	           1024	        15
   		
   255	           1	        31250
   255	           8	        3906  
   255	           64	        488    
   255	           256	        122
   255	           1024          31
*/




void pwm_disable();



//max duty is PWM_RESOLUTION pwm was initialized with
//note: due to fast-pwm mode pin can never be fully off or on
// there will still be one off/on cycle at 1023/0 duty
void pwm_setDutyCycle(uint16_t dutyCycle_compareValue);