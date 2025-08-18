#include "pwm.h"

// track configured resolution, used in setDuty
uint16_t gCurrentPwmResolution = 0;



// configure and initialize timer1 in Fast-pwm mode and output on pin PB1
// returns 1 if a parameter was not an allowed value
int pwm_initFastPwmTimer1(uint16_t resolution, uint16_t prescaler)
{
  gCurrentPwmResolution = resolution;
  // Set pwm mode Fast-PWM with selected resolution
  // Mode 5-7 Datasheet page 98 Table 39
  if (resolution == 1023)
    TCCR1A = (1 << WGM10) | (1 << WGM11); // Mode 7: Fast PWM, 10-bit
  else if (resolution == 511)
    TCCR1A = (0 << WGM10) | (1 << WGM11); // Mode 6: Fast PWM, 9-bit
  else if (resolution == 255)
    TCCR1A = (1 << WGM10) | (0 << WGM11); // Mode 5: Fast PWM, 8-bit
  else
    // invalid resolution value
    return 1;

  // Set the prescaler based on the selected value
  if (prescaler == 1)
    TCCR1B = (1 << WGM12) | (1 << CS10); // No prescaling
  else if (prescaler == 8)
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler = 8
  else if (prescaler == 64)
    TCCR1B = (1 << WGM12) | ((1 << CS11) | (1 << CS10)); // Prescaler = 64
  else if (prescaler == 256)
    TCCR1B = (1 << WGM12) | (1 << CS12); // Prescaler = 256
  else if (prescaler == 8)
    TCCR1B = (1 << WGM12) | ((1 << CS12) | (1 << CS10)); // Prescaler = 1024
  else
    // invalid prescaler value
    return 1;

  // Set non-inverting mode
  TCCR1A |= (1 << COM1A1); // Clear OC1A on Compare Match, set OC1A at BOTTOM
  // If using OCR1B, uncomment the next line
  // TCCR1A |= (1 << COM1B1);  // Clear OC1B on Compare Match, set OC1B at BOTTOM

  // Set PB1/OC1A as output (for OC1A PWM)
  DDRB |= (1 << PB1);
  // If using OC1B, set PB2 as output
  // DDRB |= (1 << PB2);
  return 0;
}



void pwm_disable(){
  TCCR1A = 0;
  TCCR1B = 0;
  //TODO set pin low necessary?
}



//max duty is PWM_RESOLUTION pwm was initialized with
//note: due to fast-pwm mode pin can never be fully off or on
// there will still be one off/on cycle at 1023/0 duty
void pwm_setDutyCycle(uint16_t dutyCycle_compareValue)
{
  //clip to max value
  if (dutyCycle_compareValue > gCurrentPwmResolution)
    dutyCycle_compareValue = gCurrentPwmResolution;

 // Set duty cycle for OC1A
  OCR1A = dutyCycle_compareValue;
  // If using OCR1B, set duty cycle for OC1B
  // OCR1B = duty_cycle;
}


