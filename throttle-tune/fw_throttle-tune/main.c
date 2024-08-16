#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"
#include "a2d.h"

//====================
//====== config ======
//====================
//#define DEBUG_PASS_THROUGH //if defined duty/output voltage is set to the same as input voltage (test hardware)
//--- thresholds (adc values) ---
#define GAS_PEDAL_MAX 750
#define GAS_PEDAL_MIN 180 // actual 172

#define CONTROLLER_START 250
#define CONTROLLER_MAX 1023

//--- levels ---
#define LEVEL1_MAX_PERCENT 7
#define LEVEL2_MAX_PERCENT 18
#define LEVEL3_MAX_PERCENT 95

//--- fading ---
#define DUTY_RAMP_UP_INCREMENT 6 //duty increment per iteration (note depends on cycle duration be aware when e.g. disabling uart)
#define DUTY_MEMORY_DECREMENT 2 //track/estimate motor rpm/rolling out for quicker resume, smaller value means rolls out longer (resumes higher)



void Timer1_FastPWM_Init(void)
{
  // Mode 7 Datasheet page 98 Table 39
  //  Set Fast PWM mode with 10-bit resolution
  TCCR1A = (1 << WGM10) | (1 << WGM11); // Set WGM10 and WGM11 for 10-bit Fast PWM
  TCCR1B = (1 << WGM12) | (1 << CS10);  // Set WGM12, no prescaler

  // Set non-inverting mode
  TCCR1A |= (1 << COM1A1); // Clear OC1A on Compare Match, set OC1A at BOTTOM
  // If using OCR1B, uncomment the next line
  // TCCR1A |= (1 << COM1B1);  // Clear OC1B on Compare Match, set OC1B at BOTTOM

  // Set PB1/OC1A as output (for OC1A PWM)
  DDRB |= (1 << PB1);
  // If using OC1B, set PB2 as output
  // DDRB |= (1 << PB2);
}



void Set_PWM_Duty_Cycle(uint16_t duty_cycle)
{
  OCR1A = duty_cycle; // Set duty cycle for OC1A
                      // If using OCR1B, set duty cycle for OC1B
                      // OCR1B = duty_cycle;
}




int main(void)
{
  // init PWM
  Timer1_FastPWM_Init(); // init timer1 in fast PWM mode with 10bit resolution on pin PB1

  // init UART
  uart_init();
  uart_sendStr("hello world\n");

  // init gpio
  // speed switch input
  //PD2, pin4  speedSwitch1_slow
  DDRD &= ~(1<<PD3);
  //PD3, pin5 speedSwitch2_fast
  DDRD &= ~(1<<PD2);

  // variables
  uint16_t dutyTarget = 0;
  uint16_t duty = 0;
  uint16_t dutyMemory = 0;

  while (1)
  {
    //===== speed-switch =====
    // define max motor percentage by speed toggle switch
    uint8_t maxPercentage;
    if ((PIND & (1 << PD3)))                // switch at level 1 (slow)
      maxPercentage = LEVEL1_MAX_PERCENT;
    else if ((PIND & (1 << PD2)))           // switch at level 3 (fast)
      maxPercentage = LEVEL3_MAX_PERCENT;
    else                                  // both low: switch at level 2 (medium)
      maxPercentage = LEVEL2_MAX_PERCENT;

      //TODO: handle reverse switch input


    //===== read adc gas pedal =====
    uint16_t adcInputGasPedal = ReadChannel(1); // PC5

    // calculate gas pedal percentage
    uint16_t pedalPercent_x10;
    if (adcInputGasPedal <= GAS_PEDAL_MIN)
      pedalPercent_x10 = 0;
    else if (adcInputGasPedal >= GAS_PEDAL_MAX)
      pedalPercent_x10 = 1000;
    else
      pedalPercent_x10 = (uint32_t)(adcInputGasPedal - GAS_PEDAL_MIN) * 1000 / (GAS_PEDAL_MAX - GAS_PEDAL_MIN);


    //====== read output ======
    //(generated analog voltage via pwm)
    uint16_t adcOutput = ReadChannel(0); // PC4  - measure generated output voltage for debugging
    // calculate motor percentage from output voltage (for logging)
    uint16_t motorPercent_x10;
    if (adcOutput <= CONTROLLER_START)
      motorPercent_x10 = 0;
    else if (adcOutput >= CONTROLLER_MAX)
      motorPercent_x10 = 1000;
    else
      motorPercent_x10 = (uint32_t)(adcOutput - CONTROLLER_START) * 1000 / (CONTROLLER_MAX - CONTROLLER_START);
    // note: tis is the actual resulted percentage, when using `duty` instead of `adcOutput` you get the target motor percentage


    //===== define duty =====
    // calculate max allowed duty according to current level
    uint16_t dutyRange = (uint32_t)(CONTROLLER_MAX - CONTROLLER_START) * maxPercentage / 100;

#ifdef DEBUG_PASS_THROUGH
    // 1:1 output (no scaling)
    duty = adcInputGasPedal;
#else
    // calculate duty
    if (pedalPercent_x10 == 0)
      dutyTarget = 0;
    else if (pedalPercent_x10 >= 1000)
      dutyTarget = dutyRange + CONTROLLER_START;
    else
      dutyTarget = (uint32_t)pedalPercent_x10 * dutyRange / 1000 + CONTROLLER_START;
      // duty = (uint32_t)(maxDuty-CONTROLLER_START) *1000 / (GAS_PEDAL_MAX - GAS_PEDAL_MIN)  * (adcInputGasPedal - GAS_PEDAL_MIN) / 1000 + CONTROLLER_START; //without rounding error
#endif

    //===== manipulate duty =====
    if (dutyTarget <= duty) // ramp down instantly
      duty = dutyTarget;
    else if (dutyTarget < dutyMemory) // motor probably already turning faster than target -> set immediately
      duty = dutyTarget;
    else if (duty < dutyMemory) // estimated current motor speed less than target but still more than current - skip ramp to estimated current rpm duty
      duty = dutyMemory;
    else if (dutyTarget - duty < DUTY_RAMP_UP_INCREMENT) // set to exact target when differs less than increment
      duty = dutyTarget;
    else if (duty < CONTROLLER_START) //immediately start at controller start value
        duty = CONTROLLER_START;
    else // ramp up slowly 
      duty+=DUTY_RAMP_UP_INCREMENT; //ramp up TODO: timestamps

    // variable to estimate/track motor rpm (motor does not immediately stop when duty is reduced)
    // used for faster resume when pressing pedal again while rolling out
    if (duty > dutyMemory)
      dutyMemory = duty;
    else if (dutyMemory < DUTY_MEMORY_DECREMENT)
      dutyMemory = 0;
    else 
      dutyMemory -= DUTY_MEMORY_DECREMENT;


    //===== apply new duty =====
    Set_PWM_Duty_Cycle(duty);


    //======= logging =======
    printf("adcIn=%04d, adcOut=%04d, duty=%04d -- dutyTarget=%04d, dutyMemory=%04d, pedalPercent=%2d, motorPercent=%2d.%d (level/max=%d%%)\n",
           adcInputGasPedal, adcOutput, duty, dutyTarget, dutyMemory,
           pedalPercent_x10 / 10, motorPercent_x10 / 10, motorPercent_x10 % 10, maxPercentage);
  }
}
