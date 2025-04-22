#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"
#include "a2d.h"

#include "gpio.h"
#include "time.h"
#include "pwm.h"

//====================
//====== config ======
//====================
//#define DEBUG_PASS_THROUGH //if defined duty/output voltage is set to the same as input voltage (test hardware)
//--- thresholds (adc values) ---
#define GAS_PEDAL_MAX 670 // actual 660 no force / 697 with force
#define GAS_PEDAL_MIN 175 // actual 171 - note weird behaviour: when pressing decreases to 160 first then increases to MAX 

#define CONTROLLER_START 250  // 250 starts, 246 stops (TODO: start higher?)
#define CONTROLLER_MAX 600

//--- levels ---
#define LEVEL1_MAX_PERCENT 10
#define LEVEL2_MAX_PERCENT 30
#define LEVEL3_MAX_PERCENT 100

// TODO handle REVERSE speeds separately

//--- fading ---
// ramp up
#define DUTY_RAMP_UP_INTERVAL_MS 10 // Time between duty increments
#define DUTY_RAMP_UP_STEP 6         // Amount of duty to add per interval
// track/estimate motor rpm/rolling out for quicker resume
#define DUTY_MEMORY_DECAY_INTERVAL_MS 50
#define DUTY_MEMORY_DECAY_STEP 2


//--- configure GPIO Pins ---
// buzzer
const GPIO_Pin buzzerPin = {PC3, &PORTC, &DDRC, &PINC};

// speed switch
const GPIO_Pin speedSwitch1_slow = {PD2, &PORTD, &DDRD, &PIND};
const GPIO_Pin speedSwitch2_fast = {PD3, &PORTD, &DDRD, &PIND};




// helper function to beep for certain count
void beep(uint8_t count){
  static const uint32_t msOn = 100;
  static const uint32_t msOff = 100;
  for (int i = 1; i <= count; i++)
  {
    GPIO_Set(&buzzerPin);
    _delay_ms(msOn);
    GPIO_Clear(&buzzerPin);
    if (i < count) // prevent unnecessary delay after last beep
      _delay_ms(msOff);
  }
}


// helper function to obtain desired max duty percentage according to speed switch position
uint8_t getMaxPercentageFromSpeedSwitches(){
    if (GPIO_Read(&speedSwitch1_slow))                // switch at level 1 (slow)
      return LEVEL1_MAX_PERCENT;
    else if (GPIO_Read(&speedSwitch2_fast))           // switch at level 3 (fast)
       return LEVEL3_MAX_PERCENT;
    else                                  // both low: switch at level 2 (medium)
       return LEVEL2_MAX_PERCENT;
}



int main(void)
{
  // init custom time functions
  // (tracking timestamp in ms since startup in a variable using timer+ISR)
  time_init();

  // init PWM
  pwm_initFastPwmTimer1(1023, 1); //10 bit res, no prescaler

  // init UART
  uart_init();
  uart_sendStr("hello world\n");


  // --- init GPIO pins ---
  // init output
  GPIO_Init(&buzzerPin, 1); // init buzzer as output
  // init inputs
  GPIO_Init(&speedSwitch1_slow, 0);
  GPIO_Init(&speedSwitch2_fast, 0);


  // --- variables ---
  uint16_t dutyTarget = 0;
  uint16_t duty = 0;
  uint16_t dutyMemory = 0;

  uint8_t maxPercentage = getMaxPercentageFromSpeedSwitches();
  uint8_t maxPercentagePrevious = maxPercentage;

  // fading
  static uint32_t timestamp_lastRampUpdate = 0;
  static uint32_t timestamp_lastDutyMemoryUpdate = 0;

  
  // beep at startup:
  beep(3);



  while (1)
  {
    //========================
    //===== speed-switch =====
    //========================
    // define max motor percentage by speed toggle switch
    maxPercentage = getMaxPercentageFromSpeedSwitches();

    // beep if speed max level changed
    if (maxPercentagePrevious != maxPercentage){
      maxPercentagePrevious = maxPercentage;
      switch (maxPercentage){
        case LEVEL1_MAX_PERCENT:
          beep(1);
          break;
        case LEVEL2_MAX_PERCENT:
          beep(2);
          break;
        case LEVEL3_MAX_PERCENT:
          beep(3);
          break;
        default:
          break;
      }
    }

    //TODO: handle reverse switch input


    //======================================
    //===== read + interpret gas-pedal =====
    //======================================
    uint16_t adcInputGasPedal = ReadChannel(1); // PC5

    // calculate gas pedal percentage
    uint16_t pedalPercent_x10;
    if (adcInputGasPedal <= GAS_PEDAL_MIN)
      pedalPercent_x10 = 0;
    else if (adcInputGasPedal >= GAS_PEDAL_MAX)
      pedalPercent_x10 = 1000;
    else
      pedalPercent_x10 = (uint32_t)(adcInputGasPedal - GAS_PEDAL_MIN) * 1000 / (GAS_PEDAL_MAX - GAS_PEDAL_MIN);


    //======================================
    //====== read back output (debug) ======
    //======================================
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


    //==============================
    //===== define target duty =====
    //==============================
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


    //==================================
    //===== manipulate actual duty =====
    //==================================
    // Fade up
    if (dutyTarget <= duty) // ramp down instantly
      duty = dutyTarget;
    else if (dutyTarget < dutyMemory) // motor probably already turning faster than target -> set immediately
      duty = dutyMemory;
    else if (duty < dutyMemory) // estimated current motor speed less than target but still more than current - skip ramping to estimated current rpm duty again
      duty = dutyMemory;
    else if (dutyTarget - duty < DUTY_RAMP_UP_STEP) // set to exact target when differs less than increment
      duty = dutyTarget;
    else if (duty < CONTROLLER_START) //immediately start at controller start value
        duty = CONTROLLER_START;
    else { // ramp up slowly 
      if (time_msPassedSince(timestamp_lastRampUpdate) >= DUTY_RAMP_UP_INTERVAL_MS) {
          duty += DUTY_RAMP_UP_STEP;
          timestamp_lastRampUpdate = time_get_ms();
      }
  }

    // variable to estimate/track motor rpm (motor does not immediately stop when duty is reduced)
    // used for faster resume when pressing pedal again while rolling out
  if (duty > dutyMemory) { // update immediately if current duty is larger
    dutyMemory = duty;
    // decrease slowly when below (running out)
  } else if (time_msPassedSince(timestamp_lastDutyMemoryUpdate) >= DUTY_MEMORY_DECAY_INTERVAL_MS) {
    // TODO: decrease more/less depending on current duty?
      if (dutyMemory < DUTY_MEMORY_DECAY_STEP)
          dutyMemory = 0;
      else
          dutyMemory -= DUTY_MEMORY_DECAY_STEP;
      timestamp_lastDutyMemoryUpdate = time_get_ms();
  }


    //==========================
    //===== apply new duty =====
    //==========================
    pwm_setDutyCycle(duty);


    //=======================
    //======= logging =======
    //=======================
    printf("adcIn=%04d, adcOut=%04d, duty=%04d -- dutyTarget=%04d, dutyMemory=%04d, pedalPercent=%2d, motorPercent=%2d.%d (level/max=%d%%)\n",
           adcInputGasPedal, adcOutput, duty, dutyTarget, dutyMemory,
           pedalPercent_x10 / 10, motorPercent_x10 / 10, motorPercent_x10 % 10, maxPercentage);
  }
}
