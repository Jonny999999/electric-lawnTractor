#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"
#include "a2d.h"

#include "gpio.h"
#include "time.h"
#include "pwm.h"
#include "stdbool.h"

//====================
//====== config ======
//====================
#define DEBUG_UART_DEBUG_OUTPUT_ENABLED 0
#define DEBUG_PASS_THROUGH 0 //if defined duty/output voltage is set to the same as input voltage (test hardware)
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
// track/estimate motor rpm/rolling out for quicker resume (start fading up from memory instead of 0)
// 
// 5, 50ms -> 5s to decay from 100% to 0% 
#define DUTY_MEMORY_DECAY_INTERVAL_MS 50
#define DUTY_MEMORY_DECAY_STEP 5 // max 1023


//--- configure GPIO Pins ---
// buzzer
const GPIO_Pin buzzerPin = {PC3, &PORTC, &DDRC, &PINC};

// speed switch
const GPIO_Pin speedSwitch_slow = {PD2, &PORTD, &DDRD, &PIND};
const GPIO_Pin speedSwitch_fast = {PD3, &PORTD, &DDRD, &PIND};
const GPIO_Pin reverseSwitch = {PC5, &PORTC, &DDRC, &PINC};



//=============================
//==== Mode configuration =====
//=============================
typedef struct {
    const char *name;
    uint8_t maxPercent;     // percent of max possible speed applied at full throttle
    uint8_t beepCount;      // count beeped when entering this mode
    uint16_t rampUpStep;    // max duty increment per interval (max 1023)
    uint16_t rampUpIntervalMs; // note: must be larger than cycle time (consider when UART used alot)
    uint8_t pedalAverageWindowSize; // window size of the moving average to smooth pedal input (0 = disabled)
    // Future: pedal averaging, throttle curve, etc.
} kettcarConfig_t;

// global variable to track the active mode
uint8_t currentModeIndex = 0;

#define NUM_MODES 4
// have readable names for config index
#define MODE_SLOW   0
#define MODE_MEDIUM 1
#define MODE_FAST   2
#define MODE_SPORT  3
// note that speed switch also affects / limits the controller (sw connected to controller as well to limit top speed)
// so maxPercent is not directly comparable between modes (e.g. even all at 100%: already slower in slow mode)
const kettcarConfig_t modeConfigs[NUM_MODES] = {
  #define TIME_FROM_0_TO_100(rampUpStep, rampUpIntervalMs) ((CONTROLLER_MAX - CONTROLLER_START) * rampUpIntervalMs / rampUpStep)
    {
        .name = "Slow",
        .maxPercent = 15,
        .beepCount = 1,
        // 1, 15ms -> 7.5s from 0 to 100% 
        .rampUpStep = 1,
        .rampUpIntervalMs = 15,
        .pedalAverageWindowSize = 0
    },
    {
        .name = "Medium",
        .maxPercent = 35,
        .beepCount = 2,
        // 1, 10ms -> 5s from 0 to 100%
        .rampUpStep = 2,
        .rampUpIntervalMs = 10,
        .pedalAverageWindowSize = 0
    },
    {
        .name = "Fast",
        .maxPercent = 95,
        // 3, 10ms -> 1.7s from 0 to 100%
        .beepCount = 3,
        .rampUpStep = 10,
        .rampUpIntervalMs = 10,
        .pedalAverageWindowSize = 0
    },
    {
        .name = "Sport",
        .maxPercent = 100,
        .beepCount = 5,
        // ramp disabled - always set to target immediately
        .rampUpStep = 1024,
        .rampUpIntervalMs = 0,
        .pedalAverageWindowSize = 100
    }
};



// helper function to beep for certain count
void beep(uint8_t count){
  static const uint32_t msOn = 80;
  static const uint32_t msOff = 70;
  for (int i = 1; i <= count; i++)
  {
    GPIO_Set(&buzzerPin);
    _delay_ms(msOn);
    GPIO_Clear(&buzzerPin);
    if (i < count) // prevent unnecessary delay after last beep
      _delay_ms(msOff);
  }
}



// function that evaluates if switch to sport mode sequence is entered and switches to sport mode
#define MODE_SPORT_ACTIVATION_TIME_WINDOW_MS 1000
#define MODE_SPORT_ACTIVATION_REVERSE_SW_EDGE_COUNT 3
void checkForSportModeActivation(void) {
    static uint8_t revToggleCount = 0;
    static bool lastRevState = false;
    static uint32_t timestamp_firstToggle = 0;

    if (!GPIO_Read(&speedSwitch_fast)) {
        // Not in fast mode, reset sport toggle tracking
        revToggleCount = 0;
        timestamp_firstToggle = 0;
        lastRevState = GPIO_Read(&reverseSwitch);
        return;
    }

    bool currentRevState = GPIO_Read(&reverseSwitch);

    if (!lastRevState && currentRevState) {
        // Rising edge detected
        uint32_t now = time_get_ms();

        if (timestamp_firstToggle == 0 || now - timestamp_firstToggle > MODE_SPORT_ACTIVATION_TIME_WINDOW_MS) {
            // Restart tracking
            revToggleCount = 1;
            timestamp_firstToggle = now;
        } else {
            revToggleCount++;
            if (revToggleCount >= MODE_SPORT_ACTIVATION_REVERSE_SW_EDGE_COUNT) {
                currentModeIndex = MODE_SPORT;
                revToggleCount = 0;
                timestamp_firstToggle = 0;
                beep(modeConfigs[MODE_SPORT].beepCount);
                printf("\nSwitched to mode %s\n", modeConfigs[currentModeIndex].name);
            }
        }
    }

    lastRevState = currentRevState;
}



// function that updates the currently selected config depending on speed switch position
#define MODE_CONFIRM_DELAY_MS 500
void handleSlowMediumFastModeSwitch(void) {
    static uint8_t lastStableIndex = 255;
    static uint32_t timestamp_lastChange = 0;

    if (GPIO_Read(&speedSwitch_fast) && currentModeIndex == MODE_SPORT) {
        // Dont override sport mode
        return;
    }

    bool slow = GPIO_Read(&speedSwitch_slow);
    bool fast = GPIO_Read(&speedSwitch_fast);
    uint8_t newIndex;
    if (slow && fast) {
        // Invalid combo, skip update
        return;
    } else if (slow) {
        newIndex = MODE_SLOW; // Slow
    } else if (fast) {
        newIndex = MODE_FAST; // Fast
    } else {
        newIndex = MODE_MEDIUM; // Medium
    }

    if (newIndex != currentModeIndex) {
        currentModeIndex = newIndex;
        timestamp_lastChange = time_get_ms();
        return;
    }

    // Stable long enough for confirmation beep?
    if (currentModeIndex != lastStableIndex && time_get_ms() - timestamp_lastChange > MODE_CONFIRM_DELAY_MS) {
        beep(modeConfigs[currentModeIndex].beepCount);
        printf("\nSwitched to mode %s\n", modeConfigs[currentModeIndex].name);
        lastStableIndex = currentModeIndex;
    }
}




// Function to calculate the moving average of the last N pedal values
#define PEDAL_SMOOTHING_MAX_WINDOW_SIZE 512
uint16_t GetSmoothedPedalInput(uint16_t newPedalValue, uint8_t windowSize) {
  // Array to store past pedal values
  static uint16_t pedalHistory[PEDAL_SMOOTHING_MAX_WINDOW_SIZE] = {0};
  static uint8_t pedalHistoryIndex = 0;

  // return same value when window is 0 or 1 aka disabled
  if (windowSize < 2)
    return newPedalValue;

  // Add new value to the history array
  pedalHistory[pedalHistoryIndex] = newPedalValue;
  
  // Move to the next index, wrapping around the array
  pedalHistoryIndex = (pedalHistoryIndex + 1) % windowSize;
  
  // Calculate the sum of the last windowSize values
  uint32_t sum = 0;
  for (uint8_t i = 0; i < windowSize; i++) {
      sum += pedalHistory[i];
  }

  // Return the average 
  return sum / windowSize;
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
  GPIO_Init(&speedSwitch_slow, 0);
  GPIO_Init(&speedSwitch_fast, 0);
  GPIO_Init(&reverseSwitch, 0);


  // --- variables ---
  uint16_t dutyTarget = 0;
  uint16_t duty = 0;
  uint16_t dutyMemory = 0;

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
    checkForSportModeActivation();
    handleSlowMediumFastModeSwitch();
    const kettcarConfig_t *currentConfig = &modeConfigs[currentModeIndex];

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

    // smooth out pedal input (remove peaks)
    pedalPercent_x10 = GetSmoothedPedalInput(pedalPercent_x10, currentConfig->pedalAverageWindowSize);

    //======================================
    //====== read back output (debug) ======
    //======================================
#if DEBUG_UART_DEBUG_OUTPUT_ENABLED
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
#endif


    //==============================
    //===== define target duty =====
    //==============================
    // calculate max allowed duty according to current level
    uint16_t dutyRange = (uint32_t)(CONTROLLER_MAX - CONTROLLER_START) * currentConfig->maxPercent / 100;

#if DEBUG_PASS_THROUGH
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
    else if (dutyTarget - duty < currentConfig->rampUpStep) // set to exact target when differs less than increment
      duty = dutyTarget;
    else if (duty < CONTROLLER_START) //immediately start at controller start value
        duty = CONTROLLER_START;
    else { // ramp up slowly 
      if (time_msPassedSince(timestamp_lastRampUpdate) >= currentConfig->rampUpIntervalMs) {
          duty += currentConfig->rampUpStep;
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
#if DEBUG_UART_DEBUG_OUTPUT_ENABLED
    printf("adcIn=%04d, adcOut=%04d, duty=%04d -- dutyTarget=%04d, dutyMemory=%04d, pedalPercent=%2d, motorPercent=%2d.%d (level/max=%d%%)\n",
           adcInputGasPedal, adcOutput, duty, dutyTarget, dutyMemory,
           pedalPercent_x10 / 10, motorPercent_x10 / 10, motorPercent_x10 % 10, currentConfig->maxPercent);
#endif

  } // end while(1)

} // end main()
