#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"
#include "a2d.h"





void Timer1_FastPWM_Init(void) {
  //Mode 7 Datasheet page 98 Table 39
    // Set Fast PWM mode with 10-bit resolution
    TCCR1A = (1 << WGM10) | (1 << WGM11); // Set WGM10 and WGM11 for 10-bit Fast PWM
    TCCR1B = (1 << WGM12) | (1 << CS10);  // Set WGM12, no prescaler

    // Set non-inverting mode
    TCCR1A |= (1 << COM1A1);  // Clear OC1A on Compare Match, set OC1A at BOTTOM
    // If using OCR1B, uncomment the next line
    // TCCR1A |= (1 << COM1B1);  // Clear OC1B on Compare Match, set OC1B at BOTTOM

    // Set PB1/OC1A as output (for OC1A PWM)
    DDRB |= (1 << PB1);
    // If using OC1B, set PB2 as output
    // DDRB |= (1 << PB2);
}

void Set_PWM_Duty_Cycle(uint16_t duty_cycle) {
    OCR1A = duty_cycle;  // Set duty cycle for OC1A
    // If using OCR1B, set duty cycle for OC1B
    // OCR1B = duty_cycle;
}





// config
#define GAS_PEDAL_MAX 750
#define GAS_PEDAL_MIN 200 // actual 172

#define CONTROLLER_START 250
#define CONTROLLER_MAX 1023

#define LEVEL1_MAX_PERCENT 6


// calculations
#define LEVEL1_MAX_DUTY (CONTROLLER_MAX - CONTROLLER_START) * LEVEL1_MAX_PERCENT / 100  + CONTROLLER_START


int main(void)
{
  Timer1_FastPWM_Init(); // init timer1 in fast PWM mode with 10bit resolution on pin PB1

  uart_init();

  // Define PC3 as output
  DDRC |= (1 << PC3);

  uart_sendStr("hello world\n");

  uint16_t duty = 0;

  while (1)
  {
    // read adc
    // max: 1023 = 5V
    // read gas pedal
    uint16_t adcInputGasPedal = ReadChannel(1); //PC5
    if (adcInputGasPedal < GAS_PEDAL_MIN) adcInputGasPedal = GAS_PEDAL_MIN;
    if (adcInputGasPedal > GAS_PEDAL_MAX) adcInputGasPedal = GAS_PEDAL_MAX;
    // read output (generated analog voltage via pwm)
    uint16_t adcOutput = ReadChannel(0); //PC4  - measure generated output voltage for debugging

    // calculate percentages for logging
    uint16_t pedalPercent = (uint32_t)(adcInputGasPedal-GAS_PEDAL_MIN) * 100 / (GAS_PEDAL_MAX - GAS_PEDAL_MIN);

    uint8_t motorPercent;
    //uint8_t motorPercent = (duty-CONTROLLER_START) * 100 / (CONTROLLER_MAX - CONTROLLER_START);
    if (adcOutput < CONTROLLER_START) motorPercent = 0;
    else motorPercent = (adcOutput-CONTROLLER_START) * 100 / (CONTROLLER_MAX - CONTROLLER_START);

    // logging
    printf("adcIn=%04d, adcOut=%04d, duty=%04d -- pedalPercent=%2d, motorPercent=%2d\n", 
    adcInputGasPedal, adcOutput, duty, pedalPercent, motorPercent);



    // calculate duty
    //TODO limit to range 0-1023   in case of negative = large number?!
    duty = (uint32_t)(LEVEL1_MAX_DUTY-CONTROLLER_START) *1000 / (GAS_PEDAL_MAX - GAS_PEDAL_MIN)  * (adcInputGasPedal - GAS_PEDAL_MIN) / 1000 + CONTROLLER_START;

    // 1:1 output (no scaling)
    //duty = adcInputGasPedal;

  //turn off completely when below controller start threshold
    if (duty <= CONTROLLER_START) duty = 0;

  // apply new duty
    Set_PWM_Duty_Cycle(duty);

    // blink led
    // Turn on PC4
    //PORTC |= (1 << PC3);
    //_delay_ms(50);
    //// Turn off PC4
    //PORTC &= ~(1 << PC3);
    //_delay_ms(50);
  }
}
