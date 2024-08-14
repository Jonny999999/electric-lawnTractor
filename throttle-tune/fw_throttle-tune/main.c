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







int main(void)
{
  Timer1_FastPWM_Init(); // Initialize Timer1 in Fast PWM mode with 10Bit resolution on Pin PB1

  uart_init();

  // Define PC4 as output
  DDRC |= (1 << PC4);

  uart_sendStr("hello world\n");

  uint16_t duty = 0;

  while (1)
  {
    // read adc
    // max: 1023 = 5V
    uint16_t adcInput = ReadChannel(5); //PC5
    uint16_t adcOutput = ReadChannel(4); //PC4  - measure generated output voltage for debugging

    // logging
    printf("adcIn=%04d, adcOut=%04d, duty=%04d\n", adcInput, adcOutput, duty);

    // test pwm
    duty += 50; // increment
    duty %= 1023; //limit to range
    Set_PWM_Duty_Cycle(duty); // Example: Set duty cycle to 50%

    // blink led
    // Turn on PC4
    PORTC |= (1 << PC4);
    _delay_ms(50);
    // Turn off PC4
    PORTC &= ~(1 << PC4);
    _delay_ms(50);
  }
}
