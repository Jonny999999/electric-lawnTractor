#include <avr/io.h>
#include <util/delay.h>

int main(void)
{

  // Define PC4 as output
  DDRC |= (1 << PC4);

  while (1)
  {
    // Turn on PC4
    PORTC |= (1 << PC4);
    _delay_ms(100);
    // Turn off PC4
    PORTC &= ~(1 << PC4);
    _delay_ms(100);
  }
}
