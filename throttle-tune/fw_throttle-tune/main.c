#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"




int main(void)
{

  uart_init();

  // Define PC4 as output
  DDRC |= (1 << PC4);

  uart_sendStr("hello world\n");

int count = 0;

  while (1)
  {
    printf("count=%03d\n", count++);

    // Turn on PC4
    PORTC |= (1 << PC4);
    _delay_ms(100);
    // Turn off PC4
    PORTC &= ~(1 << PC4);
    _delay_ms(100);
  }
}
