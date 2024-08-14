
#include "usart.h"

#define BAUD 9600UL      // Baudrate
 
// Calculate required values from baudrate
#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)   // round
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))     // actual baudrate
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUD) // calculate error in thousands, 1000 = no error.
 
#if ((BAUD_ERROR<990) || (BAUD_ERROR>1010))
  #error Error in Baudrate is too large (more than 1%) -> choose another baudrate in usart.c
#endif 

// Create a file stream for UART (needed for using uart as stdout)
FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void uart_init(){
  UBRRH = UBRR_VAL >> 8;
  UBRRL = UBRR_VAL & 0xff;
  UCSRB |= (1<<TXEN); // | (1<<RXEN);

  #ifdef USE_UART_AS_STDOUT
  // Redirect stdout to UART
	stdout = &uart_output;
  #endif
}

void uart_sendChar(char c){
  while(!(UCSRA & (1<<UDRE))){
  }
  UDR = c;
}

void uart_sendStr(char * s){
  while(*s!='\0'){
    uart_sendChar(*s);
    s++;
  }
}

// Custom putchar function needed for using uart as stdout (printf output to UART)
int uart_putchar(char c, FILE *stream) {
    if (c == '\n') {
        uart_sendChar('\r');  // Send carriage return before newline
    }
    uart_sendChar(c);
    return 0;
}

