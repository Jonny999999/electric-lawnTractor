#include <avr/io.h>
#include <stdio.h>


void uart_init(void);
void mputc(char c);
void mputs(char * s);
//static FILE uart_stdout = FDEV_SETUP_STREAM(mputc, 0, _FDEV_SETUP_WRITE);
