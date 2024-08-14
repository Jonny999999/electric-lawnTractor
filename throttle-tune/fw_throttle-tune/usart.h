#include <avr/io.h>
#include <stdio.h>

// If defined e.g. printf() output goes to uart
#define USE_UART_AS_STDOUT


void uart_init(void);
void uart_sendChar(char c);
void uart_sendStr(char * s);
// Custom putchar function needed for using uart as stdout (printf output to UART)
int uart_putchar(char c, FILE *stream);
