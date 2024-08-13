
#include "usart.h"

#define BAUD 9600UL      // Baudrate
 
// Berechnungen
#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)   // clever runden
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))     // Reale Baudrate
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUD) // Fehler in Promille, 1000 = kein Fehler.
 
#if ((BAUD_ERROR<990) || (BAUD_ERROR>1010))
  #error Systematischer Fehler der Baudrate grösser 1% und damit zu hoch! 
#endif 

void uart_init(){
  // //UBBR = F_CPU/(9600*16) -1;
  // //UBRR0 = 51; //set Baud Rate to 9600 bit/s
  // 8MHz:
  //UBRRH = 51 >> 8;
  //UBRRL = 51 & 0xff;
  // 12MHz:
	//stdout = &uart_stdout;
  UBRRH = UBRR_VAL >> 8;
  UBRRL = UBRR_VAL & 0xff;
  UCSRB |= (1<<TXEN); // | (1<<RXEN);

}

void mputc(char c){
  while(!(UCSRA & (1<<UDRE))){
  }
  UDR = c;
}

void mputs(char * s){
  while(*s!='\0'){
    mputc(*s);
    s++;
  }
}

