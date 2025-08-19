#include <avr/io.h>
#include "a2d.h"
uint16_t ReadChannel(uint8_t mux){
	uint16_t i;
	uint32_t result;

	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);    //frequPrescaler 64

	ADMUX = mux; //Kanal
	ADMUX |= (1<<REFS0);  //Vcc RefU
	ADMUX &= ~(1<<REFS1);  //Vcc RefU
	//ADMUX |= (1 << REFS0); //

	ADCSRA |= (1<<ADSC);   // eine ADC-Wandlung Dummy-Readout

	while ( ADCSRA & (1<<ADSC) ) {
		;     //warten bis fertig 
	}
	result = ADCW;
	result = 0; 
	for( i=0; i<4; i++ ){
    //if(mux == 1){
    //  PORTA = (1<<PA7);
    //}
		ADCSRA |= (1<<ADSC);            // eine Wandlung "single conversion"
		while ( ADCSRA & (1<<ADSC) ) {
			;   // auf Abschluss der Konvertierung warten
		}
		result += ADCW;
    //PORTA = 0;
    // XXX this is no general purpose function, because I add delay here
    //_delay_us(70);
		//result += (ADCL | (ADCH << 8));		    // Wandlungsergebnisse aufaddieren
	}
	ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren 
	result /= 4;
	return result;
}
