#include "time.h"

#define TIMER_PRESCALER 64 // Prescaler value
#define TIMER_FREQUENCY 1000 // Timer interrupt frequency (in Hz) for 1ms ticks

// calculate compare value for interrupt that increments timestamp every 1 ms
#define TIMER_COMPARE_VALUE ((uint32_t)F_CPU / ((uint32_t)TIMER_PRESCALER * (uint32_t)TIMER_FREQUENCY) - 1)


volatile uint32_t time_ms = 0;

void time_init(void) {
    // Reset the timer counter
    TCNT2 = 0;

    // Set CTC mode (set count to 0 on compare value)
    TCCR2 |= (1 << WGM21);

    // Set necessary compare value (defined above)
    OCR2 = TIMER_COMPARE_VALUE;

    // Enable Timer2 compare match interrupt
    TIMSK |= (1 << OCIE2);

    // configure selected prescaler (todo: add support for more prescalers according to datasheet)
    #if TIMER_PRESCALER == 8
    // Start Timer2 with 8 prescaler
    TCCR2 |= (1 << CS21);
    #elif TIMER_PRESCALER == 64
    // Start Timer2 with 64 prescaler
    TCCR2 |= (1 << CS22);
    #elif TIMER_PRESCALER == 128
    TCCR2 |= (1 << CS22) | (1 << CS20);
    #elif TIMER_PRESCALER == 256
    // Start Timer2 with 256 prescaler
    TCCR2 |= (1 << CS22) | (1 << CS21);
    #else
    #error "Invalid TIMER_PRESCALER selected"
    #endif


    // Enable global interrupts
    sei();
}




// ISR that invrements timestamp every 1 ms
ISR(TIMER2_COMP_vect) {
    // Increment the millisecond counter every time the ISR is called
    time_ms++;
}




uint32_t time_get_ms(void) {
    uint32_t ms;

    // Temporarily disable interrupts to prevent inconsistencies
    cli();
    ms = time_ms;
    sei();

    return ms;
}



uint32_t time_get(void) {
    return time_get_ms();
}



inline uint32_t time_delta(uint32_t a, uint32_t b) {
    return a - b;
}



uint32_t time_msPassedSince(uint32_t timestampOld) {
    return time_delta(time_get_ms(), timestampOld);
}