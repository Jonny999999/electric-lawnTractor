
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

void time_init(void);

/// @return time since startup in ms
uint32_t time_get_ms(void);
uint32_t time_get(void); // legacy alias for old code
uint32_t time_delta(uint32_t a, uint32_t b);
uint32_t time_msPassedSince(uint32_t timestampOld);