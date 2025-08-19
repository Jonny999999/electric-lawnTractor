#pragma once

#include <avr/io.h>


#define GPIO_PIN(bit, port, ddr, pin)  { bit, &port, &ddr, &pin }

typedef struct {
    uint8_t bit;
    volatile uint8_t *port;
    volatile uint8_t *ddr;
    volatile uint8_t *pin;
} GPIO_Pin;

//create instance:
//GPIO_Pin relay = { PD3, &PORTD, &DDRD, &PIND };
//using macro:
//GPIO_Pin relay = GPIO_PIN(PD3, PORTD, DDRD, PIND);


void GPIO_Init(const GPIO_Pin *gpio, uint8_t direction);


void GPIO_Set(const GPIO_Pin *gpio);

void GPIO_Clear(const GPIO_Pin *gpio);

void GPIO_Toggle(const GPIO_Pin *gpio);

void GPIO_SetLevel(const GPIO_Pin *gpio, uint8_t level);

uint8_t GPIO_Read(const GPIO_Pin *gpio);