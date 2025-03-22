#include "gpio.h"


void GPIO_Init(const GPIO_Pin *gpio, uint8_t direction) {
    if (direction) {
        *(gpio->ddr) |= (1 << gpio->bit);  // Set as output
    } else {
        *(gpio->ddr) &= ~(1 << gpio->bit);  // Set as input
    }
}



void GPIO_Set(const GPIO_Pin *gpio) {
    *(gpio->port) |= (1 << gpio->bit);
}

void GPIO_Clear(const GPIO_Pin *gpio) {
    *(gpio->port) &= ~(1 << gpio->bit);
}

void GPIO_Toggle(const GPIO_Pin *gpio) {
    *(gpio->port) ^= (1 << gpio->bit);
}

void GPIO_SetLevel(const GPIO_Pin *gpio, uint8_t level) {
    if (level != 0)
        GPIO_Set(gpio);
    else
        GPIO_Clear(gpio);
}

uint8_t GPIO_Read(const GPIO_Pin *gpio) {
    return (*(gpio->pin) & (1 << gpio->bit)) != 0;
}