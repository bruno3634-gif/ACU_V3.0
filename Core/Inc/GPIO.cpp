/*
 * GPIO.cpp
 *
 *  Created on: Mar 7, 2026
 *      Author: bruno
 */

#include <GPIO.hpp>

GPIO::GPIO(GPIO_TypeDef* port, uint16_t pin) : _port(port), _pin(pin) {}

void GPIO::initOutput(uint32_t pull, uint32_t speed) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = _pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-Pull por defeito
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = speed;

    HAL_GPIO_Init(_port, &GPIO_InitStruct);
}

void GPIO::initInput(uint32_t pull) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = _pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = pull;

    HAL_GPIO_Init(_port, &GPIO_InitStruct);
}

void GPIO::write(GPIO_PinState state) {
    HAL_GPIO_WritePin(_port, _pin, state);
}

void GPIO::toggle() {
    HAL_GPIO_TogglePin(_port, _pin);
}

bool GPIO::read() const {
    return HAL_GPIO_ReadPin(_port, _pin) == GPIO_PIN_SET;
}

