/*
 * GPIO.h
 *
 *  Created on: Mar 7, 2026
 *      Author: bruno
 */

#ifndef INC_GPIO_HPP_
#define INC_GPIO_HPP_

#include "main.h" // Inclui os headers da HAL configurados no STM32CubeIDE

class GPIO {
public:
    // Construtor guarda a referência do hardware
    GPIO(GPIO_TypeDef* port, uint16_t pin);

    // Configuração simplificada
    void initOutput(uint32_t pull = GPIO_NOPULL, uint32_t speed = GPIO_SPEED_FREQ_LOW);
    void initInput(uint32_t pull = GPIO_NOPULL);

    // Métodos de controlo (Encapsulam a HAL)
    void write(GPIO_PinState state);
    void toggle();
    bool read() const;

    // Atalhos úteis
    void set()   { write(GPIO_PIN_SET); }
    void reset() { write(GPIO_PIN_RESET); }

private:
    GPIO_TypeDef* _port;
    uint16_t      _pin;
};

#endif /* INC_GPIO_HPP_ */
