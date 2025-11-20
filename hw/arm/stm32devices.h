/***************************************************************************
 *   Copyright (C) 2025 by Santiago González                               *
 *                                                                         *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include <stdint.h>
//#include <stdio.h>

#ifndef STM32_DEVICES_H
#define STM32_DEVICES_H

typedef struct STM32Periph{
    uint8_t gpio;
    uint8_t uart;
    uint8_t i2c;
    uint8_t spi;
    uint8_t timer;
    uint8_t adc;
} STM32Periph_t;

typedef struct STM32Model{
    uint32_t oscFreq;
    uint16_t flashSize;
    uint8_t ramSize;
    uint8_t gpioN;
    uint8_t uartN;
    uint8_t i2cN;
    uint8_t spiN;
    uint8_t timerM;
    uint8_t adcSize;
} STM32Model_t;

STM32Periph_t stm32f101x46  = { .gpio=4, .uart=2, .i2c=1, .spi=1, .timer=0b00000110, .adc=1 };
STM32Periph_t stm32f101x8b  = { .gpio=5, .uart=3, .i2c=2, .spi=2, .timer=0b00001110, .adc=1 };
STM32Periph_t stm32f101xcde = { .gpio=7, .uart=5, .i2c=2, .spi=3, .timer=0b01111110, .adc=1 };

STM32Periph_t stm32f103x46  = { .gpio=4, .uart=2, .i2c=1, .spi=1, .timer=0b00000111, .adc=2 };
STM32Periph_t stm32f103x8b  = { .gpio=5, .uart=3, .i2c=2, .spi=2, .timer=0b00001111, .adc=2 };
STM32Periph_t stm32f103xcde = { .gpio=7, .uart=5, .i2c=2, .spi=3, .timer=0b11111111, .adc=3 };

// T=0, C=1, R=2, V=3, Z=4

static STM32Model_t stm32_get_model( uint32_t id )
{
    //printf("stm32_get_model %i\n", id); fflush( stdout );
    uint32_t oscFreq ;
    uint16_t flashSize;
    uint8_t  ramSize;
    STM32Periph_t periph = stm32f103x8b;

    uint8_t family = (id & 0xFF0000) >> 16;
    uint8_t variant = id & 0x0000FF;
    switch( family )
    {
        case 1: // F101
        {
            oscFreq = 4500000; break;
            switch( variant ) // 4=0, 6=1, 8=2, B=3, C=4, D=5, E=6, F=7, G=8
            {
            case 0: flashSize =  16; ramSize =  6; periph = stm32f101x46;  break; // 4
            case 1: flashSize =  32; ramSize = 10; periph = stm32f101x46;  break; // 6
            case 2: flashSize =  64; ramSize = 20; periph = stm32f101x8b;  break; // 8
            case 3: flashSize = 128; ramSize = 20; periph = stm32f101x8b;  break; // B
            case 4: flashSize = 256; ramSize = 48; periph = stm32f101xcde; break; // C
            case 5: flashSize = 384; ramSize = 64; periph = stm32f101xcde; break; // D
            case 6: flashSize = 512; ramSize = 64; periph = stm32f101xcde; break; // E

            default: flashSize = 128; ramSize = 20; periph = stm32f101x8b; break; // B
            }
        }break;
        case 2: // F102
        {
            oscFreq = 6000000; break;
            switch( variant ) // 4=0, 6=1, 8=2, B=3, C=4, D=5, E=6, F=7, G=8
            {
            case 0: flashSize =  16; ramSize =  6; periph = stm32f101x46;  break; // 4
            case 1: flashSize =  32; ramSize = 10; periph = stm32f101x46;  break; // 6
            case 2: flashSize =  64; ramSize = 20; periph = stm32f101x8b;  break; // 8
            case 3: flashSize = 128; ramSize = 20; periph = stm32f101x8b;  break; // B

            default: flashSize = 128; ramSize = 20; periph = stm32f101x8b; break; // B
            }
        }break;
        case 3: // F103
        {
            oscFreq = 8000000; break;
            switch( variant ) // 4=0, 6=1, 8=2, B=3, C=4, D=5, E=6, F=7, G=8
            {
            case 0: flashSize =  16; ramSize =  6; periph = stm32f103x46;  break; // 4
            case 1: flashSize =  32; ramSize = 10; periph = stm32f103x46;  break; // 6
            case 2: flashSize =  64; ramSize = 20; periph = stm32f103x8b;  break; // 8
            case 3: flashSize = 128; ramSize = 20; periph = stm32f103x8b;  break; // B
            case 4: flashSize = 256; ramSize = 48; periph = stm32f103xcde; break; // C
            case 5: flashSize = 384; ramSize = 64; periph = stm32f103xcde; break; // D
            case 6: flashSize = 512; ramSize = 64; periph = stm32f103xcde; break; // E

            default: flashSize = 128; ramSize = 20; periph = stm32f103x8b; break; // B
            }
        } break;
        default:
            oscFreq = 8000000; flashSize = 128; ramSize = 20; periph = stm32f103x8b; break; // F103x8
    }


    STM32Model_t model = { oscFreq, flashSize, ramSize, periph.gpio, periph.uart
                          , periph.i2c, periph.spi, periph.timer, periph.adc };
    return model;
}
#endif /* STM32_DEVICES_H */
