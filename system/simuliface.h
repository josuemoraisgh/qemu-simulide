/***************************************************************************
 *   Copyright (C) 2025 by Santiago González                               *
 *                                                                         *
 ***( see copyright.txt file at root folder )*******************************/

#ifndef QEMU_SIMULIFACE_H
#define QEMU_SIMULIFACE_H

#include <stdint.h>
#include <stdbool.h>

#include "qemu/typedefs.h"

// ------------------------------------------------
// -------- ARENA ---------------------------------

typedef struct qemuArena{
    uint64_t time;
    uint32_t data32;
    uint32_t mask32;
    uint16_t data16;
    uint16_t mask16;
    uint8_t  data8;
    uint8_t  mask8;
    uint8_t  state;
    uint8_t  action;
    double   ps_per_inst;
} qemuArena_t;

enum esp32Actions{
    ESP_GPIO_OUT = 1,
    ESP_GPIO_DIR,
    ESP_GPIO_IN,
    ESP_IOMUX,
    ESP_MATRIX_IN,
    ESP_MATRIX_OUT
};

enum arm32Actions{
    ARM_GPIO_OUT = 1,
    ARM_GPIO_CRx,
    ARM_GPIO_IN,
};

enum simuAction{
    SIM_I2C=10,
    SIM_USART
};

enum simuI2c_action {
    SIM_I2C_START_READ=0,
    SIM_I2C_START_WRITE,
    SIM_I2C_START_WRITE_ASYNC,
    SIM_I2C_STOP,
    SIM_I2C_NOACK, /* Masker NACKed a receive byte.  */
    SIM_I2C_WRITE,
    SIM_I2C_READ,
    SIM_I2C_MATCH,
};

enum simuUsart_action {
    SIM_USART_READ=0,
    SIM_USART_WRITE,
    SIM_USART_BAUD
};

extern volatile qemuArena_t* m_arena;
// ------------------------------------------------

extern uint64_t m_timeout;

uint64_t getQemu_ps(void);

bool waitEvent(void);

int simuMain( int argc, char** argv );

#endif
