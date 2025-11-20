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
    uint64_t simuTime;    // in ps
    uint64_t qemuTime;    // in ps
    uint32_t data32;
    uint32_t mask32;
    uint16_t data16;
    uint16_t mask16;
    uint8_t  data8;
    uint8_t  mask8;
    uint8_t  simuAction;
    uint8_t  qemuAction;
    double   ps_per_inst;
    bool     running;
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
    ARM_ALT_OUT,
    ARM_REMAP
};

enum simAction{
    SIM_I2C=10,
    SIM_SPI,
    SIM_USART,
    SIM_TIMER,
    SIM_GPIO_IN,
    SIM_EVENT=1<<7,
};

extern volatile qemuArena_t* m_arena;
// ------------------------------------------------

extern uint64_t m_timeout;

uint64_t getQemu_ps(void);

//bool waitEvent(void);
void doAction(void);

int simuMain( int argc, char** argv );

#endif
