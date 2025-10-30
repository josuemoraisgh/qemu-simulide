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
};

enum simAction{
    SIM_I2C=10,
    SIM_USART,
    SIM_TIMER,
    SIM_EVENT=1<<7,
};

enum simI2cAction {
    SIM_I2C_START_READ=1,
    SIM_I2C_START_WRITE,
    SIM_I2C_START_WRITE_ASYNC,
    SIM_I2C_STOP,
    SIM_I2C_NOACK, /* Masker NACKed a receive byte.  */
    SIM_I2C_WRITE,
    SIM_I2C_READ,
    SIM_I2C_MATCH,
};

enum simUsartAction {
    SIM_USART_READ=1,
    SIM_USART_WRITE,
    SIM_USART_BAUD
};

enum simTimerAction{
    QTIMER_CR1=1,
    QTIMER_READ,
    QTIMER_WRITE,
    QTIMER_SET_FREQ,
    QTIMER_SET_LIMIT,
    QTIMER_OVF,
};

extern volatile qemuArena_t* m_arena;
// ------------------------------------------------

extern uint64_t m_timeout;

uint64_t getQemu_ps(void);

//bool waitEvent(void);
void doAction(void);
void stm32_f103c8_uart_action(void);

int simuMain( int argc, char** argv );

#endif
