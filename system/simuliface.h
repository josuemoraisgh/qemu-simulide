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
} qemuArena_t;

enum actions{
    GPIO_OUT = 1,
    GPIO_DIR,
    GPIO_IN,
    IOMUX,
};

extern volatile qemuArena_t* m_arena;
// ------------------------------------------------

extern uint64_t m_timeout;

uint64_t getQemu_ps(void);

bool waitEvent(void);

int simuMain( int argc, char** argv );

#endif
