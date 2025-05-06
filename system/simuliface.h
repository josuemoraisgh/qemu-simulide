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
    bool     qemuRun;
    bool     readInput;
    uint64_t nextInput;
    uint64_t maskInput;
    uint64_t nextState;
    uint64_t nextDirec;
    uint64_t nextEvent;
    uint64_t pullUps;
    uint64_t puChanged;
    uint64_t pullDown;
    uint64_t pdChanged;
    uint64_t inputEn;
    uint64_t ieChanged;
} qemuArena_t;

extern volatile qemuArena_t* m_arena;
// ------------------------------------------------

extern uint64_t m_timeout;

uint64_t getQemu_ps(void);

bool waitEvent(void);

int simuMain( int argc, char** argv );

#endif
