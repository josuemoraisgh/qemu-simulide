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
extern volatile bool* m_qemuStep ;
extern volatile bool* m_qemuRun  ;
extern volatile bool* m_resetHard;

extern volatile bool*     m_readInput;
extern volatile uint64_t* m_nextInput;
extern volatile uint64_t* m_maskInput;
extern volatile uint64_t* m_nextState;
extern volatile uint64_t* m_nextDirec;

extern volatile uint64_t* m_nextEvent;

// ------------------------------------------------
// ------ IRQ -------------------------------------
extern qemu_irq* gpio_irq;
extern qemu_irq* dirio_irq;
extern qemu_irq* readIn_irq;
//extern qemu_irq* spi_cs_irq;

extern qemu_irq input_irq[40];

// ------------------------------------------------

extern uint64_t m_timeout;

uint64_t getQemu_ps(void);

bool waitEvent(void);

int simuMain( int argc, char** argv );

void gpioChanged( void *opaque, int pin, int state );
void dirioChanged( void *opaque, int pin, int dir );
void readInput( void *opaque, int n, int value );

#endif
