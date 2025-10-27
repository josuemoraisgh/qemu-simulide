/***************************************************************************
 *   Copyright (C) 2025 by Santiago González                               *
 *                                                                         *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#ifdef __linux__
#include <sys/mman.h>
#include <sys/shm.h>
//#elif defined(_WIN32)
//#include <windows.h>
#endif

#include "simuliface.h"

#include "qemu/osdep.h"
#include "qemu-main.h"
#include "qemu/timer.h"
#include "sysemu/runstate.h"
#include "sysemu/sysemu.h"
#include "sysemu/cpu-timers.h"
#include "hw/irq.h"

// ------------------------------------------------
// -------- ARENA ---------------------------------

volatile qemuArena_t* m_arena = NULL;

// ------------------------------------------------

uint64_t m_timeout;
//uint64_t m_resetEvent;
//uint64_t m_ClkPeriod;
uint64_t m_lastQemuTime;
//bool m_running;

QEMUTimer* qtimer;


uint64_t getQemu_ps(void)
{
    uint64_t qemuTime = icount_get_ps(); //qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL ); // ns //icount_get_ps; //
    //qemuTime *= 1000;
    return qemuTime;
}

static void getNextEvent(void)
{
    while( !m_arena->qemuTime ) // Wait for SimuliDE
    {
        if( m_timeout++ > 1e9 ) break; // Terminate process if timed out
    }
    m_timeout = 0;

    uint64_t nextTime_ns = m_arena->qemuTime/1000;
    m_arena->qemuTime = 0;

    if( m_lastQemuTime != nextTime_ns )
    {
        //printf("Qemu: config timer at %lu\n", nextTime ); fflush( stdout );
        m_lastQemuTime = nextTime_ns;
        timer_mod_ns( qtimer, nextTime_ns );
    }
    //printf("Qemu: Next time %lu\n", m_lastQemuTime*1000 ); fflush( stdout );
}

void doAction(void)
{
    //printf("Qemu: doAction at time %lu\n", getQemu_ps() ); fflush( stdout );
    m_arena->simuTime = getQemu_ps();

    while( m_arena->simuTime )  // Wait for SimulIDE to execute action
    {
        if( m_arena->qemuAction )
        {
            switch( m_arena->qemuAction )
            {
            case SIM_I2C: break;
            case SIM_USART: stm32_f103c8_uart_action(); break;
            case SIM_TIMER: stm32_f103c8_timer_action(); break;
            default: break;
            }
            m_arena->qemuAction = 0;
            //while( !m_arena->qemuAction ){;}
        }
        if( m_timeout++ > 1e9 ) break; // Terminate process if timed out
    }
    m_timeout = 0;

    //printf("Qemu: qemuAction %u %u\n", m_arena->qemuAction, m_arena->data8 );
    //printf("      at time %lu\n", getQemu_ps() ); fflush( stdout );

    getNextEvent();
}

static void simu_event( void* opaque )
{
    //printf("Qemu: simu_event at %lu\n", getQemu_ps() ); fflush( stdout );
    if( !m_arena->running ) return;

    m_arena->simuAction = SIM_EVENT;
    doAction();
}

int simuMain( int argc, char** argv )
{
    const int   shMemSize = sizeof( qemuArena_t );
    const char* shMemKey;

    if( argc > 2 ) // Check if there are any arguments
    {
        shMemKey = argv[1];
        argv = &argv[2];
        argc -= 2;
    } else {
        printf("Qemu Error: No arguments provided.\n");
        return 1;
    }

    void* arena = NULL;

#ifdef __linux__
    int shMemId = shm_open( shMemKey, O_RDWR, 0666 ); // Open the shared memory object
    if( shMemId == -1 )
    {
        printf("Qemu: Error opening arena: %s\n", shMemKey );
        return 1;
    }
    else printf("Qemu: arena ok: %s\n", shMemKey );
    arena = mmap( 0, shMemSize, PROT_READ | PROT_WRITE, MAP_SHARED, shMemId, 0);
#elif defined(_WIN32)
    HANDLE hMapFile = OpenFileMapping( FILE_MAP_ALL_ACCESS,  FALSE, shMemKey );

    if( hMapFile == NULL ) {
        //std::cerr << "Could not create file mapping object: " << GetLastError() << std::endl;
        return 1;
    }
    arena = MapViewOfFile( hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, shMemSize );
#endif

    if( !arena )
    {
        printf("Qemu: Error mapping arena\n"); fflush( stdout );
        return 1;
    }
    else printf("Qemu: arena mapped %i bytes\n", shMemSize );

    //------------------------------------------------------------------

    m_arena = (qemuArena_t*)arena;

    //------------------------------------------------------------------

    //m_ClkPeriod = 100*1000; // 100 us              //*1000; // ~1 ms

    printf("-----------------------------------\n");
    for( int i=0; i<argc; i++)
    {
        printf( "%s",argv[i] );
        if( !(i&1) ) printf("\n");
        else         printf(" ");
    }
    printf("-----------------------------------\n");
    fflush( stdout );

    qemu_init( argc, argv );

    qtimer = (QEMUTimer*)malloc( sizeof(QEMUTimer) );
    timer_init_full( qtimer, NULL, QEMU_CLOCK_VIRTUAL, 1, 0, simu_event, NULL );

    m_lastQemuTime = 0; //m_resetEvent;
    m_arena->running = true;

    printf("Qemu: initialized\n" );fflush( stdout );

    while( m_arena->qemuTime == 0 )  // Wait for simulide to set next time
    {
        m_timeout += 1;
        if( m_timeout > 1e9 ) break; // Terminate loop if timed out
    }
    m_timeout = 0;
    getNextEvent();

    printf("Qemu: starting main loop\n");fflush( stdout );
    int status = qemu_main_loop();
    qemu_cleanup( status );

#ifdef __linux__
    munmap( arena, shMemSize ); // Un-map shared memory
#elif defined(_WIN32)
    UnmapViewOfFile( arena );
    CloseHandle( hMapFile );
#endif

    printf("Qemu: process finished\n");fflush( stdout );

    return 0;
}
