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
#include "hw/irq.h"

// ------------------------------------------------
// -------- ARENA ---------------------------------

volatile qemuArena_t* m_arena = NULL;

// ------------------------------------------------

uint64_t m_timeout;
uint64_t m_resetEvent;
uint64_t m_ClkPeriod;
uint64_t m_lastQemuTime;

QEMUTimer* qtimer;

uint64_t getQemu_ps(void)
{
    uint64_t qemuTime = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL ); // ns
    qemuTime *= 1000;
    return qemuTime-m_resetEvent;
}

void waitForTime(void)
{
    //printf("Qemu: waiting for Time \n"); fflush( stdout );
    while( m_arena->qemuTime == 0 )  // Wait for simulide to set next time
    {
        m_timeout += 1;
        if( m_timeout > 1e9 ) break; // Terminate loop if timed out
    }
    m_timeout = 0;

    uint64_t nextTime = m_arena->qemuTime/1000;
    /// if( nextTime > 1e9 )
    ///     nextTime = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL ) + m_ClkPeriod;

    if( m_lastQemuTime != nextTime )
    {
        m_lastQemuTime = nextTime;
        timer_mod_ns( qtimer, nextTime );

    }
    //printf("Qemu: waitForTime next %lu\n", m_lastQemuTime ); fflush( stdout );
}

bool waitEvent(void)
{
    /// Here we can listen to simulide input events
    //printf("Qemu: waitEvent at %lu pending %i at %lu\n",
    //       qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL ),
    //       m_arena->action, m_arena->simuTime/1000  ); fflush( stdout );

    while( m_arena->simuTime )  // An event is pending, Wait for QemuDevice::runEvent()
    {
        m_timeout += 1;
        if( m_timeout > 1e9 ) break; // Terminate process if timed out
    }
    m_timeout = 0;
    if( !m_arena->state ) return false;// Simulation stopped

    return true;
}

static void simu_event( void* opaque )
{
    //printf("Qemu: simu_event at %lu\n", qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL ) ); fflush( stdout );
    if( !waitEvent() ) return;

    m_arena->simuTime = getQemu_ps(); // ps
    m_arena->action = SIM_EVENT;

    while( m_arena->action )  // Wait for simulide to execute action
    {
        m_timeout += 1;
        if( m_timeout > 1e9 ) break; // Terminate loop if timed out
    }
    m_timeout = 0;
    m_arena->simuTime = 0;

    //printf("Qemu: simu_event at %lu\n", qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL ) ); fflush( stdout );
    waitForTime();
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

    m_ClkPeriod = 100*1000; // 100 us              //*1000; // ~1 ms

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

    m_resetEvent = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL );
    m_lastQemuTime = m_resetEvent;

    //timer_mod_ns( qtimer, m_resetEvent + m_ClkPeriod );
    printf("Qemu: resetEvent %lu\n", m_resetEvent );
    m_arena->state = 1;  // QemuDevice::stamp() unblocked here

    //usleep( 1*1000 );   // Let Simulation fully start running

    //printf("Qemu: waiting for time\n");fflush( stdout );
    //simu_event( NULL );
    waitForTime();

    printf("Qemu: starting main loop\n");fflush( stdout );

    int status = qemu_main_loop();
    qemu_cleanup( status );

    m_arena->state = 0;

#ifdef __linux__
    munmap( arena, shMemSize ); // Un-map shared memory
#elif defined(_WIN32)
    UnmapViewOfFile( arena );
    CloseHandle( hMapFile );
#endif

    printf("Qemu: process finished\n");fflush( stdout );

    return 0;
}
