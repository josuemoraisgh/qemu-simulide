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

#include "sysemu/cpu-timers.h"

// ------------------------------------------------
// -------- ARENA ---------------------------------

volatile qemuArena_t* m_arena = NULL;

// ------------------------------------------------

uint64_t m_timeout;
//uint64_t m_resetEvent;
uint64_t m_ClkPeriod;

QEMUTimer* qtimer;

uint64_t getQemu_ps(void)
{
    uint64_t qemuTime = icount_get_ps(); //qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL ); // ns cpu_get_clock(); //
    //qemuTime *= 1000;
    return qemuTime; //-m_resetEvent;
}

bool waitEvent(void)
{
    /// Here we can listen to simulide input events

    while( m_arena->time )  // An event is pending, Wait for QemuDevice::runEvent()
    {
        m_timeout += 1;
        if( m_timeout > 1e9 ) break; // Terminate process if timed out
    }
    m_timeout = 0;
    if( !m_arena->state ) return false;// Simulation stopped

    return true;
}

static void user_timeout_cb( void* opaque )
{
    int64_t now = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL );
    timer_mod_ns( qtimer, now + m_ClkPeriod );

    if( !waitEvent() ) return;
    //printf("timeout_cb %lu\n", now ); fflush( stdout );
    m_arena->time = getQemu_ps(); // ps
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
        printf("Qemu: Error mapping arena\n");
        return 1;
    }
    else printf("Qemu: arena mapped %i bytes\n", shMemSize );

    fflush( stdout );

    //------------------------------------------------------------------

    m_arena = (qemuArena_t*)arena;

    //------------------------------------------------------------------

    m_ClkPeriod = 1*1000*1000; // ~1 ms

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
    timer_init_full( qtimer, NULL, QEMU_CLOCK_VIRTUAL, 1, 0, user_timeout_cb, NULL );

    //m_resetEvent = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL );
    timer_mod_ns( qtimer,  m_ClkPeriod );

    m_arena->state = 1;  // QemuDevice::stamp() unblocked here

    usleep( 1*1000 );   // Let Simulation fully start running

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
