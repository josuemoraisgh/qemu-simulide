
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#include "simuliface.h"

#include "qemu/osdep.h"
#include "qemu-main.h"
#include "qemu/timer.h"
#include "sysemu/runstate.h"
#include "sysemu/sysemu.h"

// ------------------------------------------------
// -------- ARENA ---------------------------------
volatile bool* m_qemuStep;
volatile bool* m_qemuRun;
volatile bool* m_resetHard;

volatile bool*     m_readInput;
volatile uint64_t* m_nextInput;
volatile uint64_t* m_maskInput;
volatile uint64_t* m_nextState;
volatile uint64_t* m_nextDirec;

volatile uint64_t* m_nextEvent;

// ------------------------------------------------
// ------------------------------------------------

uint64_t m_timeout;
uint64_t m_resetEvent;
uint64_t m_ClkPeriod;

QEMUTimer* qtimer;

uint64_t getQemu_ps(void)
{
    uint64_t qemuTime = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL ); // ns
    qemuTime *= 1000;
    //printf("Qemu_ps %lu ", qemuTime);
    return qemuTime-m_resetEvent;
}

bool waitEvent(void)
{
    while( *m_nextEvent )  // An event is pending, Wait for QemuDevice::runEvent()
    {
        m_timeout += 1;
        if( m_timeout > 1e9 ) break; // Terminate process if timed out
    }

    if( !*m_qemuRun ) return false;// Simulation stopped

    return true;
}


static void user_timeout_cb( void* opaque )
{
    int64_t now = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL );
    timer_mod_ns( qtimer, now + m_ClkPeriod );

    //if( !*m_qemuRun )

    if( !waitEvent() ) return;

    if( *m_resetHard )
    {
        qemu_system_reset( SHUTDOWN_CAUSE_HOST_QMP_SYSTEM_RESET );

        *m_resetHard = false;

        m_resetEvent = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL )*1000;

        printf("Qemu reset at us %lu\n", m_resetEvent/1000000 );
    }
    else *m_nextEvent = getQemu_ps(); // ps
}

int simuMain( int argc, char** argv )
{
    const size_t shMemSize = 10*8;
    const char*  shMemKey;

    if( argc > 2 ) // Check if there are any arguments
    {
        shMemKey = argv[1];
        argv = &argv[2];
        argc -= 2;
    } else {
        printf("Error: No arguments provided.\n");
        return 1;
    }

    const int shMemId = shm_open( shMemKey, O_RDWR, 0666 ); // Open the shared memory object

    if( shMemId == -1 )
    {
        printf("Error opening arena: %s\n", shMemKey );
        return 1;
    }
    else printf("arena ok: %s\n", shMemKey );

    // Map shared memory
    void* arena = mmap( 0, shMemSize, PROT_READ | PROT_WRITE, MAP_SHARED, shMemId, 0);
    if( !arena )
    {
        printf("Error mapping arena\n");
        return 1;
    }
    else printf("arena mapped\n");

    fflush( stdout );

    //------------------------------------------------------------------

    bool* start = (bool*)arena;
    int dataSize = 8;
    m_qemuStep  = (bool*)     start;
    m_qemuRun   = (bool*)    (start + 1*dataSize);
    m_resetHard = (bool*)    (start + 2*dataSize);
    m_readInput = (bool*)    (start + 3*dataSize);
    m_nextState = (uint64_t*)(start + 4*dataSize);
    m_nextDirec = (uint64_t*)(start + 5*dataSize);
    m_nextEvent = (uint64_t*)(start + 6*dataSize);
    m_nextInput = (uint64_t*)(start + 7*dataSize);
    m_maskInput = (uint64_t*)(start + 8*dataSize);

    m_ClkPeriod = 1*1000*1000; // ~1 ms

    //------------------------------------------------------------------

    //runThread( argc, argv );

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

    m_resetEvent = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL );
    timer_mod_ns( qtimer, m_resetEvent + m_ClkPeriod );

    *m_qemuRun = true;  // QemuDevice::stamp() unblocked here

    usleep( 1*1000 );   // Let Simulation fully start running

    //int status = qemu_main_loop();
    //qemu_cleanup(status);

    munmap( arena, shMemSize ); // Un-map shared memory

    //shm_unlink( shMemKey );

    printf("QemuDevice: process finished\n");

    return 0;
}
