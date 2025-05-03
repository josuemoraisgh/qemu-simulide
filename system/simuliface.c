
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
#include "hw/irq.h"

// ------------------------------------------------
// -------- ARENA ---------------------------------

volatile qemuArena_t* m_arena = NULL;

// ------------------------------------------------

// ------ IRQ -------------------------------------
qemu_irq* gpio_irq   = NULL;
qemu_irq* dirio_irq  = NULL;
qemu_irq* readIn_irq = NULL;
//static qemu_irq *spi_cs_irq;

qemu_irq input_irq[40];
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
    while( m_arena->nextEvent )  // An event is pending, Wait for QemuDevice::runEvent()
    {
        m_timeout += 1;
        if( m_timeout > 1e9 ) break; // Terminate process if timed out
    }
    if( !m_arena->qemuRun ) return false;// Simulation stopped

    return true;
}


static void user_timeout_cb( void* opaque )
{
    int64_t now = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL );
    timer_mod_ns( qtimer, now + m_ClkPeriod );

    //if( !m_arena->qemuRun )

    if( !waitEvent() ) return;

    m_arena->nextEvent = getQemu_ps(); // ps
}

int simuMain( int argc, char** argv )
{
    const size_t shMemSize = sizeof( qemuArena_t );
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

    m_resetEvent = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL );
    timer_mod_ns( qtimer, m_resetEvent + m_ClkPeriod );

    m_arena->qemuRun = true;  // QemuDevice::stamp() unblocked here

    usleep( 1*1000 );   // Let Simulation fully start running

    int status = qemu_main_loop();
    qemu_cleanup(status);

    munmap( arena, shMemSize ); // Un-map shared memory

    m_arena->qemuRun = false;

    printf("QemuDevice: process finished\n");

    return 0;
}

// ----- IRQ Callbacks ---------------------------------------

void gpioChanged( void *opaque, int pin, int state )
{
    uint64_t qemuTime = getQemu_ps();

    //printf("write_pin\n");

    if( !waitEvent() ) return;

    uint64_t newState = m_arena->nextState;
    newState &= ~(1<<pin);
    newState |= state<<pin;

    m_arena->nextState = newState;
    m_arena->nextEvent = qemuTime;
}

void dirioChanged( void *opaque, int pin, int dir )
{
    //picsimlab_dir_pin( n, dir );

    uint64_t qemuTime = getQemu_ps();

    if( !waitEvent() ) return;

    if( pin > 0 )          // Set Pin direction
    {
        uint64_t newDirec = m_arena->nextDirec;
        newDirec &= ~(1<<pin);
        newDirec |= dir<<pin;

        m_arena->nextDirec = newDirec;
    }
    else                 // Pin extra config
    {
        //// pinExtraConfig( dir );

    }
    m_arena->nextEvent = qemuTime;
}

void readInput( void *opaque, int n, int value )
{
    uint64_t qemuTime = getQemu_ps();

    if( !waitEvent() ) return;

    m_arena->nextEvent = qemuTime;

    m_arena->readInput = true;
    while( m_arena->readInput ) // Wait for read completed
    {
        m_timeout += 1;
        if( m_timeout > 1e9 ) return; // Exit if timed out
    }

    for( int pin=0; pin<40; ++pin )
    {
        uint64_t mask = 1LL<<pin;
        bool state = m_arena->nextInput & mask;

        if( m_arena->maskInput & mask )         // Pin changed
        {
            if( state ) qemu_irq_raise( input_irq[pin] );
            else        qemu_irq_lower( input_irq[pin] );
        }
    }
}
