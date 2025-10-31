/*
 * STM32 fa103c8 (Blue Pill) Development Board
 *
 * Copyright (C) 2018 Basel Alsayeh
 * Copyright (C) 2020 Luis CLaudio G Lopes
 *
 * Implementation based on
 * Olimex "STM-P103 Development Board Users Manual Rev. A, April 2008"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/arm/stm32.h"
#include "hw/sysbus.h"
#include "hw/arm/armv7m.h"
// #include "hw/devices.h"
#include "ui/console.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/arm/boot.h"
#include "hw/i2c/i2c.h"
#include "hw/ssi/ssi.h"

#include "../system/simuliface.h"

typedef struct
{
   Stm32 *stm32;
   qemu_irq pin_irq[4*16]; // 4 Ports of 16 Pins
   qemu_irq *pout_irq;
   qemu_irq *pdir_irq;
   qemu_irq *psync_irq;
   DeviceState *gpio_a;
   DeviceState *gpio_b;
   DeviceState *gpio_c;
   DeviceState *gpio_d;
   DeviceState *uart1;
   DeviceState *uart2;
   DeviceState *uart3;
   DeviceState *i2c1;
   DeviceState *i2c2;
   DeviceState *spi1;
   DeviceState *spi2;
   DeviceState *afio;
   DeviceState *tim1;
   DeviceState *tim2;
   DeviceState *tim3;
   DeviceState *tim4;
   DeviceState *tim5; 
   DeviceState *adc1;
   DeviceState *adc2;
   //DeviceState *exti;
} Stm32_F103c8_Mcu;

Stm32_F103c8_Mcu* mcu;

//#define QEMU_INTERNAL_UART0_BAUD 7
//#define QEMU_INTERNAL_UART1_BAUD 8
//#define QEMU_INTERNAL_UART2_BAUD 9

uint32_t stm32_uart_baud_rate(void *opaque);
void stm32_uart_receive( Stm32Uart* uart, const uint8_t data );
//void stm32_f103c8_uart_action(void);

void stm32_f103c8_uart_action(void)
{
    //sw
    Stm32Uart* uart = NULL;

    int id = m_arena->data8;
    uint16_t data = m_arena->data16;

    switch( id )
    {
    case 0: uart = (Stm32Uart*)mcu->uart1; break;
    case 1: uart = (Stm32Uart*)mcu->uart2; break;
    case 2: uart = (Stm32Uart*)mcu->uart3; break;
    }
    stm32_uart_receive( uart, data );
}

void stm32_f103c8_gpio_in_action(void)
{
    uint8_t port   = m_arena->data8;
    uint8_t pin    = m_arena->mask8;
    uint16_t state = m_arena->data16;

    //printf("stm32_f103c8_gpio_in_action %i %i %i\n", port, pin, state );fflush( stdout );

    int irqNumber = port*16+pin;
    assert( mcu->pin_irq[irqNumber] );
    qemu_set_irq( mcu->pin_irq[irqNumber], state );
}

Stm32Timer* stm32_get_timer( int number )
{
    switch( number )
    {
    case 1: return (Stm32Timer*)mcu->tim1;
    case 2: return (Stm32Timer*)mcu->tim2;
    case 3: return (Stm32Timer*)mcu->tim3;
    case 4: return (Stm32Timer*)mcu->tim4;
    case 5: return (Stm32Timer*)mcu->tim5;
    }
    return NULL;
}
//void stm32_f103c8_timer_action(void);
//void stm32_f103c8_timer_action(void)
//{
//    Stm32Timer* timer = NULL;
//
//    int id = m_arena->data8;
//
//    switch (id)
//    {
//    case 0: timer = (Stm32Timer*)mcu->tim1; break;
//    case 1: timer = (Stm32Timer*)mcu->tim2; break;
//    case 2: timer = (Stm32Timer*)mcu->tim3; break;
//    case 3: timer = (Stm32Timer*)mcu->tim4; break;
//    case 4: timer = (Stm32Timer*)mcu->tim5; break;
//    }
//    stm32_timer_action( timer );
//}


/// TODO: flash and ram sizes configurable ???
#define FLASH_SIZE 0x00020000
#define RAM_SIZE 0x00005000
/* Main SYSCLK frequency in Hz (24MHz) */
#define SYSCLK_FRQ 24000000ULL

static void stm32_f103c8_init( MachineState *machine )
{
   Clock *sysclk;

   //uint32_t freq = getFrequency();

   mcu = (Stm32_F103c8_Mcu*)g_malloc0( sizeof(Stm32_F103c8_Mcu) );

   sysclk = clock_new( OBJECT(machine), "SYSCLK");
   clock_set_hz( sysclk, SYSCLK_FRQ );
   stm32_init( FLASH_SIZE, RAM_SIZE, machine->kernel_filename, 8000000, 32768, sysclk );

   mcu->gpio_a = DEVICE( object_resolve_path("/machine/stm32/gpio[a]" , NULL) );
   mcu->gpio_b = DEVICE( object_resolve_path("/machine/stm32/gpio[b]" , NULL) );
   mcu->gpio_c = DEVICE( object_resolve_path("/machine/stm32/gpio[c]" , NULL) );
   mcu->gpio_d = DEVICE( object_resolve_path("/machine/stm32/gpio[d]" , NULL) );
   mcu->uart1  = DEVICE( object_resolve_path("/machine/stm32/uart[1]" , NULL) );
   mcu->uart2  = DEVICE( object_resolve_path("/machine/stm32/uart[2]" , NULL) );
   mcu->uart3  = DEVICE( object_resolve_path("/machine/stm32/uart[3]" , NULL) );
   mcu->i2c1   = DEVICE( object_resolve_path("/machine/stm32/i2c[1]"  , NULL) );
   mcu->i2c2   = DEVICE( object_resolve_path("/machine/stm32/i2c[2]"  , NULL) );
   mcu->spi1   = DEVICE( object_resolve_path("/machine/stm32/spi[1]"  , NULL) );
   mcu->spi2   = DEVICE( object_resolve_path("/machine/stm32/spi[2]"  , NULL) );
   mcu->afio   = DEVICE( object_resolve_path("/machine/stm32/afio"    , NULL) );
   mcu->tim1   = DEVICE( object_resolve_path("/machine/stm32/timer[1]", NULL) );
   mcu->tim2   = DEVICE( object_resolve_path("/machine/stm32/timer[2]", NULL) );
   mcu->tim3   = DEVICE( object_resolve_path("/machine/stm32/timer[3]", NULL) );
   mcu->tim4   = DEVICE( object_resolve_path("/machine/stm32/timer[4]", NULL) );
   mcu->tim5   = DEVICE( object_resolve_path("/machine/stm32/timer[5]", NULL) );
   mcu->adc1   = DEVICE( object_resolve_path("/machine/stm32/adc[1]"  , NULL) );
   mcu->adc2   = DEVICE( object_resolve_path("/machine/stm32/adc[2]"  , NULL) );
   //mcu->exti   = DEVICE( object_resolve_path("/machine/stm32/exti"  , NULL) );

   assert( mcu->gpio_a );
   assert( mcu->gpio_b );
   assert( mcu->gpio_c );
   assert( mcu->gpio_d );
   assert( mcu->uart2 );
   assert( mcu->uart1 );
   assert( mcu->uart3 );
   assert( mcu->i2c1 );
   assert( mcu->i2c2 );
   assert( mcu->spi1 );
   assert( mcu->spi2 );
   assert( mcu->afio );
   assert( mcu->tim1 );
   assert( mcu->tim2 );
   assert( mcu->tim3 );
   assert( mcu->tim4 );
   assert( mcu->tim5 );
   assert( mcu->adc1 );
   assert( mcu->adc2 );
   //assert( mcu->exti );

   /* Connect RS232 to UART 1 */
   //stm32_uart_connect( (Stm32Uart *)mcu->uart1, serial_hd(0)/*, 0*/ );

   /* These additional UARTs have not been tested yet... */
   //stm32_uart_connect( (Stm32Uart *)mcu->uart2, serial_hd(1)/*, 0*/  );
   //stm32_uart_connect( (Stm32Uart *)mcu->uart3, serial_hd(2)/*, 0*/  );

   DeviceState *i2c_master1 = DEVICE(mcu->i2c1);
   I2CBus *i2c_bus1 = I2C_BUS( qdev_get_child_bus(i2c_master1, "i2c") );
   i2c_slave_create_simple( i2c_bus1, "i2c_iface", 0x00 );

   DeviceState *i2c_master2 = DEVICE(mcu->i2c2);
   I2CBus *i2c_bus2 = I2C_BUS(qdev_get_child_bus(i2c_master2, "i2c"));
   i2c_slave_create_simple(i2c_bus2, "i2c_iface", 0x01);

   /// FIXME simulide: DeviceState *spi_master1 = DEVICE(mcu->spi1);
   /// FIXME simulide: SSIBus *spi_bus1 = (SSIBus *)qdev_get_child_bus(spi_master1, "ssi");
   /// FIXME simulide: ssi_create_peripheral(spi_bus1, "simul_spi");

   /// FIXME simulide: DeviceState *spi_master2 = DEVICE(mcu->spi2);
   /// FIXME simulide: SSIBus *spi_bus2 = (SSIBus *)qdev_get_child_bus(spi_master2, "ssi");
   /// FIXME simulide: ssi_create_peripheral(spi_bus2, "simul_spi");

   // GPIO In Irq:
   int start = 0;
   for( int pin=0; pin<16; ++pin )
       mcu->pin_irq[start+pin] = qdev_get_gpio_in( mcu->gpio_a, pin );
   start += 16;
   for( int pin=0; pin<16; ++pin )
       mcu->pin_irq[start+pin] = qdev_get_gpio_in( mcu->gpio_b, pin );
   start += 16;
   for( int pin=0; pin<16; ++pin )
       mcu->pin_irq[start+pin] = qdev_get_gpio_in( mcu->gpio_c, pin );
   start += 16;
   for( int pin=0; pin<16; ++pin )
       mcu->pin_irq[start+pin] = qdev_get_gpio_in( mcu->gpio_d, pin );

   for( int pin=0; pin<16*4; ++pin ) assert( mcu->pin_irq[pin] );

   armv7m_load_kernel( ARM_CPU(first_cpu), machine->kernel_filename, 0, FLASH_SIZE );
}

static void stm32_f103c8_machine_init(MachineClass *mc)
{
   mc->desc = "STM32F103C8 MCU";
   mc->init = stm32_f103c8_init;
   mc->block_default_type = IF_IDE;
   mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("stm32-f103c8-simul", stm32_f103c8_machine_init)
