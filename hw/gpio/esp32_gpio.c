/*
 * ESP32 GPIO emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
// Modified by Santiago Gonzalez, April 2025

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"


static uint64_t esp32_gpio_read( void *opaque, hwaddr addr, unsigned int size )
{
    Esp32GpioState *gpioS = ESP32_GPIO(opaque);
    uint64_t r = 0;

    if( addr < 0x88 ){
        switch( addr )
        {
        case 0x04: r = gpioS->gpio_out;    break; //GPIO_OUT_REG
        case 0x20: r = gpioS->gpio_enable; break; //GPIO_ENABLE_REG
        case 0x38: r = gpioS->strap_mode;  break; //A_GPIO_STRAP:
        case 0x3C:                                //GPIO_IN_REG
            qemu_set_irq( gpioS->read_irq, -1 );  // Read pins
            r = gpioS->gpio_in[0];
            break;
        case 0x40:                                //GPIO_IN1_REG
            qemu_set_irq( gpioS->read_irq, -1 );  //Read pins
            r = gpioS->gpio_in[1];
            break;
        case 0x44: r = gpioS->gpio_status[0];   break; //GPIO_STATUS_REG
        case 0x50: r = gpioS->gpio_status[1];   break; //gpio_status[1]_REG
        case 0x60: r = gpioS->gpio_acpu_int[0]; break; //GPIO_ACPU_INT_REG
        case 0x68: r = gpioS->gpio_pcpu_int[0]; break; //GPIO_PCPU_INT_REG
        case 0x74: r = gpioS->gpio_acpu_int[1]; break; //GPIO_ACPU_INT1_REG
        case 0x7c: r = gpioS->gpio_pcpu_int[1]; break; //GPIO_PCPU_INT1_REG
        default: break;
        }
    }
    else if( addr >= 0x88 && addr < 0x130 ) { //GPIO_PINXX_REG
        int n = (addr - 0x88) / 4;
        r = gpioS->gpio_pin[n];
    }
    else if( addr >= 0x130 && addr < 0x530 ) { //GPIO_FUNCY_IN_SEL_CFG_REG
        int n = (addr - 0x130) / 4;
        r = gpioS->gpio_in_sel[n];
    }
    else if( addr >= 0x530 && addr < 0x5d0 ) { //GPIO_FUNCX_OUT_SEL_CFG_REG
        int n = (addr - 0x530) / 4;
        r = gpioS->gpio_out_sel[n];
    }
    return r;
}

static int get_triggering( int int_type, int oldval, int val ) //Interrupt type selection
{
    switch( int_type )
    {
    case 1: return (val >  oldval); //rising edge trigger
    case 2: return (val <  oldval); //falling edge trigger
    case 3: return (val != oldval); //any edge trigger
    case 4: return (val == 0);      //low level trigger
    case 5: return (val == 1);      //high level trigger
    }
    return 0; //GPIO interrupt disable
}

static void inputChanged( void *opaque, int n, int val )
{
    Esp32GpioState *gpioS = ESP32_GPIO( opaque );

    int n1;
    int i;
    if( n < 32 ) { n1 = n;    i = 0; }
    else         { n1 = n-32; i = 1; }

    uint32_t* pcpuInt = &gpioS->gpio_pcpu_int[i];
    uint32_t* acpuInt = &gpioS->gpio_acpu_int[i];
    uint32_t* gpioIn  = &gpioS->gpio_in[i];

    int oldval = (*gpioIn >> n1) & 1;
    *gpioIn &= ~(1 << n1);
    *gpioIn |= (val << n1);

    uint32_t gpioPin = gpioS->gpio_pin[n];
    int int_type = (gpioPin >> 7) & 7;
    int irq = get_triggering( int_type, oldval, val );

    // says bit 16 in the ref manual, is that wrong?
    if( irq && (gpioPin & (1 << 15)) )   // pro cpu int enable
    {
        qemu_set_irq( gpioS->irq, 1);
        *pcpuInt |= (1 << n1);
    }
    if( irq && (gpioPin & (1 << 13)) )   // app cpu int enable
    {
        qemu_set_irq( gpioS->irq, 1);
        *acpuInt |= (1 << n1);
    }
}

static void clearStatus( Esp32GpioState* gpioS, int n, uint64_t value )
{
    int start = n ? 32 : 0;
    int end   = n ? 40 : 32;

    for( int i=start; i<end; i++ )  // GPIO interrupts
    {
        uint32_t mask = 1 << (i-start);
        if( value & mask )
        {
            int int_type = (gpioS->gpio_pin[i] >> 7) & 7;
            uint32_t gpioMask = gpioS->gpio_in[n] & mask;

            if( (int_type == 4 && !gpioMask)
             || (int_type == 5 &&  gpioMask) )
                return;                        // RETURN, no clear, no irq
        }
    }
    gpioS->gpio_status[n]   &= ~value;
    gpioS->gpio_pcpu_int[n] &= ~value;
    gpioS->gpio_acpu_int[n] &= ~value;
    qemu_set_irq( gpioS->irq, 0 );
}

static void esp32_gpio_write( void *opaque, hwaddr addr, uint64_t value, unsigned int size )
{
    Esp32GpioState *gpioS = ESP32_GPIO( opaque );

    uint32_t oldOut    = gpioS->gpio_out;
    uint32_t oldEnable = gpioS->gpio_enable;

    if( addr < 0x88 ){
        switch( addr )
        {
        case 0x04: gpioS->gpio_out        =  value; break; // GPIO_OUT_REG
        case 0x08: gpioS->gpio_out       |=  value; break; // GPIO_OUT_W1TS_REG
        case 0x0C: gpioS->gpio_out       &= ~value; break; // GPIO_OUT_W1TC_REG
        case 0x20: gpioS->gpio_enable     =  value; break; // GPIO_ENABLE_REG
        case 0x24: gpioS->gpio_enable    |=  value; break; // GPIO_ENABLE_W1TS_REG
        case 0x28: gpioS->gpio_enable    &= ~value; break; // GPIO_ENABLE_W1TC_REG
        case 0x38: gpioS->strap_mode      =  value; break; // A_GPIO_STRAP
        case 0x44: gpioS->gpio_status[0]  =  value; break; // GPIO_STATUS_REG
        case 0x48: gpioS->gpio_status[0] |=  value; break; // GPIO_STATUS_W1TS_REG
        case 0x4c: clearStatus( gpioS, 0, value );  break; // GPIO_STATUS_W1TC_REG
        case 0x50: gpioS->gpio_status[1]  =  value; break; // GPIO_STATUS1_REG
        case 0x54: gpioS->gpio_status[1] |=  value; break; // GPIO_STATUS1_W1TS_REG
        case 0x58: clearStatus( gpioS, 1, value );  break; // GPIO_STATUS1_W1TC_REG
        }
    }
    else if( addr >= 0x88 && addr < 0x130) {            //GPIO_PINXX_REG
        int n = (addr - 0x88) / 4;
        gpioS->gpio_pin[n] = value;
    }
    else if( addr >= 0x130 && addr < 0x530) {           //GPIO_FUNCY_IN_SEL_CFG_REG
        int n = (addr - 0x130) / 4;
        gpioS->gpio_in_sel[n] = value;
        qemu_set_irq( gpioS->conf_irq, (0x1000 | n) );  //report in sel cfg change
    }
    else if( addr >= 0x530 && addr < 0x5d0) {           //GPIO_FUNCX_OUT_SEL_CFG_REG
        int n = (addr - 0x530) / 4;
        gpioS->gpio_out_sel[n] = value;
        qemu_set_irq( gpioS->conf_irq, (0x2000 | n) );  //report out sel cfg change
    }

    //printf("out 0x%04X in 0x%04X enable 0x%04X \n",gpioS->gpio_out, gpioS->gpio_in, gpioS->gpio_enable);

    uint32_t enableDiff = gpioS->gpio_enable ^ oldEnable;
    uint32_t outDiff    = gpioS->gpio_out    ^ oldOut;
    uint32_t changed    = outDiff | enableDiff;

    if( !changed ) return;      // RETURN, no pin changed

    for( int i=0; i<32; i++ )
    {
        uint32_t bitMask    = 1 << i;
        uint32_t enableMask = gpioS->gpio_enable & bitMask;
        uint32_t bitChanged = changed & bitMask;

        if( bitChanged && enableMask )
        {
            uint32_t outMask = gpioS->gpio_out & bitMask;

            qemu_set_irq( gpioS->out_irq[i], outMask >> i );

            gpioS->gpio_in[0] &= ~bitMask;
            gpioS->gpio_in[0] |=  outMask;
        }
        if( enableDiff & bitMask )
            qemu_set_irq( gpioS->dir_irq[i], enableMask >> i );
    }
}

static const MemoryRegionOps uart_ops = {
    .read =  esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset( DeviceState *dev )
{
    /// TODO
}

static void esp32_gpio_realize( DeviceState *dev, Error **errp )
{}

static void esp32_gpio_init( Object *obj )
{
    Esp32GpioState *gpioS = ESP32_GPIO(obj);
    SysBusDevice   *sbd   = SYS_BUS_DEVICE(obj);

    /* Set the default value for the strap_mode property */
    object_property_set_int( obj, "strap_mode", ESP32_STRAP_MODE_FLASH_BOOT, &error_fatal );

    memory_region_init_io( &gpioS->iomem, obj, &uart_ops, gpioS, TYPE_ESP32_GPIO, 0x1000 );
    sysbus_init_mmio( sbd, &gpioS->iomem );
    sysbus_init_irq( sbd, &gpioS->irq );

    qdev_init_gpio_out_named( DEVICE(gpioS), &gpioS->irq     , SYSBUS_DEVICE_GPIO_IRQ, 1 );
    qdev_init_gpio_out_named( DEVICE(gpioS),  gpioS->out_irq , ESP32_OUT_IRQ, 32 );
    qdev_init_gpio_out_named( DEVICE(gpioS),  gpioS->dir_irq , ESP32_DIR_IRQ, 32 );
    qdev_init_gpio_out_named( DEVICE(gpioS), &gpioS->read_irq, ESP32_READ_IRQ, 1 );
    qdev_init_gpio_out_named( DEVICE(gpioS), &gpioS->conf_irq, ESP32_CONF_IRQ, 1 );

    qdev_init_gpio_in_named( DEVICE(gpioS), inputChanged, ESP32_IN_IRQ, 40 );

    gpioS->gpio_in[0]  = 0x1;
    gpioS->gpio_in[1] = 0x8;
}

static Property esp32_gpio_properties[] = {
    /* The strap_mode needs to be explicitly set in the instance init, thus, set the default value to 0. */
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init( ObjectClass *klass, void *data )
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->legacy_reset = esp32_gpio_reset;
    dc->realize = esp32_gpio_realize;
    device_class_set_props( dc, esp32_gpio_properties );
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init,
    .class_size = sizeof(Esp32GpioClass),
};

static void esp32_gpio_register_types(void)
{
    type_register_static( &esp32_gpio_info );
}

type_init( esp32_gpio_register_types )
