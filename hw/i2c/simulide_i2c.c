/***************************************************************************
 *   Copyright (C) 2025 by Santiago González                               *
 *                                                                         *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/hw.h"
#include "hw/i2c/i2c.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "migration/vmstate.h"

#include "../system/simuliface.h"

enum i2c_action {
    I2C_START_READ=0,
    I2C_START_WRITE,
    I2C_START_WRITE_ASYNC,
    I2C_STOP,
    I2C_NOACK, /* Masker NACKed a receive byte.  */
    I2C_WRITE,
    I2C_READ,
    I2C_MATCH,
};

static int i2c_id = 0;

typedef struct I2cIface {
  I2CSlave i2c;
  uint8_t device_addr;
  uint8_t id;
} I2cIface;

#define TYPE_I2C_IFACE "i2c_iface"
#define I2C_IFACE(obj) OBJECT_CHECK(I2cIface, (obj), TYPE_I2C_IFACE)

static void i2c_iface_reset( DeviceState *dev ) {
  // I2cIface *s = I2C_IFACE(dev);
}

static uint8_t i2c_iface_rx( I2CSlave *i2c ) {
    //I2cIface *s = I2C_IFACE(i2c);
    uint64_t qemuTime = getQemu_ps();
    if( !waitEvent() ) return 0;
    //printf("i2c_rx %lu\n", qemuTime/1000000 );fflush( stdout );

    m_arena->time   = qemuTime;
    return 0;
}

static int i2c_iface_tx( I2CSlave *i2c, uint8_t data )
{
    I2cIface* i2cI = I2C_IFACE(i2c);

    uint64_t qemuTime = getQemu_ps();
    //printf("i2c_tx %i %lu\n", data, qemuTime/1000000 );fflush( stdout );
    if( !waitEvent() ) return 0;

    m_arena->action = SIM_I2C;
    m_arena->data32 = I2C_WRITE;
    m_arena->data16 = i2cI->id;
    m_arena->data8  = data;
    m_arena->time   = qemuTime;

    /*while( m_arena->action )
    {
        m_timeout += 1;
        if( m_timeout > 5e9 ) return 1; // Exit if timed out
    }*/

    return 0;
}

static int i2c_iface_ev( I2CSlave *i2c, enum i2c_event event )
{
    I2cIface* i2cI = I2C_IFACE(i2c);

    uint64_t qemuTime = getQemu_ps();
    //printf("i2c_ev %i %lu\n", event, qemuTime/1000000 );fflush( stdout );
    if( !waitEvent() ) return 0;

    m_arena->action = SIM_I2C;
    m_arena->data32 = event;
    m_arena->data16 = i2cI->id;
    m_arena->time   = qemuTime;

    /*while( m_arena->action )
    {
        m_timeout += 1;
        if( m_timeout > 5e9 ) return 1; // Exit if timed out
    }*/
    //m_arena->time = qemuTime + 120*1000*1000;

    return 0;
}

/*
 * For each channel, if it's enabled, recursively call match on those children.
 */
static bool i2c_iface_match( I2CSlave *candidate, uint8_t address,
                                bool broadcast, I2CNodeList *current_devs )
{
    I2cIface* i2cI = I2C_IFACE( candidate );
    //printf("i2c_match %i\n", address );fflush( stdout );

    uint64_t qemuTime = getQemu_ps();
    if( !waitEvent() ) return false;

    m_arena->action = SIM_I2C;
    m_arena->data32 = I2C_MATCH;
    m_arena->data16 = i2cI->id;
    m_arena->data8  = address;
    m_arena->time   = qemuTime;

    /*while( m_arena->action )
    {
        m_timeout += 1;
        if( m_timeout > 5e9 ) return false; // Exit if timed out
    }*/

    // Always return true
    i2cI->device_addr = address;
    I2CNode *node = g_malloc(sizeof(struct I2CNode));
    node->elt = candidate;
    QLIST_INSERT_HEAD( current_devs, node, next );
    return true;
}

static void i2c_iface_realize( DeviceState *dev, Error **errp )
{
    I2CSlave *i2c = I2C_SLAVE(dev);
    I2cIface *s   = I2C_IFACE(i2c);
    s->id = i2c_id++;
}

static const VMStateDescription vmstate_i2c_iface = {
    .name = "I2cIface",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]){VMSTATE_I2C_SLAVE(i2c, I2cIface),
                               VMSTATE_END_OF_LIST()}
};

static void i2c_iface_class_init( ObjectClass *klass, void *data )
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS( klass );
    k->event = i2c_iface_ev;
    k->recv  = i2c_iface_rx;
    k->send  = i2c_iface_tx;
    k->match_and_add = i2c_iface_match;

    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->legacy_reset = i2c_iface_reset;
    dc->vmsd    = &vmstate_i2c_iface;
    dc->realize = i2c_iface_realize;
}

  static const TypeInfo i2c_iface_info = {
      .name   = TYPE_I2C_IFACE,
      .parent = TYPE_I2C_SLAVE,
      .instance_size = sizeof(I2cIface),
      .class_init = i2c_iface_class_init,
  };

  static void i2c_iface_register_types(void) {
    type_register_static( &i2c_iface_info );
  }

type_init( i2c_iface_register_types )
