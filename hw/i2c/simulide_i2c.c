/*
 * PICSimLab I2C  Emulation
 * 
 *  
 * Luis Claudio Gamboa Lopes 2022 lcgamboa@yahoo.com
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


static int i2c_id = 0;

typedef struct I2cIface {
  /*< private >*/
  I2CSlave i2c;
  /*< public >*/
 // uint8_t device_addr;
  uint8_t id;
} I2cIface;

#define TYPE_I2C_IFACE "i2c_iface"
#define I2C_IFACE(obj) OBJECT_CHECK(I2cIface, (obj), TYPE_I2C_IFACE)

static void i2c_iface_reset( DeviceState *dev ) {
  // I2cIface *s = I2C_IFACE(dev);
}

static uint8_t i2c_iface_rx( I2CSlave *i2c ) {
  //I2cIface *s = I2C_IFACE(i2c);
  //  read i2c
  return 0; /// FIXME SIMULIDE i2c_iface_event(s->id, s->device_addr, I2C_NACK+2);
}

static int i2c_iface_tx( I2CSlave *i2c, uint8_t data )
{
    //I2cIface *s = I2C_IFACE(i2c);
    printf("i2c_tx %i\n", data );fflush( stdout );
    //  write i2c
    return 0; /// FIXME SIMULIDE i2c_iface_event(s->id, s->device_addr, (data<<8)|(I2C_NACK+1));
}

static int i2c_iface_ev( I2CSlave *i2c, enum i2c_event event ) {
  //I2cIface *s = I2C_IFACE(i2c);
    printf("i2c_ev %i\n", event );fflush( stdout );
  // i2c event
  return 0; /// FIXME SIMULIDE i2c_iface_event(s->id, s->device_addr, event);
}

/*
 * For each channel, if it's enabled, recursively call match on those children.
 */
static bool i2c_iface_match( I2CSlave *candidate, uint8_t address,
                                bool broadcast, I2CNodeList *current_devs )
{
  //I2cIface *s = I2C_IFACE( candidate );

  printf("i2c_match %i\n", address );fflush( stdout );
  // Always return true
  //s->device_addr = address;
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
