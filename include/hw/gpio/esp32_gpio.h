
// Modified by Santiago Gonzalez, April 2025

#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32_GPIO "esp32.gpio"
#define ESP32_GPIO(obj)           OBJECT_CHECK( Esp32GpioState      , (obj), TYPE_ESP32_GPIO )
#define ESP32_GPIO_GET_CLASS(obj) OBJECT_GET_CLASS( Esp32GpioClass  , obj  , TYPE_ESP32_GPIO )
#define ESP32_GPIO_CLASS(klass)   OBJECT_CLASS_CHECK(E sp32GpioClass, klass, TYPE_ESP32_GPIO )

REG32( GPIO_STRAP, 0x0038 )

#define ESP32_STRAP_MODE_FLASH_BOOT 0x12
#define ESP32_STRAP_MODE_UART_BOOT  0x0f

typedef struct Esp32GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t gpio_out;
    uint32_t strap_mode;
    uint32_t gpio_in[2];
    uint32_t gpio_status[2];
    uint32_t gpio_pcpu_int[2];
    uint32_t gpio_acpu_int[2];
    uint32_t gpio_enable;
    uint32_t gpio_pin[40];
    uint32_t gpio_in_sel[256];
    uint32_t gpio_out_sel[40];
    qemu_irq out_irq[32];
    qemu_irq dir_irq[32];
    qemu_irq read_irq;
    qemu_irq conf_irq;
} Esp32GpioState;

#define ESP32_OUT_IRQ  "esp32_out_irq"
#define ESP32_IN_IRQ   "esp32_in_irq"
#define ESP32_DIR_IRQ  "esp32_dir_irq"
#define ESP32_READ_IRQ "esp32_read_irq"
#define ESP32_CONF_IRQ "esp32_conf_irq"

typedef struct Esp32GpioClass {
    SysBusDeviceClass parent_class;
} Esp32GpioClass;
