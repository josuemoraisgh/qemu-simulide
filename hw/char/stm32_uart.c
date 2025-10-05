/*
 * STM32 Microcontroller UART module
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Source code based on pl011.c
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
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

#include "hw/hw.h"
#include "hw/irq.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/log.h"

#include "../system/simuliface.h"

/* DEFINITIONS*/

/* See the README file for details on these settings. */
//#define DEBUG_STM32_UART
//#define STM32_UART_NO_BAUD_DELAY
//#define STM32_UART_ENABLE_OVERRUN

#ifdef DEBUG_STM32_UART
#define DPRINTF(fmt, ...)                                                      \
  do {                                                                         \
    fprintf(stderr, "STM32_UART: " fmt, ##__VA_ARGS__);                        \
  } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define USART_SR_OFFSET 0x00
#define SR_TXE_BIT 7
#define SR_TC_BIT 6
#define SR_RXNE_BIT 5
#define SR_ORE_BIT 3

#define USART_DR_OFFSET 0x04

#define BRR_r_OFFSET 0x08

#define CR1_OFFSET 0x0c
#define CR1_UE_BIT 13
#define CR1_M_BIT 12
#define CR1_PCE_BIT 10
#define CR1_PS_BIT 9
#define CR1_TXEIE_BIT 7
#define CR1_TCIE_BIT 6
#define CR1_RXNEIE_BIT 5
#define CR1_TE_BIT 3
#define CR1_RE_BIT 2

#define CR2_r_OFFSET 0x10
#define CR2_r_STOP_START 12
#define CR2_r_STOP_MASK 0x00003000

#define CR3_r_OFFSET 0x14
#define CR3_r_CTSE_BIT 9
#define CR3_r_RTSE_BIT 8

#define USART_GTPR_OFFSET 0x18

#define CR3_r_DMAR_BIT 0x40
#define CR3_r_DMAT_BIT 0x80

struct Stm32Uart {
  /* Inherited */
  SysBusDevice busdev;

  /* Properties */
  stm32_periph_t periph;
  // void *stm32_rcc_prop;
  // void *stm32_gpio_prop;
  // void *stm32_afio_prop;

  /* Private */
  MemoryRegion iomem;

  Stm32Rcc *stm32_rcc;
  Stm32Gpio **stm32_gpio;
  Stm32Afio *stm32_afio;

  uint32_t bits_per_sec;
  int64_t ns_per_char;

  /* Register Values */
  uint32_t RDR_r, TDR_r, BRR_r, CR1_r, CR2_r, CR3_r;

  /* Register Field Values */
  uint32_t SR_TXE, SR_TC, SR_RXNE, SR_ORE, CR1_UE,
      CR1_TXEIE, CR1_TCIE, CR1_RXNEIE, CR1_TE,
      CR1_RE;

  bool sr_read_since_ore_set;

  /* Indicates whether the USART is currently receiving a byte. */
  bool receiving;

  /* Timers used to simulate a delay corresponding to the baud rate. */
  struct QEMUTimer *rx_timer;
  struct QEMUTimer *tx_timer;
  struct QEMUTimer *tx_dma_timer;

  CharBackend chr;

  /* Stores the USART pin mapping used by the board.  This is used to check
   * the AFIO's USARTx_REMAP register to make sure the software has set
   * the correct mapping.
   */
  //uint32_t afio_board_map;

  qemu_irq irq;
  int curr_irq_level;
  int id;
};

/* HELPER FUNCTIONS */

/* Update the baud rate based on the USART's peripheral clock frequency. */
static void stm32_uart_baud_update(Stm32Uart *s)
{
  uint32_t clk_freq = stm32_rcc_get_periph_freq(s->stm32_rcc, s->periph);
  uint64_t ns_per_bit;

  if ((s->BRR_r == 0) || (clk_freq == 0)) {
    s->bits_per_sec = 9600; //use 0 cause simulation freeze because use timer with 0 delay
  } else {
    s->bits_per_sec = clk_freq / s->BRR_r;
  }
  ns_per_bit = 1000000000LL / s->bits_per_sec;

  uint64_t qemuTime = getQemu_ps();
  if( !waitEvent() ) return;        // Wait until SimulIDE is free

  m_arena->action = SIM_USART;
  m_arena->data8  = SIM_USART_BAUD;
  m_arena->data16 = s->id;         // Uart number
  m_arena->data32 = s->bits_per_sec;
  m_arena->time   = qemuTime;  // time in ps
  /// TODO: baudrate = s->bits_per_sec

  /* We assume 10 bits per character.  This may not be exactly
  * accurate depending on settings, but it should be good enough. */
  s->ns_per_char = ns_per_bit * 10;

#ifdef DEBUG_STM32_UART
  DPRINTF("%s clock is set to %lu Hz.\n", stm32_periph_name(s->periph),
          (unsigned long)clk_freq);
  DPRINTF("%s BRR set to %lu.\n", stm32_periph_name(s->periph),
          (unsigned long)s->BRR_r);
  DPRINTF("%s Baud is set to %lu bits per sec.\n", stm32_periph_name(s->periph),
          (unsigned long)s->bits_per_sec);
#endif
}

/* Handle a change in the peripheral clock. */
static void stm32_uart_clk_irq_handler(void *opaque, int n, int level)
{
  Stm32Uart *s = (Stm32Uart *)opaque;

  assert(n == 0);

  if (level) stm32_uart_baud_update(s); /* Only update the BAUD rate if the IRQ is being set. */
}

/* Routine which updates the USART's IRQ.
 * This should be called whenever an interrupt-related flag is updated.
*/
static void stm32_uart_update_irq(Stm32Uart *s)
{
  /* Note that we are not checking the ORE flag, but we should be. */
  int new_irq_level =
      (s->CR1_TCIE   & s->SR_TC) |
      (s->CR1_TXEIE  & s->SR_TXE) |
      (s->CR1_RXNEIE & (s->SR_ORE | s->SR_RXNE));

  /* Only trigger an interrupt if the IRQ level changes.  We probably could
   * set the level regardless, but we will just check for good measure.
   */
  if (new_irq_level ^ s->curr_irq_level) {
    qemu_set_irq(s->irq, new_irq_level);
    s->curr_irq_level = new_irq_level;
  }
}

static void stm32_uart_start_tx(Stm32Uart *s, uint32_t value);

/* Routine to be called when a transmit is complete. */
static void stm32_uart_tx_complete(Stm32Uart *s)
{
  if (s->SR_TXE == 1) {
    /* If the buffer is empty, there is nothing waiting to be transmitted. Mark the transmit complete. */
    s->SR_TC = 1;
  } else {
    // Otherwise, mark the transmit buffer as empty and start transmitting the value stored there.

    s->SR_TXE = 1;
    stm32_uart_start_tx( s, s->TDR_r );
  }
  stm32_uart_update_irq(s);
}

/* Start transmitting a character. */
static void stm32_uart_start_tx( Stm32Uart *s, uint32_t value )
{
    uint64_t time_ns = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL );

    s->SR_TC = 0; // Reset the Transmission Complete flag to indicate a transmit is in progress.

    if( !waitEvent() ) return;        // Wait until SimulIDE is free

    m_arena->action = SIM_USART;
    m_arena->data8  = SIM_USART_WRITE;
    m_arena->data16 = s->id;         // Uart number
    m_arena->data32 = value;
    m_arena->time   = time_ns*1000;  // time in ps

    timer_mod( s->tx_timer, time_ns + s->ns_per_char ); // Start the transmit delay timer.
}

/* TIMER HANDLERS */
/* Once the receive delay is finished, indicate the USART is finished receiving.
 * This will allow it to receive the next character.  The current character was
 * already received before starting the delay.
 */
static void stm32_uart_rx_timer_expire(void *opaque)
{
  Stm32Uart *s = (Stm32Uart *)opaque;

  s->receiving = false;

  uint64_t curr_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
  timer_mod(s->rx_timer, curr_time + s->ns_per_char);
}

/* When the transmit delay is complete, mark the transmit as complete
 * (the character was already sent before starting the delay). */
static void stm32_uart_tx_timer_expire(void *opaque)
{
  Stm32Uart *s = (Stm32Uart *)opaque;

  stm32_uart_tx_complete(s);
}

/* DMA tx delay */
static void stm32_uart_tx_dma_timer_expire(void *opaque)
{
    Stm32Uart *s = (Stm32Uart *)opaque;
    uint64_t curr_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (s->CR3_r & CR3_r_DMAT_BIT)
    {
        qemu_set_irq(*stm32_DMA1_irq, 0x00000011);
        qemu_set_irq(*stm32_DMA1_irq, 0x00000000);

        stm32_uart_tx_complete(s);

        timer_mod( s->tx_dma_timer, curr_time + s->ns_per_char );
    }
}

/* CHAR DEVICE HANDLERS */
int stm32_uart_can_receive(void *opaque);
void stm32_uart_receive(void *opaque, const uint8_t *buf, int size);
uint32_t stm32_uart_baud_rate(void *opaque);

int stm32_uart_can_receive(void *opaque)
{
  Stm32Uart *s = (Stm32Uart *)opaque;

  if (s->CR1_UE && s->CR1_RE) /* The USART can only receive if it is enabled. */
  {
    if (s->receiving) {
      /* If the USART is already receiving, then it cannot receive another character yet. */
      return 0;
    } else {
/// TODO:
#ifdef STM32_UART_ENABLE_OVERRUN
      /* If overrun is enabled, then always allow the next character to be
       * received even if the buffer already has a value.  This is how
       * real hardware behaves.*/
      return 1;
#else
      /* Otherwise, do not allow the next character to be received until
       * software has read the previous one. */
      if (s->SR_RXNE) return 0;
      else            return 1;
#endif
    }
  } else {
    /* Always allow a character to be received if the module is disabled.
     * However, the character will just be ignored (just like on real hardware). */
    return 1;
  }
}

static void stm32_uart_event(void *opaque, QEMUChrEvent event){
  /* Do nothing */
}

uint32_t stm32_uart_baud_rate(void *opaque)
{
  Stm32Uart *s = (Stm32Uart *)opaque;
  return s->bits_per_sec;
}


void stm32_uart_receive(void *opaque, const uint8_t *buf, int size)
{
  Stm32Uart *s = (Stm32Uart *)opaque;
  uint64_t curr_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

  assert(size > 0);

  /* Only handle the received character if the module is enabled, */
  if (s->CR1_UE && s->CR1_RE)
  {
    /* If there is already a character in the receive buffer, then set the overflow flag. */
    if (s->SR_RXNE) {
      s->SR_ORE = 1;
      s->sr_read_since_ore_set = false;
      stm32_uart_update_irq(s);
    }

    /* Receive the character and mark the buffer as not empty. */
    s->RDR_r = *buf;
    s->SR_RXNE = 1;
    stm32_uart_update_irq(s);
  }

  if (s->CR3_r & CR3_r_DMAR_BIT)
  {
    qemu_set_irq(*stm32_DMA1_irq, 0x00000010);
    qemu_set_irq(*stm32_DMA1_irq, 0x00000000);
  } else {

#ifdef STM32_UART_NO_BAUD_DELAY
    /* Do nothing - there is no delay before the module reports it can receive the next character. */
#else
    /* Indicate the module is receiving and start the delay. */
    s->receiving = true;
#endif
    /* Set timer in either case - main event loop must run again to
     * trigger next receive when using STM32_UART_NO_BAUD_DELAY. */
    curr_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(s->rx_timer, curr_time + s->ns_per_char);
  }
}

/* REGISTER IMPLEMENTATION */

static uint32_t stm32_uart_SR_read(Stm32Uart *s)
{
  /* If the Overflow flag is set, reading the SR register is the first step to resetting the flag. */
  if (s->SR_ORE) s->sr_read_since_ore_set = true;

  return (s->SR_TXE << SR_TXE_BIT) |
         (s->SR_TC << SR_TC_BIT) |
         (s->SR_RXNE << SR_RXNE_BIT) |
         (s->SR_ORE << SR_ORE_BIT);
}

static void stm32_uart_SR_write(Stm32Uart *s, uint32_t new_value)
{
  uint32_t new_TC, new_RXNE;

  new_TC = extract32(new_value, SR_TC_BIT, 1);

  /* The Transmit Complete flag can be cleared, but not set. */
  if (new_TC) hw_error("Software attempted to set USART TC bit\n");

  s->SR_TC = new_TC;

  new_RXNE = extract32( new_value, SR_RXNE_BIT, 1 );
  /* The Read Data Register Not Empty flag can be cleared, but not set. */
  if (new_RXNE) hw_error("Software attempted to set USART RXNE bit\n");

  s->SR_RXNE = new_RXNE;

  stm32_uart_update_irq(s);
}

static void stm32_uart_DR_read(Stm32Uart *s, uint32_t *read_value)
{
  /* If the Overflow flag is set, then it should be cleared if the software
   * performs an SR read followed by a DR read.
   */
  if( s->SR_ORE && s->sr_read_since_ore_set ) s->SR_ORE = 0;


  if (!s->CR1_UE) hw_error("Attempted to read from USART_DR while UART was disabled.");
  if (!s->CR1_RE) hw_error("Attempted to read from USART_DR while UART receiver was disabled.");

  if (s->SR_RXNE) {
    /* If the receive buffer is not empty, return the value. and mark the buffer as empty. */
    (*read_value) = s->RDR_r;
    s->SR_RXNE = 0;
  }
  else hw_error("Read value from USART_DR while it was empty.");

  stm32_uart_update_irq(s);
}

static void stm32_uart_DR_write(Stm32Uart *s, uint32_t new_value)
{
  uint32_t write_value = new_value & 0x000001ff;

  if (!s->CR1_UE) hw_error("Attempted to write to USART_DR while UART was disabled.");
  if (!s->CR1_TE) hw_error("Attempted to write to USART_DR while UART transmitter was disabled.");

  // stm32_uart_check_tx_pin(s);

  if (s->SR_TC) {
    /* If the Transmission Complete bit is set, it means the USART is not
     * currently transmitting.  This means, a transmission can immediately start.
     */
    stm32_uart_start_tx(s, write_value);
  } else {
    /* Otherwise check to see if the buffer is empty.
     * If it is, then store the new character there and mark it as not empty.
     * If it is not empty, trigger a hardware error.  Software should check
     * to make sure it is empty before writing to the Data Register.
     */
    if (s->SR_TXE) {
      s->TDR_r = write_value;
      s->SR_TXE = 0;
    }
    else hw_error("Wrote new value to USART_DR while it was non-empty.");
  }
  stm32_uart_update_irq(s);
}

/* Update the Baud Rate Register. */
static void stm32_uart_BRR_write(Stm32Uart *s, uint32_t new_value, bool init)
{
  s->BRR_r = new_value & 0x0000ffff;

  stm32_uart_baud_update(s);
}

static void stm32_uart_CR1_write( Stm32Uart *s, uint32_t new_value, bool init )
{
    s->CR1_UE = extract32( new_value, CR1_UE_BIT, 1 );

    //if (s->CR1_UE)  /// TODO: maybe we don't need this
    //{
    //  /* Check to make sure the correct mapping is selected when enabling the USART. */
    //  if (s->afio_board_map != stm32_afio_get_periph_map(s->stm32_afio, s->periph)) {
    //    hw_error("Bad AFIO mapping for %s", stm32_periph_name(s->periph));
    //  }
    //}
    s->CR1_TXEIE  = extract32( new_value, CR1_TXEIE_BIT , 1 );
    s->CR1_TCIE   = extract32( new_value, CR1_TCIE_BIT  , 1 );
    s->CR1_RXNEIE = extract32( new_value, CR1_RXNEIE_BIT, 1 );
    s->CR1_TE     = extract32( new_value, CR1_TE_BIT, 1 );
    s->CR1_RE     = extract32( new_value, CR1_RE_BIT, 1 );

    s->CR1_r = new_value & 0x00003fff;

    stm32_uart_update_irq(s);
}

static void stm32_uart_CR2_write(Stm32Uart *s, uint32_t new_value, bool init)
{
  s->CR2_r = new_value & 0x00007f7f;
}

static void stm32_uart_CR3_write(Stm32Uart *s, uint32_t new_value, bool init)
{
  s->CR3_r = new_value & 0x000007ff;

  if (s->CR3_r & CR3_r_DMAT_BIT) {
#ifdef STM32_UART_NO_BAUD_DELAY
    printf("Warning: Tx inturrupt does not work without uart delay!\n");
#else
    uint64_t curr_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(s->tx_dma_timer, curr_time + s->ns_per_char);
#endif
  }
}

static void stm32_uart_reset(DeviceState *dev)
{
    Stm32Uart *s = STM32_UART(dev);

    /* Initialize the status registers.  These are mostly
     * read-only, so we do not call the "write" routine like normal. */
    s->SR_TXE  = 1;
    s->SR_TC   = 1;
    s->SR_RXNE = 0;
    s->SR_ORE  = 0;

    // Do not initialize USART_DR - it is documented as undefined at reset
    // and does not behave like normal registers.
    stm32_uart_BRR_write(s, 0, true);
    stm32_uart_CR1_write(s, 0, true);
    stm32_uart_CR2_write(s, 0, true);
    stm32_uart_CR3_write(s, 0, true);

    stm32_uart_update_irq(s);
}

static uint64_t stm32_uart_read(void *opaque, hwaddr offset, unsigned size)
{
  Stm32Uart *s = (Stm32Uart *)opaque;
  uint32_t value;
  int start = (offset & 3) * 8;
  int length = size * 8;

  switch (offset & 0xfffffffc) {
  case USART_SR_OFFSET: return extract64(stm32_uart_SR_read(s), start, length);
  case USART_DR_OFFSET:
    stm32_uart_DR_read(s, &value);
    return extract64(value, start, length);
  case BRR_r_OFFSET: return extract64(s->BRR_r, start, length);
  case CR1_OFFSET: return extract64(s->CR1_r, start, length);
  case CR2_r_OFFSET: return extract64(s->CR2_r, start, length);
  case CR3_r_OFFSET: return extract64(s->CR3_r, start, length);
  case USART_GTPR_OFFSET: STM32_NOT_IMPL_REG(offset, size);
    return 0;
  default: STM32_BAD_REG(offset, size);
    return 0;
  }
}

static void stm32_uart_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
  Stm32Uart *s = (Stm32Uart *)opaque;
  int start = (offset & 3) * 8;
  int length = size * 8;

  stm32_rcc_check_periph_clk((Stm32Rcc *)s->stm32_rcc, s->periph);

  switch (offset & 0xfffffffc) {
  case USART_SR_OFFSET: stm32_uart_SR_write( s, deposit64(stm32_uart_SR_read(s), start, length, value));
    break;
  case USART_DR_OFFSET: stm32_uart_DR_write( s, deposit64(0, start, length, value));
    break;
  case BRR_r_OFFSET: stm32_uart_BRR_write( s, deposit64(s->BRR_r, start, length, value), false);
    break;
  case CR1_OFFSET: stm32_uart_CR1_write( s, deposit64(s->CR1_r, start, length, value), false);
    break;
  case CR2_r_OFFSET: stm32_uart_CR2_write( s, deposit64(s->CR2_r, start, length, value), false);
    break;
  case CR3_r_OFFSET: stm32_uart_CR3_write( s, deposit64(s->CR3_r, start, length, value), false);
    break;
  case USART_GTPR_OFFSET: STM32_NOT_IMPL_REG( offset, 2 );
    break;
  default: STM32_BAD_REG( offset, 2 );
    break;
  }
}

static const MemoryRegionOps stm32_uart_ops = {
    .read  = stm32_uart_read,
    .write = stm32_uart_write,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN
};

/* PUBLIC FUNCTIONS */

extern GMainContext *g_main_context_default_l;

void stm32_uart_connect(Stm32Uart *s, Chardev *chr/*, uint32_t afio_board_map*/) {

  if (chr) {
    qemu_chr_fe_init(&s->chr, chr, &error_abort);
    // qemu_chr_add_handlers(
    qemu_chr_fe_set_handlers(&s->chr, stm32_uart_can_receive,
                             stm32_uart_receive, stm32_uart_event, NULL,
                             (void *)s, g_main_context_default_l, true);
  }
  //s->afio_board_map = afio_board_map;
}

/* DEVICE INITIALIZATION */

static void stm32_uart_init(Object *obj)
{
  SysBusDevice *dev = SYS_BUS_DEVICE(obj);
  // qemu_irq *clk_irq;
  Stm32Uart *s = STM32_UART(dev);

  // s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;
  // s->stm32_gpio = (Stm32Gpio **)s->stm32_gpio_prop;
  // s->stm32_afio = (Stm32Afio *)s->stm32_afio_prop;

  memory_region_init_io(&s->iomem, OBJECT(s), &stm32_uart_ops, s, "uart", 0x03ff);
  sysbus_init_mmio(dev, &s->iomem);

  sysbus_init_irq(dev, &s->irq);

  s->rx_timer = timer_new_ns( QEMU_CLOCK_VIRTUAL, (QEMUTimerCB *)stm32_uart_rx_timer_expire, s);
  s->tx_timer = timer_new_ns( QEMU_CLOCK_VIRTUAL, (QEMUTimerCB *)stm32_uart_tx_timer_expire, s);

#ifndef STM32_UART_NO_BAUD_DELAY
  s->tx_dma_timer = timer_new_ns( QEMU_CLOCK_VIRTUAL, (QEMUTimerCB *)stm32_uart_tx_dma_timer_expire, s);
#endif

  /* Register handlers to handle updates to the USART's peripheral clock. */
  /*clk_irq = qemu_allocate_irqs(stm32_uart_clk_irq_handler, (void *)s, 1);
  stm32_rcc_set_periph_clk_irq(s->stm32_rcc, s->periph, clk_irq[0]);
  stm32_uart_reset((DeviceState *)s);*/

  return;
}

static void stm32_uart_realize(DeviceState *dev, Error **errp) {
  Stm32Uart *s = STM32_UART(dev);
  qemu_irq *clk_irq;

  clk_irq = qemu_allocate_irqs(stm32_uart_clk_irq_handler, (void *)s, 1);
  stm32_rcc_set_periph_clk_irq(s->stm32_rcc, s->periph, clk_irq[0]);
  stm32_uart_reset((DeviceState *)s);
}

void stm32_uart_set_gpio(Stm32Uart *uart, Stm32Gpio **gpio) {
  uart->stm32_gpio = gpio;
}

void stm32_uart_set_rcc(Stm32Uart *uart, Stm32Rcc *rcc) {
  uart->stm32_rcc = rcc;
}

void stm32_uart_set_afio(Stm32Uart *uart, Stm32Afio *afio) {
  uart->stm32_afio = afio;
}

void stm32_uart_set_id(Stm32Uart *uart, int uart_num ) {
  uart->id = uart_num -1;
}


static Property stm32_uart_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32Uart, periph, STM32_PERIPH_UNDEFINED),
    // DEFINE_PROP_PTR("stm32_rcc", Stm32Uart, stm32_rcc_prop),
    // DEFINE_PROP_PTR("stm32_gpio", Stm32Uart, stm32_gpio_prop),
    // DEFINE_PROP_PTR("stm32_afio", Stm32Uart, stm32_afio_prop),
    DEFINE_PROP_END_OF_LIST()};

static void stm32_uart_class_init(ObjectClass *klass, void *data)
{
  DeviceClass *dc = DEVICE_CLASS(klass);

  dc->realize = stm32_uart_realize;

  //dc->reset = stm32_uart_reset;
  device_class_set_legacy_reset( dc,stm32_uart_reset);
  // dc->props = stm32_uart_properties;
  device_class_set_props(dc, stm32_uart_properties);
}

static TypeInfo stm32_uart_info = {
    .name = "stm32-uart",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32Uart),
    .instance_init = stm32_uart_init,
    .class_init = stm32_uart_class_init
};

static void stm32_uart_register_types(void) {
  type_register_static(&stm32_uart_info);
}

type_init(stm32_uart_register_types)
