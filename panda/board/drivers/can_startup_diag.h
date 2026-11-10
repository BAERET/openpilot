#pragma once

// H7 Panda startup observation only. No health/SPI wire-format change.
// IRQ hooks only capture into RAM. The main loop formats/drains bounded logs.
#if defined(STM32H7) && !defined(PANDA_JUNGLE)
#include "power_saving_declarations.h"

#define CAN_STARTUP_DIAG_CAPACITY 64U
#define CAN_STARTUP_DIAG_WINDOW_US 90000000U
#define CAN_STARTUP_DIAG_LINE_SIZE 448U

typedef struct {
  uint32_t seq, time_us, generation, rx_count;
  uint32_t cccr, ir, fifo, ie, ile;
  uint32_t psr_cached, ecr_cached, health_time_us;
  uint16_t mode, param;
  uint8_t kind, core, bus, harness, irq, power, flags, result, mux;
} can_startup_diag_event_t;

static can_startup_diag_event_t can_startup_diag_events[CAN_STARTUP_DIAG_CAPACITY];
static can_startup_diag_event_t can_startup_diag_first_bus[3];
static volatile uint32_t can_startup_diag_read = 0U, can_startup_diag_write = 0U;
static uint32_t can_startup_diag_sequence = 0U;
static volatile uint32_t can_startup_diag_dropped = 0U;
static uint32_t can_startup_diag_generation[3] = {0U};
static uint32_t can_startup_diag_psr[3] = {0U}, can_startup_diag_ecr[3] = {0U}, can_startup_diag_health_us[3] = {0U};
static bool can_startup_diag_health_valid[3] = {false};
static volatile bool can_startup_diag_in_init[3] = {false}, can_startup_diag_await_rx[3] = {false};
static volatile uint8_t can_startup_diag_seen_bus = 0U, can_startup_diag_first_pending = 0U;
static volatile bool can_startup_diag_active = true;
static uint32_t can_startup_diag_last_drain_us = 0U;
static uint8_t can_startup_diag_checkpoint = 0U;

static bool can_startup_diag_enabled(void) {
  if (can_startup_diag_active && (microsecond_timer_get() >= CAN_STARTUP_DIAG_WINDOW_US)) {
    // Latch off permanently, including after the 32-bit timer wraps.
    can_startup_diag_active = false;
  }
  return can_startup_diag_active;
}

// Caller holds a short critical section. PSR/ECR must NOT be read here:
// PSR error codes and ECR.CEL have read side effects.
static void can_startup_diag_capture(uint8_t core, uint8_t kind, uint8_t flags, uint8_t result) {
  static const IRQn_Type rx_irq[3] = {FDCAN1_IT0_IRQn, FDCAN2_IT0_IRQn, FDCAN3_IT0_IRQn};
  static const IRQn_Type tx_irq[3] = {FDCAN1_IT1_IRQn, FDCAN2_IT1_IRQn, FDCAN3_IT1_IRQn};
  const FDCAN_GlobalTypeDef *can = CANIF_FROM_CAN_NUM(core);
  can_startup_diag_sequence++;
  const can_startup_diag_event_t event = {
    .seq = can_startup_diag_sequence, .time_us = microsecond_timer_get(),
    .generation = can_startup_diag_generation[core], .rx_count = can_health[core].total_rx_cnt,
    .cccr = can->CCCR, .ir = can->IR, .fifo = can->RXF0S, .ie = can->IE, .ile = can->ILE,
    .psr_cached = can_startup_diag_psr[core], .ecr_cached = can_startup_diag_ecr[core],
    .health_time_us = can_startup_diag_health_us[core],
    .mode = current_safety_mode, .param = current_safety_param,
    .kind = kind, .core = core, .bus = BUS_NUM_FROM_CAN_NUM(core), .harness = harness.status,
    .irq = (NVIC_GetEnableIRQ(rx_irq[core]) != 0U ? 1U : 0U) | (NVIC_GetEnableIRQ(tx_irq[core]) != 0U ? 2U : 0U),
    .power = (uint8_t)power_save_status,
    .flags = flags | (can_startup_diag_in_init[core] ? 4U : 0U) | (can_startup_diag_health_valid[core] ? 8U : 0U),
    .result = result,
    .mux = applied_can_mode,
  };
  if ((kind == 2U) && ((flags & 1U) != 0U)) {
    // Preserve the true first logical-bus receive even if the event queue fills.
    can_startup_diag_first_bus[event.bus] = event;
    can_startup_diag_first_pending |= (uint8_t)(1U << event.bus);
  } else {
    const uint32_t next = (can_startup_diag_write + 1U) % CAN_STARTUP_DIAG_CAPACITY;
    if (next == can_startup_diag_read) {
      can_startup_diag_dropped++;
    } else {
      can_startup_diag_events[can_startup_diag_write] = event;
      can_startup_diag_write = next;
    }
  }
}

static void can_startup_diag_health(uint8_t core, uint32_t psr, uint32_t ecr) {
  if ((core < 3U) && can_startup_diag_enabled()) {
    ENTER_CRITICAL();
    can_startup_diag_psr[core] = psr;
    can_startup_diag_ecr[core] = ecr;
    can_startup_diag_health_us[core] = microsecond_timer_get();
    can_startup_diag_health_valid[core] = true;
    EXIT_CRITICAL();
  }
}

static void can_startup_diag_init_before(uint8_t core) {
  if ((core < 3U) && can_startup_diag_enabled()) {
    ENTER_CRITICAL();
    can_startup_diag_generation[core]++;
    can_startup_diag_in_init[core] = true;
    can_startup_diag_await_rx[core] = false;
    can_startup_diag_capture(core, 0U, 0U, 0xFFU);
    EXIT_CRITICAL();
  }
}

static void can_startup_diag_init_after(uint8_t core, bool result) {
  if ((core < 3U) && can_startup_diag_enabled()) {
    ENTER_CRITICAL();
    can_startup_diag_in_init[core] = false;
    can_startup_diag_capture(core, 1U, 0U, result ? 1U : 0U);
    can_startup_diag_await_rx[core] = true;
    EXIT_CRITICAL();
  }
}

static void can_startup_diag_rx(uint8_t core) {
  const uint8_t bus = BUS_NUM_FROM_CAN_NUM(core);
  if ((bus < 3U) && can_startup_diag_active &&
      (can_startup_diag_await_rx[core] || ((can_startup_diag_seen_bus & (1U << bus)) == 0U)) &&
      can_startup_diag_enabled()) {
    ENTER_CRITICAL();
    const bool first_bus = (can_startup_diag_seen_bus & (1U << bus)) == 0U;
    const bool after_init = can_startup_diag_await_rx[core] && !can_startup_diag_in_init[core];
    can_startup_diag_seen_bus |= (uint8_t)(1U << bus);
    if (after_init) can_startup_diag_await_rx[core] = false;
    can_startup_diag_capture(core, 2U, (first_bus ? 1U : 0U) | (after_init ? 2U : 0U), 0xFFU);
    EXIT_CRITICAL();
  }
}

static uint32_t can_startup_diag_text(char *line, uint32_t pos, const char *text) {
  while ((*text != '\0') && (pos < (CAN_STARTUP_DIAG_LINE_SIZE - 2U))) line[pos++] = *text++;
  return pos;
}

static uint32_t can_startup_diag_hex(char *line, uint32_t pos, const char *key, uint32_t value) {
  static const char hex[] = "0123456789abcdef";
  pos = can_startup_diag_text(line, pos, key);
  for (int shift = 28; (shift >= 0) && (pos < (CAN_STARTUP_DIAG_LINE_SIZE - 2U)); shift -= 4) {
    line[pos++] = hex[(value >> (unsigned int)shift) & 0xFU];
  }
  return pos;
}

// Main-loop only. Format outside the critical section, then enqueue atomically.
static void can_startup_diag_tick(void) {
  if (!can_startup_diag_active && (can_startup_diag_read == can_startup_diag_write) &&
      (can_startup_diag_first_pending == 0U)) return;
  static const uint32_t checkpoints_us[] = {1000000U, 3000000U, 10000000U, 30000000U, 60000000U};
  const uint32_t now = microsecond_timer_get();
  if (can_startup_diag_enabled() && (can_startup_diag_checkpoint < 5U) &&
      (now >= checkpoints_us[can_startup_diag_checkpoint])) {
    ENTER_CRITICAL();
    for (uint8_t core = 0U; core < 3U; core++) can_startup_diag_capture(core, 3U, 0U, 0xFFU);
    can_startup_diag_checkpoint++;
    EXIT_CRITICAL();
  }
  if (((can_startup_diag_read != can_startup_diag_write) || (can_startup_diag_first_pending != 0U)) &&
      (get_ts_elapsed(now, can_startup_diag_last_drain_us) >= 500000U)) {
    can_startup_diag_event_t event;
    uint8_t first_bus = 0xFFU;
    uint32_t dropped;
    ENTER_CRITICAL();
    for (uint8_t bus = 0U; bus < 3U; bus++) {
      if ((can_startup_diag_first_pending & (1U << bus)) != 0U) { first_bus = bus; break; }
    }
    event = first_bus < 3U ? can_startup_diag_first_bus[first_bus] : can_startup_diag_events[can_startup_diag_read];
    dropped = can_startup_diag_dropped;
    EXIT_CRITICAL();
    char line[CAN_STARTUP_DIAG_LINE_SIZE];
    uint32_t len = can_startup_diag_text(line, 0U, "can_startup_diag: v=1");
    len = can_startup_diag_hex(line, len, " seq=", event.seq);
    len = can_startup_diag_hex(line, len, " us=", event.time_us);
    len = can_startup_diag_hex(line, len, " kind=", event.kind);
    len = can_startup_diag_hex(line, len, " core=", event.core);
    len = can_startup_diag_hex(line, len, " bus=", event.bus);
    len = can_startup_diag_hex(line, len, " gen=", event.generation);
    len = can_startup_diag_hex(line, len, " rx=", event.rx_count);
    len = can_startup_diag_hex(line, len, " flags=", event.flags);
    len = can_startup_diag_hex(line, len, " ok=", event.result);
    len = can_startup_diag_hex(line, len, " mode=", event.mode);
    len = can_startup_diag_hex(line, len, " param=", event.param);
    len = can_startup_diag_hex(line, len, " harness=", event.harness);
    len = can_startup_diag_hex(line, len, " power=", event.power);
    len = can_startup_diag_hex(line, len, " mux=", event.mux);
    len = can_startup_diag_hex(line, len, " cccr=", event.cccr);
    len = can_startup_diag_hex(line, len, " ir=", event.ir);
    len = can_startup_diag_hex(line, len, " fifo=", event.fifo);
    len = can_startup_diag_hex(line, len, " ie=", event.ie);
    len = can_startup_diag_hex(line, len, " ile=", event.ile);
    len = can_startup_diag_hex(line, len, " irq=", event.irq);
    len = can_startup_diag_hex(line, len, " psr_cached=", event.psr_cached);
    len = can_startup_diag_hex(line, len, " ecr_cached=", event.ecr_cached);
    len = can_startup_diag_hex(line, len, " health_us=", event.health_time_us);
    len = can_startup_diag_hex(line, len, " dropped=", dropped);
    line[len++] = '\r'; line[len++] = '\n';
    ENTER_CRITICAL();
    const uint32_t used = (uart_ring_debug.w_ptr_rx + uart_ring_debug.rx_fifo_size - uart_ring_debug.r_ptr_rx) % uart_ring_debug.rx_fifo_size;
    if ((len + 128U) < (uart_ring_debug.rx_fifo_size - used)) {
      for (uint32_t i = 0U; i < len; i++) {
        uart_ring_debug.elems_rx[uart_ring_debug.w_ptr_rx] = (uint8_t)line[i];
        uart_ring_debug.w_ptr_rx = (uart_ring_debug.w_ptr_rx + 1U) % uart_ring_debug.rx_fifo_size;
      }
      if (first_bus < 3U) {
        can_startup_diag_first_pending &= (uint8_t)~(1U << first_bus);
      } else {
        can_startup_diag_read = (can_startup_diag_read + 1U) % CAN_STARTUP_DIAG_CAPACITY;
      }
      can_startup_diag_last_drain_us = now;
    }
    EXIT_CRITICAL();
  }
}
#else
#define can_startup_diag_health(core, psr, ecr) ((void)0)
#define can_startup_diag_init_before(core) ((void)0)
#define can_startup_diag_init_after(core, result) ((void)0)
#define can_startup_diag_rx(core) ((void)0)
#define can_startup_diag_tick() ((void)0)
#endif
