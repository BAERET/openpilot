"""Compile the complete startup observer with mock registers and UART storage.

These checks cover event evidence, bounded storage and output. They do not
emulate MCU timing, interrupt preemption, CAN peripherals or a physical UART.
"""
import os
from pathlib import Path
import re
import shlex
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "panda/board/drivers/can_startup_diag.h"

HARNESS = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#undef NDEBUG
#include <assert.h>
#define STM32H7
typedef int IRQn_Type;
enum { FDCAN1_IT0_IRQn, FDCAN2_IT0_IRQn, FDCAN3_IT0_IRQn,
       FDCAN1_IT1_IRQn, FDCAN2_IT1_IRQn, FDCAN3_IT1_IRQn };
// Deliberately no PSR/ECR members: a new direct read cannot compile here.
typedef struct { uint32_t CCCR, IR, RXF0S, IE, ILE; } FDCAN_GlobalTypeDef;
static FDCAN_GlobalTypeDef regs[3];
static uint8_t mapping[3] = {0U, 1U, 2U};
#define CANIF_FROM_CAN_NUM(i) (&regs[i])
#define BUS_NUM_FROM_CAN_NUM(i) mapping[i]
static struct { uint32_t total_rx_cnt; } can_health[3];
static struct { uint8_t status; } harness = {1U};
static uint16_t current_safety_mode = 8U, current_safety_param = 12U;
static uint8_t applied_can_mode = 0U;
int power_save_status = 0;
static uint32_t now_us = 10U, irq_enabled[6] = {1U, 1U, 1U, 1U, 1U, 1U};
static int critical = 0, entries = 0;
static uint32_t microsecond_timer_get(void) { return now_us; }
static uint32_t get_ts_elapsed(uint32_t now, uint32_t before) { return now - before; }
static uint32_t NVIC_GetEnableIRQ(IRQn_Type irq) { return irq_enabled[irq]; }
#define ENTER_CRITICAL() do { assert(critical == 0); critical++; entries++; } while (0)
#define EXIT_CRITICAL() do { assert(critical == 1); critical--; } while (0)
static uint8_t uart_data[1024];
static struct {
  uint32_t w_ptr_rx, r_ptr_rx, rx_fifo_size;
  uint8_t *elems_rx;
} uart_ring_debug = {0U, 0U, 1024U, uart_data};
#include "drivers/can_startup_diag.h"

static uint32_t queued(void) {
  return (can_startup_diag_write + CAN_STARTUP_DIAG_CAPACITY - can_startup_diag_read) % CAN_STARTUP_DIAG_CAPACITY;
}
static uint32_t uart_used(void) {
  return (uart_ring_debug.w_ptr_rx + 1024U - uart_ring_debug.r_ptr_rx) % 1024U;
}
static uint32_t take_uart(char *out) {
  uint32_t n = uart_used();
  for (uint32_t i = 0U; i < n; i++) out[i] = (char)uart_data[(uart_ring_debug.r_ptr_rx + i) % 1024U];
  out[n] = '\0';
  uart_ring_debug.r_ptr_rx = uart_ring_debug.w_ptr_rx;
  return n;
}

static void init_evidence(void) {
  can_startup_diag_init_before(1U);
  assert(queued() == 1U && can_startup_diag_generation[1] == 1U);
  const can_startup_diag_event_t pre = can_startup_diag_events[0];
  assert(pre.seq == 1U && pre.time_us == 10U && pre.kind == 0U);
  assert(pre.core == 1U && pre.bus == 1U && pre.generation == 1U);
  assert(pre.flags == 4U && pre.result == 255U && pre.mode == 8U && pre.param == 12U);
  assert(can_startup_diag_in_init[1] && !can_startup_diag_await_rx[1]);
  now_us = 20U;
  can_startup_diag_init_after(1U, true);
  assert(queued() == 2U && !can_startup_diag_in_init[1] && can_startup_diag_await_rx[1]);
  const can_startup_diag_event_t post = can_startup_diag_events[1];
  assert(post.seq == 2U && post.kind == 1U && post.flags == 0U && post.result == 1U);
  assert(post.generation == pre.generation && post.time_us == 20U);
  can_startup_diag_init_before(1U);
  can_startup_diag_init_after(1U, false);
  assert(can_startup_diag_events[2].generation == 2U && can_startup_diag_events[3].generation == 2U);
  assert(can_startup_diag_events[3].result == 0U && can_startup_diag_events[3].seq == 4U);
  const uint32_t seq = can_startup_diag_sequence;
  can_startup_diag_init_before(3U); can_startup_diag_init_after(255U, true);
  can_startup_diag_health(3U, 1U, 2U);
  assert(can_startup_diag_sequence == seq && critical == 0);
}

static void first_rx_mapping_and_init(void) {
  mapping[0] = 2U; mapping[2] = 0U; harness.status = 2U;
  irq_enabled[FDCAN1_IT1_IRQn] = 0U;
  can_startup_diag_health(0U, 0x1234U, 0x5678U);
  can_startup_diag_init_before(0U);
  can_health[0].total_rx_cnt = 1U;
  now_us = 21U; can_startup_diag_rx(0U);
  assert(can_startup_diag_first_pending == 4U && queued() == 1U);
  const can_startup_diag_event_t first = can_startup_diag_first_bus[2];
  assert(first.core == 0U && first.bus == 2U && first.harness == 2U && first.irq == 1U);
  assert(first.kind == 2U && first.flags == (1U | 4U | 8U) && first.rx_count == 1U);
  assert(first.psr_cached == 0x1234U && first.ecr_cached == 0x5678U);
  assert(first.health_time_us == 10U && first.time_us == 21U);
  // Receiving during init must not consume the first post-init evidence.
  can_startup_diag_init_after(0U, true);
  can_health[0].total_rx_cnt = 2U;
  now_us = 30U; can_startup_diag_rx(0U);
  const can_startup_diag_event_t after = can_startup_diag_events[2];
  assert(after.kind == 2U && after.flags == (2U | 8U) && after.time_us == 30U);
  assert(after.rx_count == 2U && !can_startup_diag_await_rx[0]);
  const uint32_t seq = can_startup_diag_sequence;
  can_startup_diag_rx(0U);
  assert(can_startup_diag_sequence == seq);
  can_startup_diag_init_before(2U); can_startup_diag_init_after(2U, true);
  can_startup_diag_rx(2U);
  assert(can_startup_diag_first_bus[0].core == 2U);
  assert(can_startup_diag_first_bus[0].flags == 3U);
  assert(can_startup_diag_first_pending == 5U && critical == 0);
}

static void cached_health_and_readonly(void) {
  for (uint8_t i = 0U; i < 3U; i++) {
    regs[i] = (FDCAN_GlobalTypeDef){0x12340001U + i, 0x12000002U + i, 0x23000003U + i, 0x34000004U + i, 0x45000005U + i};
  }
  FDCAN_GlobalTypeDef before[3]; memcpy(before, regs, sizeof(regs));
  can_startup_diag_init_before(2U);
  assert(can_startup_diag_events[0].psr_cached == 0U && !(can_startup_diag_events[0].flags & 8U));
  now_us = 77U; can_startup_diag_health(2U, 0xabcdef12U, 0x34567890U);
  now_us = 88U; can_startup_diag_init_after(2U, true);
  const can_startup_diag_event_t event = can_startup_diag_events[1];
  assert(event.flags == 8U && event.psr_cached == 0xabcdef12U && event.ecr_cached == 0x34567890U);
  assert(event.health_time_us == 77U && event.time_us == 88U);
  assert(event.cccr == before[2].CCCR && event.ir == before[2].IR && event.fifo == before[2].RXF0S);
  assert(event.ie == before[2].IE && event.ile == before[2].ILE);
  can_startup_diag_rx(2U);
  now_us = 1000000U; can_startup_diag_tick();
  assert(memcmp(before, regs, sizeof(regs)) == 0 && critical == 0 && entries > 0);
}

static void full_event_queue(void) {
  mapping[0] = 2U; mapping[2] = 0U;
  for (unsigned i = 0U; i < CAN_STARTUP_DIAG_CAPACITY + 6U; i++) can_startup_diag_init_before(0U);
  assert(queued() == CAN_STARTUP_DIAG_CAPACITY - 1U);
  assert(can_startup_diag_dropped == 7U);
  const uint32_t first_seq = can_startup_diag_sequence + 1U;
  can_startup_diag_rx(0U);
  assert(can_startup_diag_first_pending == 4U && can_startup_diag_first_bus[2].seq == first_seq);
  assert(can_startup_diag_dropped == 7U);
  now_us = 500000U; can_startup_diag_tick();
  char text[1025]; take_uart(text);
  assert(strstr(text, " bus=00000002") && strstr(text, " core=00000000"));
  assert(strstr(text, " dropped=00000007\r\n"));
  assert(can_startup_diag_first_pending == 0U && queued() == CAN_STARTUP_DIAG_CAPACITY - 1U);
  assert(critical == 0);
}

static void uart_backpressure_and_atomic_line(bool reserved_first) {
  can_startup_diag_init_before(0U);
  if (reserved_first) can_startup_diag_rx(0U);
  memset(uart_data, 0xa5, sizeof(uart_data));
  uart_ring_debug.r_ptr_rx = 1U;
  uint8_t old_uart[1024]; memcpy(old_uart, uart_data, sizeof(uart_data));
  now_us = 500000U; can_startup_diag_tick();
  assert(queued() == 1U && can_startup_diag_read == 0U && can_startup_diag_last_drain_us == 0U);
  assert(can_startup_diag_first_pending == (reserved_first ? 1U : 0U));
  assert(uart_ring_debug.w_ptr_rx == 0U && memcmp(old_uart, uart_data, sizeof(uart_data)) == 0);
  uart_ring_debug.r_ptr_rx = 0U;
  can_startup_diag_tick();
  char text[1025]; const uint32_t n = take_uart(text);
  assert(n > 0U && n <= CAN_STARTUP_DIAG_LINE_SIZE && text[n - 2U] == '\r' && text[n - 1U] == '\n');
  assert(strstr(text, reserved_first ? " seq=00000002" : " seq=00000001"));
  assert(strncmp(text, "can_startup_diag: v=1", 20U) == 0);
  assert(strstr(text, " dropped=00000000\r\n") && queued() == (reserved_first ? 1U : 0U));
  assert(can_startup_diag_first_pending == 0U);
  assert(can_startup_diag_last_drain_us == now_us && critical == 0);
}

static void complete_line_and_rate(void) {
  can_startup_diag_checkpoint = 5U;
  can_startup_diag_init_before(0U); can_startup_diag_init_after(0U, false);
  can_startup_diag_event_t *event = &can_startup_diag_events[0];
  event->seq = event->time_us = event->generation = event->rx_count = UINT32_MAX;
  event->cccr = event->ir = event->fifo = event->ie = event->ile = UINT32_MAX;
  event->psr_cached = event->ecr_cached = event->health_time_us = UINT32_MAX;
  event->mode = event->param = UINT16_MAX;
  event->kind = event->core = event->bus = event->harness = event->irq = event->power = event->flags = event->result = event->mux = UINT8_MAX;
  can_startup_diag_dropped = UINT32_MAX;
  now_us = 499999U; can_startup_diag_tick(); assert(uart_used() == 0U && queued() == 2U);
  now_us = 500000U; can_startup_diag_tick();
  char text[1025]; const uint32_t len = take_uart(text);
  assert(len < CAN_STARTUP_DIAG_LINE_SIZE && strlen(text) == len);
  assert(strstr(text, " psr_cached=ffffffff") && strstr(text, " ecr_cached=ffffffff"));
  assert(strstr(text, " health_us=ffffffff dropped=ffffffff\r\n"));
  assert(queued() == 1U);
  now_us = 999999U; can_startup_diag_tick(); assert(uart_used() == 0U && queued() == 1U);
  now_us = 1000000U; can_startup_diag_tick(); assert(uart_used() == len && queued() == 0U);
  assert(critical == 0);
}

static void stop_window_and_wrap(void) {
  now_us = 89999999U;
  can_startup_diag_init_before(0U);
  assert(can_startup_diag_sequence == 1U && can_startup_diag_active);
  now_us = 90000000U;
  can_startup_diag_init_after(0U, true); can_startup_diag_rx(1U);
  can_startup_diag_health(0U, 12U, 34U); can_startup_diag_tick();
  assert(!can_startup_diag_active && can_startup_diag_sequence == 1U);
  assert(!can_startup_diag_health_valid[0] && can_startup_diag_checkpoint == 0U);
  // Already captured records may drain after 90 seconds, but capture never rearms.
  assert(queued() == 0U);
  now_us = UINT32_MAX; can_startup_diag_tick();
  now_us = 12U;
  can_startup_diag_init_before(0U); can_startup_diag_init_after(0U, false);
  can_startup_diag_rx(2U); can_startup_diag_health(0U, 56U, 78U); can_startup_diag_tick();
  assert(!can_startup_diag_active && can_startup_diag_sequence == 1U && queued() == 0U);
  assert(can_startup_diag_generation[0] == 1U && can_startup_diag_seen_bus == 0U && critical == 0);
}

int main(int argc, char **argv) {
  assert(argc == 2);
  if (!strcmp(argv[1], "init")) init_evidence();
  else if (!strcmp(argv[1], "rx")) first_rx_mapping_and_init();
  else if (!strcmp(argv[1], "readonly")) cached_health_and_readonly();
  else if (!strcmp(argv[1], "queue")) full_event_queue();
  else if (!strcmp(argv[1], "uart")) uart_backpressure_and_atomic_line(false);
  else if (!strcmp(argv[1], "uart-first")) uart_backpressure_and_atomic_line(true);
  else if (!strcmp(argv[1], "line-rate")) complete_line_and_rate();
  else if (!strcmp(argv[1], "window")) stop_window_and_wrap();
  else assert(false);
  puts("startup observer test passed");
  return 0;
}
'''


def compile_c(tmp_path, source_text, name="startup"):
  source = tmp_path / f"{name}.c"
  executable = tmp_path / (f"{name}.exe" if os.name == "nt" else name)
  source.write_text(source_text, encoding="utf-8")
  compiler = shlex.split(os.environ.get("CC", "cc"))
  subprocess.run([*compiler, "-std=gnu11", "-O2", "-Wall", "-Wextra", "-Werror",
                  "-I", str(ROOT / "panda/board"), str(source), "-o", str(executable)], check=True, timeout=120)
  return executable


@pytest.fixture(scope="module")
def observer_binary(tmp_path_factory):
  return compile_c(tmp_path_factory.mktemp("startup_observer"), HARNESS)


@pytest.mark.parametrize("scenario", ["init", "rx", "readonly", "queue", "uart", "uart-first", "line-rate", "window"])
def test_complete_production_observer(observer_binary, scenario):
  subprocess.run([str(observer_binary), scenario], check=True, timeout=10)


def test_observer_never_reads_side_effectful_status_registers():
  source = HEADER.read_text(encoding="utf-8")
  source = re.sub(r"//[^\n]*|/\*.*?\*/", "", source, flags=re.DOTALL)
  assert not re.search(r"(?:->|\.)\s*(?:PSR|ECR)\b", source)


@pytest.mark.parametrize("defines", ["", "#define STM32H7\n#define PANDA_JUNGLE\n"])
def test_non_h7_and_jungle_are_noops(tmp_path, defines):
  source = defines + r'''
#include "drivers/can_startup_diag.h"
int main(void) {
  can_startup_diag_health(no_hardware_core, no_psr_register, no_ecr_register);
  can_startup_diag_init_before(no_hardware_core);
  can_startup_diag_init_after(no_hardware_core, no_result);
  can_startup_diag_rx(no_hardware_core);
  can_startup_diag_tick();
  return 0;
}
'''
  executable = compile_c(tmp_path, source, "excluded")
  subprocess.run([str(executable)], check=True, timeout=10)
