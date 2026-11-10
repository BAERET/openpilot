# Sorento UM CAN startup observation

This personal-fork diagnostic observes the unconditional CAN reinitialization
introduced in b95733cc. It does not change safety policy, CAN forwarding, retry
limits, initialization timeouts, health packet layouts or the decision to call
`can_init_all()`.

## Capture and scope

H7 Panda main firmware records events during the first 90 seconds of its MCU
microsecond timer. F4, Jungle and bootstub are excluded. Reboot the comma normally
to start a fresh Panda capture; an ignition cycle on a long-running Panda does
not reopen the window. There is no new CAN command. The existing serial-poll
schedule is unchanged; draining new bytes can still require extra SPI transfers.

`can_startup_diag: v=1` lines are drained from RAM through the existing debug
serial reader and published at warning level for tmux and normal log collection.
The hooks never print. Output is limited to two lines per second, and a complete
line is inserted atomically only when the 1024-byte UART ring has room plus a
128-byte reserve. This bounds extra serial-transfer work; it does not prove
zero timing impact. Logs still cannot be guaranteed during a persistent transport
failure or a storm of other debug messages.

The event queue has 63 usable slots. `dropped` reports queue overflow; sequence
gaps are meaningful. The first receive on each logical bus has a separate slot,
so it survives event-queue overflow. These first-receive records drain first;
output order is not event order. Sort by MCU `us`, with `seq` for equal times.

## Fields

All numeric fields except `v=1` are eight-digit hexadecimal values.

| Field | Meaning |
| --- | --- |
| `seq`, `us` | Event sequence and MCU timer at capture, not host print time |
| `kind=0/1/2/3` | Before `can_init`, after `can_init`, first RX observation, periodic snapshot |
| `core`, `bus` | Physical FDCAN index and logical CAN source after harness mapping |
| `gen` | Per-core `can_init()` generation; not the existing `canCoreResetCnt` |
| `rx` | Cumulative FIFO frames processed, including the current RX event |
| `flags` | Bit 0: first logical-bus RX; bit 1: first RX after init return; bit 2: inside init; bit 3: error cache valid |
| `ok` | After-init combined speed/init return: 0 failure, 1 success; `ff` otherwise |
| `mode`, `param`, `harness`, `power`, `mux` | Safety and routing context at capture |
| `cccr`, `ir`, `fifo`, `ie`, `ile`, `irq` | Read-only controller snapshots; IRQ bits 0/1 mean NVIC RX/TX enabled |
| `psr_cached`, `ecr_cached`, `health_us` | Values and time of the existing health read, valid only with flag bit 3 |
| `dropped` | Event-queue overflow observed when this line was drained |

PSR and ECR are not read again by this diagnostic: reads can consume error codes
and counters. Cached error evidence can predate the event or belong to an earlier
init generation. `health_us` must be checked before interpreting it as current.
CCCR/FIFO/IRQ snapshots are captured at the event itself.

A missing after-init event can also result from capture-window expiry or queue
overflow; it does not by itself prove that initialization hung. Generation pairing
assumes the current non-nested per-core init callers; changing interrupt priorities
or introducing reentrant init would require revisiting this observer.

Snapshots at 1, 3, 10, 30 and 60 seconds help describe a bus with no receive event.
RX flags distinguish an IRQ serviced during initialization from the first frame
processed after initialization returns. `can_init()` includes the normal full
initialization path; direct core recovery via `can_clear_send()` is still a
separate mechanism and is not counted by `gen`.

## Interpretation and limits

Compare normal and failed starts using the same firmware and settings. Keep the
first boot that flashes firmware separate from later ordinary boots. Collect the
first rlog segment and matching tmux, preferably including the first two minutes
so queued startup events have time to drain.

Use `core` and `bus` together: a flipped harness exchanges physical 0/2 mapping.
Use the MCU timeline for before/after-init comparisons. MCU `us` and the host
rlog clock have different origins; do not subtract them without synchronization.

No RX does not prove that the vehicle ECU was silent. FIFO/IRQ state, cached
errors, host raw CAN, SPI events and harness state must be considered together.
An RX immediately after init does not by itself prove init was necessary: time,
relay/mux changes and ECU readiness remain possible confounders. This diagnostic
does not implement a delayed retry, automatic recovery or a safety bypass.

## Why upstream introduced preservation

The b1485d40 notes describe safety command 0xDC running synchronously inside the
SPI RX DMA handler. Reapplying the mux and repeatedly entering/exiting controller
INIT could delay SPI service. Conditional preservation was intended to avoid that
work when routing and controllers already appeared healthy. The notes explicitly
do not establish that a multi-second SPI retry burst was spent in CAN init.

c09494aa extended the guarded H7 optimization to classic Hyundai modes 8 and 23.
Its commit message explicitly leaves startup timing and classic-CAN SPI-burst
resolution unproven. Both commits had zero public commit comments and no linked
pull requests when checked on 2026-11-11. This was a latency hypothesis validated
with mock tests and builds, not proof that reinitialization is unnecessary on
every vehicle.

- https://github.com/ajouatom/openpilot/commit/b1485d40e301c7f307e97353f2faf64f94278e94
- https://github.com/ajouatom/openpilot/blob/b1485d40e301c7f307e97353f2faf64f94278e94/docs/canfd_cluster_rx_forwarding.md#L203
- https://github.com/ajouatom/openpilot/commit/c09494aa4c974a58c44f841424f891db7217b435
- https://github.com/ajouatom/openpilot/blob/c09494aa4c974a58c44f841424f891db7217b435/docs/canfd_cluster_rx_forwarding.md#L258
