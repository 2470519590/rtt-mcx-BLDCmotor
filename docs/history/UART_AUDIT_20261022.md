# UART review and evidence, 2026-10-22

## Scope and changes

No flash/reset command, motor command, calibration or model upload was issued.
Only D<nonce> diagnostics were sent on COM32, selected from CH343 identity.
The capture tool constructs Serial(port=None), sets DTR/RTS low, then opens
the port. A later claim that it opened before setting those lines was wrong.
No reset command was issued; electrical reset-line behavior was not measured.
The running image has not been identified or matched to the current ELF.
This folder is not a Git repository; existing changes were not reverted.

Confirmed source defects corrected offline:

- motor_control.c staged 12 floats at indices 0..11, but angle_shadow_set_hidden
  reads 1..12. This was an out-of-bounds READ and shifted all weights, not proof
  of an out-of-bounds write into scheduler memory. Allocate 13 and store 1..12.
- cli_print_state passed an uninitialized automatic motor_state_t to a getter
  that can return without writing after a lock timeout. Use a zero-initialized
  persistent last-successful snapshot. Its age is still not exposed on wire.
- Signed g payloads need 0..3999, so stride 2500 aliases the next index.
  Change both endpoints to stride 4000; build stamp/required upload version 3003.
  This is intentionally incompatible with 3002. No new image was flashed.
- Host row_commands omitted n0 commit and h hidden-bias upload. Add both;
  regressions now decode signed fields instead of only checking magnitudes.

Existing ELF/BIN preserved in runs/uart_audit_20261022/before_build before
offline rebuilding. These are disk artifacts, not proof of the running image.

## Audit limits and remaining findings

- g validates index before its bounded write; s_hidden validates neuron and
  loops within the 13-entry destination. No scheduler-corrupting write was
  established in these paths. Static s_upload_* are zero-initialized and writes
  require s_upload_valid selected by bounded M. This is not a whole-program proof.
- y can pass invalid negative/large values, but coefficient() checks neuron
  and slot before array access. Invalid requests returning 0 are ambiguous.
- h still changes the live bias before n commits, so the entire neuron upload
  is not atomic if transmission stops mid-row. This remains unmodified.
- set_norm now checks contiguous ordering before mutating its table; rejected
  points no longer corrupt the live table. predict validates finiteness and
  conversion range before the int32 truncation.
- The p phase double-scaling is fixed: the staged phase is radians and the
  controller API now accepts radians without dividing a second time.
- CLI D/E branches intercept all uppercase D/E, not just D0/E0. Existing sweep
  D/E handlers are therefore shadowed. No change made to this protocol conflict.
- Existing command_load_test.py is not trusted: D replies are not ACK lines;
  ask-answer does not actually wait for a reply; ACK cursor logic can mispair;
  reader errors disappear and raw evidence is not preserved. It was not fixed
  or used for the new experiment. capture_raw.py has no read-start timestamps.
- rt_hw_console_output is wired and polls under interrupt disable. This does
  not guarantee successful output during HardFault. Continued fresh T frames
  from the same thread contradict that thread permanently stuck in a handler.

## Capture contract

Use tools/uart_evidence.py, tested by tools/test_uart_evidence.py. One owner,
one reader, finite read/write timeout, no buffer clear, no prefix filtering.
read(size) with timeout can return short data; size is a maximum target, not
a packet boundary. A block may span any number of lines or split any byte.

Save raw.bin plus reads.jsonl offsets, requested/returned sizes and begin/end
perf-counter timestamps, including empty reads. Reassemble before newline
framing; preserve initial/internal malformed lines and trailing partial bytes.
Check full T schema and modulo-32-bit seq/tick differences; do not infer a wrap
from truncated decimal digits. Tick units are 1 ms in the current rtconfig,
but only if it matches the running image. Read timestamps bound delivery,
not physical byte arrival. Do not infer driver capacity from source byte rate.

commands.jsonl records bytes, nonce, write interval, returned length and reply
offsets. D<nonce> has an echoed value and last-command string in current source;
match both, not an ACK count. It does not prove motor_control_command works.
No transaction is confirmed if its reply is absent, malformed or ambiguous.
Never label a timeout alone a firmware failure. Random nonces reduce stale
matches but do not establish firmware identity. Reader exceptions invalidate
the capture and are recorded. A timeout stops further requests.

## Unflashed experiment results

- runs/uart_evidence_20261022_run1: one D304073 sent as an 8-byte write;
  no D reply in saved raw stream; 252 full T frames, all 251 seq steps=1,
  all 251 tick steps=10. One trailing partial T byte retained.
- runs/uart_evidence_20261022_run2: one D688161 sent bytewise with requested
  3 ms sleeps (actual Windows sleep timing is not guaranteed); no D reply;
  288 full T frames, all 287 seq steps=1, all 287 tick steps=10.
- These short captures do not show internal T-frame loss. They do NOT prove
  all traffic is lossless, host TX reached MCU RX, UART RX works, or current
  source matches the running image. No 140-command success claim is made.
- Capture timestamps were initially monotonic_ns with coarse resolution on
  this Python/Windows setup. Script now uses perf_counter_ns; saved original
  runs are unchanged. Reply matching now accepts reads spanning a write,
  using nonce correlation rather than read-start timing. Raw streams were
  inspected separately and contain no D lines, so that fix does not reverse
  these two absence observations. No more live tests were run after this fix.

## Next diagnostic, before any flash

With motor output physically safe, obtain a bounded RAM-only debugger snapshot
of RX bytes/lines/last command before and after one D0 while retaining raw
capture. First identify the running image/symbol layout; do not use new 3003
ELF addresses on an unconfirmed old image. Do not issue run-to-breakpoint,
reset or peripheral reads with side effects. Halting perturbs control timing;
if safe stop/image identity cannot be established, do not perform it.

The existing tools/read_uart_diag.jlink is unsuitable: it connects and halts,
then reads hard-coded addresses. Those addresses do not match either current
or archived ELF symbols. Do not run that script or infer the live image from
those addresses. No J-Link session was started during this audit.

RX bytes unchanged points to host TX/wiring/MCU RX path. Bytes increase without
lines points to line assembly/terminator/loss. A complete D command and D path
execution without captured reply points to response generation/transport.
These are branch criteria, not root causes; keep the raw/snapshot evidence.

The current repair is offline only. Root cause of reported command failure is
not closed. FOC bounds, actuation, J-Link policy and SWD pins were not changed.

## Follow-up evidence

- New D0 compatibility capture: runs/uart_evidence_20261022_D0. One D0 sent,
  no D/ACK text captured. 302 full T frames, seq steps all 1, tick steps all 10.
  This bypasses dependence on a six-digit D argument, but still does not prove
  the running firmware identity or physical delivery to MCU RX. D0 is not a
  unique nonce, so any response would require extra stale-response caution.
- Reviewed SDK MCXA346_features.h defines FSL_FEATURE_LPUART_FIFO_SIZEn(x)=4
  and FSL_FEATURE_LPUART_HAS_FIFO=1. LPUART_Init enables RX/TX FIFO. This is
  source evidence, not a live register measurement. At 115200 8N1 four bytes
  arrive in about 347 us; the CLI sleeps one 1-ms tick and blocks during TX.
  Burst RX overrun is therefore a concrete risk, not proof of this failure.
- g_uart_rx_overflows only counts the 24-byte software command-buffer reset,
  not the hardware OR flag. SDK's receive routine checks/clears OR, whereas
  this CLI only polls RDRF and reads DATA. A zero software counter cannot
  exclude UART RX overrun. No speculative RX firmware patch was applied.
- Added tools/test_angle_native.c and test_angle_native.py: compile actual
  angle_shadow.c/nn_controller.c with GCC ASan, UBSan, float-cast-overflow,
  Wall/Wextra/Werror. Exercise bounded coefficient accesses, all 8x12 weights,
  invalid indices, norm-update rollback, radian phase and nonfinite inputs.
  This is host C execution, not F401/NXP hardware or a whole-CLI memory proof.
