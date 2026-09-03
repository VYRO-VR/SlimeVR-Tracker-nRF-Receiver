# Yaw drift work — receiver requirements

Scope: the parts of the VYRO yaw-drift effort that land in **this repository**
(`VYRO-VR/SlimeVR-Tracker-nRF-Receiver`). The wider effort spans four repos; this
document records only what the receiver has to provide, plus the cross-repo
context needed to know why.

## Background

Goal: increase the time between yaw / full resets on VYRO IBIS trackers
(LSM6DSV or ICM-45686 IMU, QMC6309 magnetometer).

| Repo | Role |
| --- | --- |
| `VYRO-VR/jitingcn-smol-slime-firmware` | Tracker firmware — sensor fusion, calibration |
| `VYRO-VR/SlimeVR-Tracker-nRF-Receiver` | **This repo** — dongle: ESB link, HID + console control surface |
| `VYRO-VR/preflight` | GUI (Electron + React + TS) — guided calibration flows |
| `SlimeVR/SlimeVR-Server` | Upstream server — skeleton, resets, Stay Aligned |

Fusion runs entirely on the tracker. The receiver forwards packets and carries
commands; it holds no fusion or calibration state of its own. That boundary
decides what does and does not belong here.

Two drift sources drive the work:

- **Per-unit gyro scale factor error.** Not calibrated per unit today. A 0.5%
  scale error is 1.8° of yaw per full body turn, accumulating with every turn.
  The tracker has `sens auto <x|y|z> [rev]`, but it is manual-only.
- **Magnetometer interference.** The tracker's disturbance detection compares
  field norm and dip against a reference, so a distortion that rotates the
  horizontal field without changing norm or dip passes undetected — and after
  ~12 cumulative seconds of movement near a consistent disturbed field, the
  filter *adopts* it as the new reference.

Phase 1 (gyro sensitivity calibration, GUI-driven) is first and highest ROI.
Phase 2 (mag hardening) is tracker firmware plus one new command from here.
Phase 3 (server-side cross-tracker corroboration) actuates through here.

## What already exists in this repo

The sens-auto command path is complete in both directions. Nothing needs
building for Phase 1a beyond confirming behaviour.

| Surface | Location |
| --- | --- |
| `ESB_PONG_FLAG_SENS_AUTO` (0x24) | `src/connection/esb.h:102` |
| Console: `send <id\|all> sens auto <x\|y\|z> [rev]` | `src/console_send.c:217` |
| HID opcode handler | `src/rcv_cmd.c:566` (`rcv_cmd_remote_sens_auto_hid`) |
| Shared entry point (console + HID) | `src/rcv_cmd.c:753` (`rcv_cmd_remote_sens_auto`) |
| Queue command for one tracker | `src/connection/esb.c:2901` |
| Axis + revolutions on the wire | `src/connection/esb.c:1629` — PONG ack `data[3]` = axis, `data[4..5]` = revolutions (big-endian) |
| Tracker confirmation callback | `src/connection/esb.c:2198` → `src/rcv_cmd.c:471` (`on_remote_confirm`) |

Notes for the GUI side:

- **Revolutions.** The console accepts 1–100 (`SENS_AUTO_MAX_REVOLUTIONS`,
  `src/console_send.c:31`); `0` means "use the tracker's own default". The
  10-revolution figure the calibration flow wants is well inside the ceiling.
  The HID path does not range-check revolutions itself — it only rejects
  `axis > 2` — so the caller owns that bound.
- **Command channel.** Either channel works. HID opcodes in the tracker range
  reuse the `ESB_PONG_FLAG_*` values directly (`src/rcv_hid_cmd.h`), so the HID
  opcode for sens-auto *is* `0x24`. Target `0xFF` (`RCV_HID_TARGET_ALL`) fans
  out to all active trackers.
- **CDC and HID coexist.** The dongle enumerates the console CDC ACM class
  alongside the HID interface (`src/usb.c:184`), so Preflight can drive the
  console while SlimeVR Server holds the HID interface.
- **Confirmation is delivery-only.** `on_remote_confirm` fires when the tracker
  acknowledges *receipt* of the flag. It says nothing about whether the
  calibration then succeeded. That gap is R2.

## Requirements

### R1 — Support the guided sensitivity flow (Phase 1a)

No new code expected. Verify and, if needed, correct:

1. `send <id> sens auto <axis> 10` reaches a single tracker and returns
   `RCV_HID_ST_STARTED` / `RCV_HID_ST_QUEUED` as documented.
2. The same command over HID opcode `0x24` behaves identically.
3. Command strings and opcode values are stable — Preflight keeps them in its
   own `shared/config.ts` as a single source of truth, so any change here is a
   breaking change for the GUI and must be called out.

### R2 — Report sens-cal state and result back to the host (firmware task F3) — **done**

`cal_sens.c` used to report success, failure and the computed scale only via
`printk` to the *tracker's own* serial console, so Preflight v1 would have had
to infer the outcome from phase timeouts. The tracker now sends a dedicated
report and this receiver forwards it, removing that inference.

Wire format is **stream packet type 6**, a standalone 17-byte ESB frame
(16 bytes + sequence byte). Payload is 7 bytes from `data[2]`:

| b0 | b1 | b2 | b3 | b4 | b5 | b6..b7 | b8..b9 | b10..b15 |
|---|---|---|---|---|---|---|---|---|
| 6 | id | phase | result | axis | seq | `scale_q12` | `progress` | resv |

`scale_q12` and `progress` are little-endian `uint16`; `scale = scale_q12 /
4096.0` and `progress` is integrated absolute rotation in whole degrees
(divide by 360 for a turn counter). Enum values are documented next to the
packet table in `src/hid.c` and are a wire contract — never renumber, only
append. Cadence is 2 Hz during a run plus a 10 s linger after a terminal
result.

**Receiver behaviour.** No forwarding code was needed: the standalone
length-17 path already forwards any stream type ≤ 223 to HID unchanged
(`src/connection/esb.c:2242`), so type 6 reaches the host through the existing
sequence-checked path. `src/hid.c` gained the layout documentation only.

Two things this deliberately does **not** do:

- **Type 6 is never added to the composite (`0xFE`) length table.** The tracker
  sends it standalone precisely because an unknown sub-type inside a composite
  cannot be skipped. The receiver's composite parser already fails safe on an
  unknown sub-type — it stops parsing the frame rather than mis-offsetting the
  rest (`src/connection/esb.c:2400-2408`) — and it must stay that way.
- **The earlier plan to widen the status sub-packet (type 3) was dropped.** It
  carries only 2 bytes from the tracker and the receiver already overwrites
  `pkt[4]`/`pkt[5]` with its own packet-loss counters, so it was the wrong
  vehicle.

One divergence to be aware of at the host: type 6 in the legacy SlimeVR stream
protocol meant "reduced precision quat and accel with button and sleep time".
This tracker firmware does not send that packet, so the type is free here — but
any host that still parses type 6 the legacy way will misread these reports.

### R3 — `ESB_PONG_FLAG_MAG_HOLD` / `MAG_UNHOLD` (firmware task F1) — **done**

| Flag | Value |
|---|---|
| `ESB_PONG_FLAG_MAG_HOLD` | `0x27` |
| `ESB_PONG_FLAG_MAG_UNHOLD` | `0x28` |

Neither carries PONG payload bytes — `data[3..11]` are untouched, so the normal
time-sync path applies. Both sit outside `0x30`–`0x33` (the OTA range the
tracker executes immediately), so they take the standard ~1500 ms deferred
execution. Acknowledgement is the existing generic echo in PING `data[7]`; no
new ack handling.

Plumbed the usual way:

- constants in `src/connection/esb.h`
- names in `esb_pong_flag_name` (`src/connection/esb.c`)
- allowlisted in `rcv_hid_opcode_is_pong_flag` (`src/rcv_hid_cmd.h`), which
  gates **both** the HID path and the console path, so the HID opcode is the
  flag value itself (`0x27` / `0x28`)
- console verb `send <id|all> mag hold <on|off>` (`src/console_send.c`)

On the tracker the flag makes the fusion backend skip the magnetometer update
entirely — no heading correction, no new-field adoption — and stops online mag
calibration collecting samples or committing to flash. There is **no** fusion
restart, **no** orientation glitch and **no** NVS write, and the hold is not
persisted: a tracker reboot clears it.

Why a new flag rather than the existing `MAG_ON`/`MAG_OFF` (0x19/0x1A): on the
tracker, toggling mag suspends the sensor thread, does a full fusion re-init
(visible orientation glitch) and performs a synchronous eager NVS flash write —
and reboots the tracker if the sensor was not initialised. It is not usable as a
runtime control knob. `MAG_HOLD` is the safe equivalent. **Do not expose
`MAG_ON`/`MAG_OFF` as the mechanism for any of this work.**

Side effect worth knowing about at the host: while held, the tracker reports mag
disturbance continuously, so the existing temperature-byte sign-flip shows
"disturbed" for the duration of the hold.

### R4 — Carry Phase 3 actuation

When the server gains cross-tracker magnetometer corroboration, it decides a
tracker's field is untrustworthy and actuates through
`HID → receiver → PONG → tracker` using the R3 flag. No receiver work beyond R3:
the opcode range, the fan-out target and the confirmation path all already
exist.

Mag data already flows for free and needs no change. Trackers send the
quat + calibrated device-frame mag packet (type 4) whenever mag is enabled; the
receiver forwards it, both standalone and as composite sub-type 4
(`src/connection/esb.c:2394`, 14 bytes). The HID packet layout is documented in
the table at the top of `src/hid.c`. Note this is nRF/HID only — the ESP/UDP
path carries no mag data at all.

## Non-goals

Explicitly out of scope for this repo:

- **Receiver-side storage of raw mag history.** RAM and flash cost is
  prohibitive and it is unnecessary — corroboration needs only current
  per-tracker state, which the server already holds.
- **Radio, TDMA or packet loss as a drift source.** Fusion runs on the tracker;
  the receiver only forwards. Link quality is not a yaw-drift lever.
- **Runtime mag enable/disable toggling as a correction mechanism.** See R3.
- **ODR error compensation.** Already handled on the tracker — the LSM6DSV
  driver reads `INTERNAL_FREQ_FINE` and scales ODR; the ICM variant uses an
  external 32 kHz clock.

## Cross-repo work not tracked here

Listed so this document is not mistaken for the whole plan:

- **Preflight**: add `trackerMask.rotation` to the SolarXR data feed; port the
  3D tracker preview widget from SlimeVR-Server's `IMUVisualizerWidget.tsx`
  (MIT, keep the notice); build the guided per-axis calibration flow with edge
  alignment coaching, a live turn counter and a verification spin.
- **Tracker firmware**: F1 (the `MAG_HOLD` runtime bit), F2 (rest-gated heading
  disturbance check — while at rest, treat movement of the mag-implied heading
  disagreement as a disturbance regardless of norm and dip), F3 (report
  sens-cal result), F4 (optional: reset-anchored mag reference, raw mag spike
  gate, mag temperature compensation, and removing the blend logic that trusts
  a *more* divergent calibration candidate more).
- **SlimeVR-Server**: cross-tracker mag corroboration on norm and dip
  (yaw-invariant), with per-tracker relative baselines snapshotted at Full
  Reset, biased toward rejection when ambiguous.
