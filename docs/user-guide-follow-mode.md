# Follow Mode — Pilot's Guide

This guide is for a pilot who already has two aircraft running FormationFlight
(FF) and INAV, can see each other on the radar HUD, and wants to turn on
autonomous **Follow Mode** — where one aircraft (the "follower") automatically
flies a formation slot behind/beside/above another aircraft (the "leader").

Everything here is done from FF's web UI (the **Follow** panel, reachable by
joining the aircraft's WiFi AP and browsing to `192.168.4.1`) and INAV
Configurator. You don't need to read any spec or source code to follow this
guide.

**Scope:** this only covers the *follower* side. The leader aircraft needs
nothing beyond ordinary FF/INAV setup — it just needs to be broadcasting its
position, which it's already doing if you can see it on the radar HUD.

**Craft type:** the follower must be a multirotor. Fixed-wing follower
support isn't implemented.

---

## Table of contents

1. [How it works, in one paragraph](#1-how-it-works-in-one-paragraph)
2. [One-time INAV setup on the follower](#2-one-time-inav-setup-on-the-follower)
3. [The Follow panel, field by field](#3-the-follow-panel-field-by-field)
4. [Apply vs. Save to EEPROM](#4-apply-vs-save-to-eeprom)
5. [Flying it: engaging Follow Mode](#5-flying-it-engaging-follow-mode)
6. [Showing Follow status on your OSD (GVARs)](#6-showing-follow-status-on-your-osd-gvars)
7. [Trimming the slot live with RC channels](#7-trimming-the-slot-live-with-rc-channels)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. How it works, in one paragraph

While Follow Mode is engaged, FF continuously computes a 3D position —
a configurable offset (distance behind/ahead, left/right, above/below) from
wherever the leader currently is and however it's currently pointed — and
streams that position to the follower's own flight controller over MSP, at
up to 4 times a second. On the flight-controller side this rides entirely on
two stock INAV modes, **`NAV POSHOLD`** and **`GCS NAV`**, which together make
INAV treat FF's streamed position as a moving "fly here and hold" target.
FF never touches motors or bypasses INAV's own failsafe/RTH — it only ever
proposes a target position; INAV's own flight-control loop decides how to get
there and remains the authority on safety.

---

## 2. One-time INAV setup on the follower

Do this once per follower aircraft, in INAV Configurator, before you ever try
Follow Mode in the air.

### 2.1 Assign switches in the Modes tab

Follow Mode is gated on two INAV flight modes being active **together**:

| INAV mode | Purpose |
|---|---|
| `NAV POSHOLD` | Tells INAV to hold/fly to a position rather than obey raw stick input |
| `GCS NAV` | Tells INAV to take that position from an external device over MSP (that's FF) — this is INAV's stock "follow-me" hook |
| `HEADING HOLD` | If you plan to use [heading control](#heading-panel) modes other than "Off," also assign **`HEADING HOLD`** (sometimes labeled `MAG` in older INAV) to a switch — see the Heading section below for why. This is only really needed when the follower aircraft is a rotorcraft which can travel in a direction that is different from the direction of travel |

In Configurator's **Modes** tab, assign both `NAV POSHOLD`, `GCS NAV`, and `HEADING HOLD` to
an AUX switch. The simplest setup is to put all three on the *same* switch/range so
one flip engages the three  together — that single flip is what turns Follow Mode
on and off in flight (§5).


### 2.2 Confirm prerequisites

- The follower flies `NAV POSHOLD` cleanly on its own (calibrated compass,
  reliable GPS 3D fix) — verify this normally, with Follow Mode not involved,
  before ever combining it with Follow Mode.
- If your craft does not have a calibrated compass (along with the GPS), fly back and forth in a straight line to help the craft determine its orientation. You can confirm this by making sure the home arrow is point in the correct direction in the OSD.
- FF is connected to the flight controller over its configured MSP UART (the
  same link already used for the radar HUD).
- Both aircraft can already see each other on the radar HUD — i.e. peer
  telemetry already works end-to-end.

---

## 3. The Follow panel, field by field

Open the **Follow** panel in FF's web UI. It's organized into the sections
below.

### Status

Read-only. Shows whether the follow gate is currently active, the current
lock state (`IDLE` / `ACQUIRING` / `LOCKED` / `LOCKED_HOLDING`), which peer is
currently locked, and the last target position actually sent to the flight
controller. This is the fastest way to check things are working from your
phone/laptop before or after a flight — see [§8](#8-troubleshooting).

### Follow Slot

This is where you set *where* the follower flies relative to the leader. It
has two views — a toggle in the top-right switches between them, and they
both edit the exact same underlying values:

- **Friendly grid (default view):** three axis pickers —
  **Longitudinal** (Ahead / Center / Behind), **Lateral** (Left / Center /
  Right), **Vertical** (Above / Level / Below) — each paired with a **Gap**
  distance in meters. E.g. "Behind" + "15 m" means 15 meters behind the
  leader.
- **Advanced (raw offsets):** the same slot as three signed meter values —
  **Longitudinal Offset** (+ahead / −behind), **Lateral Offset** (+right /
  −left), **Vertical Offset** (+above / −below). Useful if you want a value
  that doesn't fit the Ahead/Behind/Center labels' mental model, or you're
  scripting config changes.

See [§9's diagram](#9-slot-geometry-diagram) below for a picture of what each
axis means, and how the leader's own heading rotates the whole slot with it.

The factory default is **Behind 15 m, Center, Above 10 m** ("chase-high") —
chosen because it keeps the follower clear of the leader's rotor wash while
still being a sane geometry to bench-test with the leader sitting still on
the ground.

### Safety Bounds

| Field | Meaning |
|---|---|
| **Min Separation** | Smallest allowed straight-line (3D) distance from the leader. A slot that works out to less than this is rejected — this exists specifically to forbid an accidental "sit exactly on top of the leader" (0,0,0) configuration. |
| **Min Vertical Separation (when stacked)** | When the slot is directly above/below the leader with no horizontal offset at all, the minimum vertical gap required. This is set well above the pure physical clearance you'd expect (13 m by default) because both aircrafts' GPS altitude has real error, and a stacked slot is the one geometry where that error alone could cause a collision. |
| **Max Target Distance** | If the leader is ever reported farther away than this, Follow Mode stops emitting rather than letting the follower chase indefinitely across an implausible distance (e.g. a bad GPS reading). |
| **Min Altitude Floor** | The lowest altitude (home-relative) the follower will ever be commanded to, regardless of what the leader is doing. If the leader descends, lands, or you've configured a "Below" slot, the commanded altitude is clamped up to this floor rather than letting the follower fly toward or below ground level. It's a clamp, not a full stop — the follower keeps tracking the leader horizontally and just holds at the floor altitude. |
| **Min Course Speed** | Below this ground speed, the leader's reported heading is too noisy to trust for orienting the slot (a stationary or near-stationary GPS course jitters unpredictably). Below this speed, FF freezes the slot's orientation at the last heading it trusted, rather than following that jitter. |

The web UI blocks **Apply**/**Save** if your Follow Slot values violate Min
Separation or Min Vertical Separation — you'll see an inline error rather
than being able to save an unsafe geometry.

### Heading

<a name="heading-panel"></a>

Controls which way the follower's *nose* points while following — independent
of which direction it's actually flying, since a multirotor can translate
sideways/backwards without yawing.

| Mode | Behavior |
|---|---|
| **Off** | Don't touch heading at all — leave it wherever pilot stick input / previous mode left it. (Probably best when the follower is a fixed-wing) |
| **Direction of Travel** | Nose points the way the leader is currently heading (not necessarily the way the follower itself is moving). |
| **Point at Leader** | Nose always points toward the leader's live position — useful for keeping a camera aimed at the leader regardless of formation position. |
| **Fixed Compass Heading** | Holds an absolute compass heading you enter (0° = North, 90° = East), regardless of the leader. |
| **Offset From Course** | Like Direction of Travel, but with a configurable degree offset added — e.g. +90° points the nose 90° clockwise from the leader's heading. |

**Important:** any mode other than "Off" requires INAV's **`HEADING HOLD`**
box to be active on the follower (assigned in Configurator's Modes tab,
§2.1) — this is a separate mode from `NAV POSHOLD`/`GCS NAV`, and heading
commands are silently ignored by the flight controller if it's not switched
on. If you configure a heading mode and the nose isn't doing what you expect,
this is the first thing to check.

### Trigger & Target

| Field | Meaning |
|---|---|
| **Trigger Mode** | Read-only, shown for reference. How Follow Mode gets switched on — the shipped default is `GCSNAV`, meaning the gate is simply "is `GCS NAV` currently active on the flight controller" (§2.1's switch). The other option has not yet been implemented. |
| **Target Peer** | Which aircraft to follow. "First Active" (the default) locks onto whichever peer is first seen once you engage Follow Mode. If you have more than two aircraft in the air, you can instead pin this to a specific peer's ID so there's no ambiguity about which one gets followed. |
| **Emit Rate** | How often (times per second) the *the follower* sends the target position to the craft — higher gives smoother tracking  of the leader craft but to high to swamp the iNav processor and lead to dropped messages. |
| **Peer Timeout** | How long without hearing from the locked leader before its telemetry is considered stale. When this trips, the follower holds its last commanded position rather than continuing to chase a stale reading (see the lock-state note below). |

**On peer locking:** once Follow Mode engages, the follower locks onto one
specific leader and **does not automatically switch to a different aircraft**
if that leader is lost — even if other peers are visible. If the locked
leader's telemetry goes stale, the follower holds position and waits for that
*same* aircraft's telemetry to come back; it never silently starts following
someone else. To deliberately re-target, either change **Target Peer** (this
forces a fresh lock) or cycle the follow switch off and on.

---

## 4. Apply vs. Save to EEPROM

The panel has two buttons at the bottom, and the difference between them
matters:

- **Apply** — sends your changes to the aircraft and puts them into effect
  **immediately**, live, without needing to reboot or reflash. They are
  **not** persisted: if the flight controller/FF board reboots or loses
  power, your changes are gone and it reverts to whatever was last actually
  saved (or the firmware's compiled-in defaults, if nothing was ever saved).
- **Save to EEPROM** — does the same live-apply as above, and then *also*
  writes the configuration to persistent storage on the FF board, so it
  survives a reboot/power cycle.

**Workflow:** use **Apply** while you're experimenting — try a slot, fly it
(or bench-test it), tweak, repeat — with no commitment. Once you're happy
with a configuration, hit **Save to EEPROM** once to lock it in as your new
default. If you only ever hit Apply, you'll need to re-enter your settings
after every reboot.

---

## 5. Flying it: engaging Follow Mode

1. Confirm both aircraft are powered on, connected to FF's WiFi if you need
   to check status, and visible to each other on the radar HUD.
2. On the follower, arm as normal.
3. Flip the switch you assigned in §2.1 to activate **`NAV POSHOLD`** +
   **`GCS NAV`** + **`HEADING HOLD`** (for rotorcraft) together.
4. That's the entire trigger — as soon as `GCS NAV` goes active, FF's Status
   panel should show the lock state move from `IDLE` → `ACQUIRING` →
   `LOCKED` (typically within a second or two once a peer is visible), and
   the follower should begin flying toward its configured slot relative to
   the leader.
5. To hand control back to yourself, flip the switch off. This immediately
   stops FF from sending target updates and drops the lock — INAV's normal
   stick control (or whatever mode you switch to) takes over exactly as it
   would for leaving any other nav mode.

**First flight recommendation:** don't discover your configured slot's
behavior for the first time in the air. Bench-test it first (there are
dedicated bench/HITL testing guides in `docs/explainers/` for anyone
comfortable with that level of setup), and on the first real flight, use a
generous, low-risk slot (e.g. the "chase-high" default, trailing well behind
and above) at low leader speed, with the follow switch within easy reach the
entire time.

**Screenshot placeholder:** *[FPV goggles — normal formation flight, follower
tracking the leader at the configured slot]*

---

## 6. Showing Follow status on your OSD (GVARs) (Optional)

FF's web/status panel is not something you can see while flying goggles-in.
To get follow-state feedback directly on your OSD, FF can write small status
codes into INAV **Global Variables (GVARs)**, which you then turn into
on-screen text using INAV's own Programming Framework (Logic Conditions +
Custom OSD Elements). FF only ever writes a number — INAV does the rest.

**Requires INAV 9.0.0 or later** on the follower flight controller.
`MSP2_INAV_SET_GVAR` doesn't exist before that; on older/non-INAV firmware,
FF simply never sends anything — this whole feature stays inert, not broken.

### 6.1 Enable it in the Follow panel

In the **OSD Status (GVAR)** section, set:

- **Status GVAR Index** — which GVAR slot (`0`–`7`, or `Disabled`) carries
  the primary lock-state code.
- **Condition Flags GVAR Index** — which GVAR slot carries a secondary
  condition code (currently: whether the altitude floor is actively clamping
  the commanded altitude, or an RC-related condition — §7). Must be a
  *different* slot than Status GVAR Index; the UI blocks picking the same one
  twice.

Leave either on `Disabled` (the default) if you don't want that indicator —
zero MSP traffic is sent for a disabled slot.

| GVAR value (Status) | Meaning |
|---|---|
| `0` | Follow gate inactive — nothing to show |
| `1` | `ACQUIRING` — searching for a leader to lock onto |
| `2` | `LOCKED` — tracking normally |
| `3` | `LOCKED_HOLDING` — leader telemetry stale/lost, holding position |
| `4` | Peer identity mismatch caught — lock invalidated, needs a switch cycle |

| GVAR value (Condition Flags) | Meaning |
|---|---|
| `0` | No condition active |
| `1` | Altitude floor is actively clamping the commanded altitude |
| `2` | RC-driven slot is frozen at its last safe position, or the pre-arm RC check failed (§7) |

### 6.2 One-time INAV CLI setup

Pick your two GVAR indices first (whatever you entered in §6.1 above — the
snippets below use `<STATUS_GVAR_INDEX>` and `<CONDITION_FLAGS_GVAR_INDEX>`
as placeholders; substitute your actual numbers before pasting). Paste the
whole block into Configurator's **CLI** tab, then run `save`.

```
logic 0 1 -1 1 5 <STATUS_GVAR_INDEX> 0 1 0
logic 1 1 -1 1 5 <STATUS_GVAR_INDEX> 0 2 0
logic 2 1 -1 1 5 <STATUS_GVAR_INDEX> 0 3 0
logic 3 1 -1 1 5 <STATUS_GVAR_INDEX> 0 4 0
osd_custom_elements 0 1 0 0 0 0 0 2 0 "SEARCHING"
osd_custom_elements 1 1 0 0 0 0 0 2 1 "LOCKED"
osd_custom_elements 2 1 0 0 0 0 0 2 2 "HOLD LOST"
osd_custom_elements 3 1 0 0 0 0 0 2 3 "ID LOST"
osd_custom_elements 4 1 0 0 0 0 0 1 <CONDITION_FLAGS_GVAR_INDEX> "ALT FLOOR"
save
```

What this does:

- The four `logic` lines each define a Logic Condition that evaluates "does
  the Status GVAR currently equal this code" (`1`–`4`).
- The first four `osd_custom_elements` lines each define a fixed piece of OSD
  text, visible only when its matching Logic Condition is true. Text is
  capped at 16 characters and INAV auto-uppercases it regardless of how you
  type it here — edit the quoted strings to whatever wording you prefer.
- The fifth `osd_custom_elements` line is simpler: rather than a Logic
  Condition, it's gated directly on the Condition Flags GVAR being nonzero
  (visibility type `1`), since that GVAR is already a plain "is this
  condition active" flag.
- Code `0` (gate inactive) deliberately has no matching element — the OSD
  stays clean on flights where you never engage Follow Mode.

This example uses Logic Condition slots `0`–`3` and Custom OSD Element slots
`0`–`4`. If any of those are already used by something else on your aircraft
(another Logic Condition setup, existing custom elements), use free slots
instead and adjust the `osd_custom_elements` visibility values to match.

### 6.3 Place the elements on your OSD layout

`osd_custom_elements` only *defines* the elements — it doesn't position them.
In Configurator's **OSD** tab, the elements you just created appear in the
item list as `CUSTOM ELEMENT 1`–`5`; drag each onto the OSD preview wherever
you want it to appear on screen.

### 6.4 Verify

With the follower connected and Follow Mode engaged, watch the OSD (in
Configurator's OSD preview, or in goggles) as you cycle through follow
states — engage the switch, let it lock, then temporarily move out of radio
range of the leader (or stop the leader's telemetry) to see `HOLD LOST`
appear. Exactly one primary-indicator element should ever be visible at a
time; with the gate off, none should be.

**Screenshot placeholders:**

- *[FPV goggles — OSD showing "SEARCHING"]*
- *[FPV goggles — OSD showing "LOCKED"]*
- *[FPV goggles — OSD showing "HOLD LOST"]*
- *[FPV goggles — OSD showing "ID LOST"]*
- *[FPV goggles — OSD showing "ALT FLOOR" (or the RC-frozen condition) alongside a primary state]*

---

## 7. Trimming the slot live with RC channels

Landing, connecting to FF's web UI, editing a slot, and taking off again is
slow if you just want to nudge the formation gap mid-flight. The **RC Axis
Control** panel lets you assign an ordinary RC channel to each axis so you
can adjust the slot live from your transmitter while Follow Mode is engaged.
This is done by assigning an RC channel to a slider or a rotary knob so that
you can smoothly control the value of the channel.

### 7.1 What assigning a channel actually does

Once a channel is assigned to an axis, that axis's **configured gap stops
being a fixed point and becomes a live-adjustable range**, from the negative
of that gap to the positive of that gap:

- Slider/channel centered (1500µs) → the slot sits at the center of that axis
  (0 offset).
- Slider/channel at full deflection one way (2000µs) → the slot sits at the
  full **positive** configured gap for that axis.
- Slider/channel at full deflection the other way (1000µs) → the slot sits at
  the full **negative** configured gap.

Concretely: if you configure "Behind, 15 m" and assign a channel to the
Longitudinal axis, you're really configuring "this slot moves live between
15 m ahead and 15 m behind" — the "Behind" label you picked in the friendly
grid becomes the *maximum in that direction*, not a fixed position, the
moment a channel is assigned.

An axis left on **Disabled** (the default) is completely unaffected by this
— it stays exactly at its configured fixed value, same as today.

### 7.2 Setting it up

In the **RC Axis Control** panel, set **Longitudinal Channel** / **Lateral
Channel** / **Vertical Channel** to whichever RC channel number (as reported
by your receiver/flight controller) you want driving that axis, or leave
**Disabled**. Each axis needs a different channel — the UI blocks assigning
the same channel to more than one axis.

If you assign a channel to an axis whose configured gap is `0`, that channel
currently has no effect (there's no range to move within) — the panel shows
an inline warning if you do this.

### 7.3 Safety: the slot won't fly straight through the leader

RC input can, in principle, drive all three axes to `0, 0, 0` at once — the
same degenerate "on top of the leader" position the Safety Bounds section
forbids for the static configuration. FF guards against this live:

- Any candidate position that violates Min Separation / Min Vertical
  Separation is rejected outright for that cycle.
- Beyond that, if your stick input would cause the slot to **cross** from one
  side of the leader to the other (e.g. sweeping the vertical channel from
  "fully below" to "fully above" while horizontal is centered) while the
  *other* two axes don't yet provide enough separation on their own, that
  crossing is blocked too — it would otherwise mean the commanded position
  passes directly through the leader's position for an instant.

When either of these trips, the slot **freezes** at the last position it was
safely holding — it stops responding to further stick movement in the unsafe
direction — until you either move that stick back to a safe combination, or
widen one of the other RC-assigned axes past Min Separation first (the same
way a real formation pass has to route *around* another aircraft, not through
it). The Follow panel shows an inline warning (and, if you've set up the
Condition Flags GVAR, `2` on your OSD) whenever this freeze is active.

### 7.4 Pre-arm check

Because the very first "safe position" the freeze logic knows about is your
static configured slot, if your transmitter is already sitting somewhere
that disagrees with that configured slot the instant you engage Follow Mode,
you can get frozen away from your actual current stick position with no
warning beyond the OSD condition code.

To catch this before it matters, FF continuously checks — **while the
aircraft is disarmed only** — whether your current RC stick/channel
positions would produce a safe slot if Follow Mode were engaged right now,
and whether they reproduce your configured static default on every
RC-assigned axis. If either check fails, the Follow panel shows a red
pre-arm warning banner.

**This is advisory only — it never blocks arming.** But heed it: **the check
requires holding each RC-assigned axis at full deflection toward your
configured default's sign** — center-channel, which feels like the "neutral"
position, will *not* clear this check if your configured default for that
axis is nonzero. That's intentional: it forces you to deliberately confirm
channel position before arming, rather than assuming center is safe. Get your
channel into the position the warning wants before arming and engaging follow,
and this scenario never comes up.

---

## 8. Troubleshooting

| Symptom | Likely cause |
|---|---|
| Flipping the switch does nothing, Status panel stays `IDLE` | `NAV POSHOLD` + `GCS NAV` aren't both actually active — check INAV Configurator's Status tab, not just your transmitter switch position. |
| Status shows `ACQUIRING` and never moves to `LOCKED` | No peer visible yet, or the only visible peer has `id = 0`/is marked lost. Check the radar HUD / `Target Peer` peer list is populated. |
| Status shows `LOCKED_HOLDING` | The locked leader's telemetry went stale (radio range, or the leader stopped broadcasting). The follower is intentionally holding position, not searching for a new target — see the peer-locking note in §3. |
| Status shows lock dropping to `ID LOST`-style behavior | The same peer ID got reassigned to a different physical aircraft mid-flight (e.g. the original leader dropped out long enough to lose its radio slot). Cycle the follow switch to force a fresh lock. |
| Follower won't arm/engage at all with a "unsafe slot" warning | Your Follow Slot values violate Min Separation or Min Vertical Separation — widen the gap, or reduce Min Separation if that's genuinely appropriate for your aircraft size. |
| Follower holds well below/above where you expected | Check Min Altitude Floor — if the leader's altitude (or your vertical offset) would put the follower below that floor, altitude gets clamped up to it; this is intentional, not a bug. |
| Nose doesn't point where the Heading setting says it should | `HEADING HOLD` isn't active on the follower — see the note under [Heading](#heading-panel). This is a separate switch from `NAV POSHOLD`/`GCS NAV`. |
| RC channel doesn't move the slot at all | Either the axis is still `Disabled`, or its configured gap is `0` m (nothing to move within) — the panel flags the latter with an inline warning. |
| RC channel stops responding partway through its travel | You've hit the sign-lock freeze — see §7.3. Center the stick or widen another RC-assigned axis first. |
| Settings revert after a power cycle | You used **Apply** but never **Save to EEPROM** — see §4. |

If none of the above explains what you're seeing, the Status panel's
`Locked Peer` and `Last Target` fields are the most useful live diagnostic —
compare the reported target lat/lon/altitude against where you'd expect the
slot to be given the leader's current position and heading.

---

## 9. Slot geometry diagram

<a name="9-slot-geometry-diagram"></a>

The follow slot is defined relative to the **leader's current heading**, not
to compass directions — it rotates with the leader. "Behind" always means
behind wherever the leader is currently pointed, not south, or whatever
direction the leader happened to be facing when you configured the slot.

```
                                  leader's direction of travel
                                              ▲
                                              │
                         AHEAD (+ longitudinal)
                                              │
                    ┌─────────────────────────┼─────────────────────────┐
                    │                         │                         │
                    │        ▲ ABOVE (+ vertical, out of the page)      │
                    │                         │                         │
      LEFT ─────────┼─────────────────────────┼─────────────────────────┼───────── RIGHT
   (− lateral)      │                         │                         │       (+ lateral)
                    │                        LEADER                     │
                    │                       (origin)                    │
                    │        ▼ BELOW (− vertical, into the page)        │
                    │                         │                         │
                    └─────────────────────────┼─────────────────────────┘
                                              │
                         BEHIND (− longitudinal)
                                              │
                                              ▼

   Example: "chase-high" default = Behind 15m, Center, Above 10m
   → follower sits 15m behind and 10m above the leader, tracking
     whichever way the leader is currently facing.
```

|  | Friendly-grid label | Raw offset sign | Axis |
|---|---|---|---|
| Longitudinal | Ahead | positive (+) | along the leader's direction of travel |
| Longitudinal | Behind | negative (−) | " |
| Lateral | Right | positive (+) | perpendicular to travel, 90° clockwise |
| Lateral | Left | negative (−) | " |
| Vertical | Above | positive (+) | straight up |
| Vertical | Below | negative (−) | straight down |

For the full worked math behind this projection (how a track-relative offset
becomes an absolute lat/lon), see
[`docs/explainers/follow-target-geometry.md`](explainers/follow-target-geometry.md)
— that's implementation detail aimed at developers, not required reading to
fly this feature.
