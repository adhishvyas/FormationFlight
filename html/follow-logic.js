'use strict';

// Pure logic extracted from follow.js (spec
// docs/spec/2026-09-03-FollowTestSuite.md §3.5) -- no DOM/Preact
// dependency, so this module is importable both by follow.js (the UI) and
// by a plain Node test (test/follow-logic.test.js), run with
// `node --test`, no build step, no new dependency.

// Mirrors FollowManager::applyConfig()'s stacked-slot epsilon
// (src/lib/Follow/FollowManager.cpp) so client-side validation agrees with
// the server-side check for the same config (spec §7.4 — both must exist,
// client-side isn't a substitute for server-side).
export const STACKED_HORIZONTAL_EPSILON_M = 0.5;

// The AHEAD/BEHIND/LEFT/RIGHT/ABOVE/BELOW "friendly grid" is purely a
// client-side view over the canonical signed offset that's actually stored
// (ofsLongM/ofsLatM/ofsVertM) — the server only ever sees that one
// representation (spec §7.3).
export function slotFromOffset(v, posLabel, negLabel, zeroLabel) {
  return v > 0 ? posLabel : v < 0 ? negLabel : zeroLabel;
}

// Recomputes a signed offset from a slot label + unsigned gap, e.g. going
// from ('BEHIND', 15) back to -15.
export function offsetFromSlot(slot, gapM, posLabel, negLabel) {
  const g = Math.abs(+gapM);
  return slot === posLabel ? g : slot === negLabel ? -g : 0;
}

// Client-side mirror of FollowManager::applyConfig()'s validation
// (spec §7.4 + basic field sanity). Blocks Save on failure; the server
// re-validates independently, so this is a UX nicety, not the guard.
// Returns { section, message } (section names the panel that owns the
// offending field, so the error can render next to the inputs it's about)
// or null if cfg is valid.
export function validateConfig(cfg) {
  if (!(cfg.emitHz > 0)) return { section: 'trigger', message: 'Emit rate must be > 0' };
  if (!(cfg.peerTimeoutMs > 0)) return { section: 'trigger', message: 'Peer timeout must be > 0' };
  if (cfg.minSepM < 0 || cfg.minVSepM < 0 || cfg.minAltM < 0) {
    return { section: 'bounds', message: 'Min separation / vertical separation / altitude floor must be >= 0' };
  }
  if (!(cfg.maxTargetDistM > 0)) return { section: 'bounds', message: 'Max target distance must be > 0' };
  if (cfg.minCourseSpeed < 0) return { section: 'bounds', message: 'Min course speed must be >= 0' };

  const long = +cfg.ofsLongM, lat = +cfg.ofsLatM, vert = +cfg.ofsVertM;
  const horizontalMag = Math.sqrt(long * long + lat * lat);
  const mag3d = Math.sqrt(horizontalMag * horizontalMag + vert * vert);
  if (mag3d < cfg.minSepM) return { section: 'bounds', message: 'Slot magnitude is below Min Separation (spec §7.4)' };
  if (horizontalMag < STACKED_HORIZONTAL_EPSILON_M && Math.abs(vert) < cfg.minVSepM) {
    return { section: 'bounds', message: 'Stacked slot\'s vertical offset is below Min Vertical Separation (spec §7.4)' };
  }
  const gvarFields = [
    ['statusGvarIndex', 'Status'], ['conditionFlagsGvarIndex', 'Condition Flags'],
    ['targetSpeedGvarIndex', 'Target Speed'], ['autothrottleEngageGvarIndex', 'Autothrottle Engage'],
  ].filter(([k]) => cfg[k] !== -1);
  for (let i = 0; i < gvarFields.length; i++) {
    for (let j = i + 1; j < gvarFields.length; j++) {
      if (cfg[gvarFields[i][0]] === cfg[gvarFields[j][0]]) {
        return { section: 'gvar', message: `${gvarFields[i][1]} and ${gvarFields[j][1]} GVAR indices must be different (or both Disabled)` };
      }
    }
  }

  const rcChannels = [cfg.rcLongChannel, cfg.rcLatChannel, cfg.rcVertChannel].filter(c => c !== -1);
  if (new Set(rcChannels).size !== rcChannels.length) {
    return { section: 'rc', message: 'Each RC axis must use a different channel (or Disabled)' };
  }
  if (cfg.autothrottleEnableRcChannel !== -1 && rcChannels.includes(cfg.autothrottleEnableRcChannel)) {
    return { section: 'autothrottle', message: 'Autothrottle Arm Channel must be different from the RC axis channels (or Disabled)' };
  }
  if (cfg.autothrottleEnableMaxThresholdUs <= cfg.autothrottleEnableMinThresholdUs) {
    return { section: 'autothrottle', message: 'Autothrottle Arm Range Max must be greater than Min' };
  }
  return null;
}
