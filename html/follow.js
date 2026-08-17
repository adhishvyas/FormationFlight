'use strict';
import { useState, useEffect, html } from './bundle.js';
import { Icons, Setting, Button, Notification, Colored, tipColors } from './components.js';

// Permit using the web ui locally for development (mirrors main.js).
const ENDPOINT_PREFIX = window.location.host != "192.168.4.1" ? "http://192.168.4.1" : "";

// UI-only imperial readouts (spec says all inputs/values are metric on the
// wire — these never leave the browser and are never sent back on save).
const M_TO_FT = 3.28084;
const MPS_TO_MPH = 2.23694;
const asFt = m => `≈ ${(+m * M_TO_FT).toFixed(1)} ft`;
const asMph = mps => `≈ ${(+mps * MPS_TO_MPH).toFixed(1)} mph`;

// Mirrors FollowManager::applyConfig()'s stacked-slot epsilon
// (src/lib/Follow/FollowManager.cpp) so client-side validation agrees with
// the server-side check for the same config (spec §7.4 — both must exist,
// client-side isn't a substitute for server-side).
const STACKED_HORIZONTAL_EPSILON_M = 0.5;

// The AHEAD/BEHIND/LEFT/RIGHT/ABOVE/BELOW "friendly grid" is purely a
// client-side view over the canonical signed offset that's actually stored
// (ofsLongM/ofsLatM/ofsVertM) — the server only ever sees that one
// representation (spec §7.3).
function slotFromOffset(v, posLabel, negLabel, zeroLabel) {
  return v > 0 ? posLabel : v < 0 ? negLabel : zeroLabel;
}

// Recomputes a signed offset from a slot label + unsigned gap, e.g. going
// from ('BEHIND', 15) back to -15.
function offsetFromSlot(slot, gapM, posLabel, negLabel) {
  const g = Math.abs(+gapM);
  return slot === posLabel ? g : slot === negLabel ? -g : 0;
}

// Client-side mirror of FollowManager::applyConfig()'s validation
// (spec §7.4 + basic field sanity). Blocks Save on failure; the server
// re-validates independently, so this is a UX nicety, not the guard.
// Returns { section, message } (section names the panel that owns the
// offending field, so the error can render next to the inputs it's about)
// or null if cfg is valid.
function validateConfig(cfg) {
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
  if (cfg.statusGvarIndex !== -1 && cfg.statusGvarIndex === cfg.conditionFlagsGvarIndex) {
    return { section: 'gvar', message: 'Status and Condition Flags GVAR indices must be different (or both Disabled)' };
  }

  const rcChannels = [cfg.rcLongChannel, cfg.rcLatChannel, cfg.rcVertChannel].filter(c => c !== -1);
  if (new Set(rcChannels).size !== rcChannels.length) {
    return { section: 'rc', message: 'Each RC axis must use a different channel (or Disabled)' };
  }
  return null;
}

const lockStateColors = {
  IDLE: tipColors.gray,
  ACQUIRING: tipColors.yellow,
  LOCKED: tipColors.green,
  LOCKED_HOLDING: tipColors.yellow,
};

const slotLongOptions = [['AHEAD', 'Ahead'], ['CENTER', 'Center'], ['BEHIND', 'Behind']];
const slotLatOptions = [['LEFT', 'Left'], ['CENTER', 'Center'], ['RIGHT', 'Right']];
const slotVertOptions = [['BELOW', 'Below'], ['LEVEL', 'Level'], ['ABOVE', 'Above']];
const headingModeOptions = [
  ['OFF', 'Off (leave heading alone)'],
  ['COURSE', 'Direction of Travel'],
  ['POINT_LEADER', 'Point at Leader'],
  ['FIXED', 'Fixed Compass Heading'],
  ['COURSE_RELATIVE', 'Offset From Course'],
];
const gvarIndexOptions = [[-1, 'Disabled']].concat([0,1,2,3,4,5,6,7].map(i => [i, String(i)]));
const rcChannelOptions = [[-1, 'Disabled']].concat(Array.from({length: 16}, (_, i) => [i + 1, String(i + 1)]));

export default function FollowPanel() {
  const [config, setConfig] = useState(null);
  const [advanced, setAdvanced] = useState(false);
  const [status, setStatus] = useState(null);
  const [peers, setPeers] = useState(null);
  const [saveResult, setSaveResult] = useState(null);
  const [validationError, setValidationError] = useState(null);

  // "advanced" is a local display preference only — both views edit the
  // same canonical ofsLongM/ofsLatM/ofsVertM fields (spec §7.3), so there's
  // no server-side mode to restore it from.
  const applyFetchedConfig = r => setConfig(r);
  const refreshConfig = () => fetch(ENDPOINT_PREFIX + '/followmanager/config').then(r => r.json()).then(applyFetchedConfig);
  const refreshStatus = () => fetch(ENDPOINT_PREFIX + '/followmanager/status').then(r => r.json()).then(setStatus);
  const refreshPeers = () => fetch(ENDPOINT_PREFIX + '/peermanager/status').then(r => r.json()).then(setPeers);

  useEffect(() => {
    refreshConfig();
    refreshStatus();
    refreshPeers();
    // Status/peers poll continuously so the panel is useful as a bench-test
    // aid (spec §12.1); config is only re-fetched after a successful save,
    // since polling it would clobber whatever the user is mid-edit on.
    const t = setInterval(() => { refreshStatus(); refreshPeers(); }, 1000);
    return () => clearInterval(t);
  }, []);

  // Renders validationError inline, in red, but only within the panel it's
  // actually about — avoids implying an error confined to one panel (e.g.
  // Safety Bounds) when it may be about a field elsewhere on the page.
  const sectionError = section => validationError && validationError.section === section &&
    html`<div class="text-sm text-red-900 mb-2">${validationError.message}<//>`;

  const mksetfn = k => (v => setConfig(x => Object.assign({}, x, { [k]: v })));
  // Grid-view setters: recompute the canonical offset from a slot label or
  // a gap magnitude, keeping whichever of the two wasn't just edited.
  const mkslotfn = (offsetKey, posLabel, negLabel) => (slot => setConfig(x => Object.assign({}, x, {
    [offsetKey]: offsetFromSlot(slot, Math.abs(x[offsetKey]), posLabel, negLabel),
  })));
  const mkgapfn = (offsetKey, posLabel, negLabel) => (gapM => setConfig(x => Object.assign({}, x, {
    [offsetKey]: offsetFromSlot(slotFromOffset(x[offsetKey], posLabel, negLabel, posLabel), gapM, posLabel, negLabel),
  })));

  // Posts the current form state to /followmanager/config (validate +
  // apply to RAM only). Returns a promise resolving to whether the apply
  // succeeded, so onsaveEeprom below knows whether it's safe to proceed to
  // an EEPROM commit.
  const applyLive = successMessage => {
    const err = validateConfig(config);
    if (err) {
      setValidationError(err);
      return Promise.resolve(false);
    }
    setValidationError(null);

    const body = new URLSearchParams();
    body.append('ofsLongM', config.ofsLongM);
    body.append('ofsLatM', config.ofsLatM);
    body.append('ofsVertM', config.ofsVertM);
    body.append('targetPeer', config.targetPeer);
    body.append('emitHz', config.emitHz);
    body.append('peerTimeoutMs', config.peerTimeoutMs);
    body.append('minSepM', config.minSepM);
    body.append('minVSepM', config.minVSepM);
    body.append('maxTargetDistM', config.maxTargetDistM);
    body.append('minAltM', config.minAltM);
    body.append('minCourseSpeed', config.minCourseSpeed);

    body.append('headingMode', config.headingMode);
    body.append('headingDeg', config.headingDeg);

    body.append('statusGvarIndex', config.statusGvarIndex);
    body.append('conditionFlagsGvarIndex', config.conditionFlagsGvarIndex);

    body.append('rcLongChannel', config.rcLongChannel);
    body.append('rcLatChannel', config.rcLatChannel);
    body.append('rcVertChannel', config.rcVertChannel);

    body.append('debug', config.debug);

    return fetch(ENDPOINT_PREFIX + '/followmanager/config', { method: 'POST', body })
      .then(r => r.ok
        ? r.json().then(r => { applyFetchedConfig(r); setSaveResult({ status: true, message: successMessage }); return true; })
        : r.text().then(t => { setSaveResult({ status: false, message: t }); return false; }));
  };

  const onsave = () => applyLive('Applied (live, session-only)');

  // Phase 4D: applies the current form state to RAM (same as onsave), then
  // — only if that succeeded — commits it to EEPROM (Phase 4C) so it
  // survives a reboot.
  const onsaveEeprom = () => applyLive('Applied — saving to EEPROM…').then(ok => {
    if (!ok) return false;
    return fetch(ENDPOINT_PREFIX + '/followmanager/commit', { method: 'POST' })
      .then(r => r.ok
        ? r.text().then(() => { setSaveResult({ status: true, message: 'Saved permanently to EEPROM' }); return true; })
        : r.text().then(t => { setSaveResult({ status: false, message: 'Applied, but EEPROM save failed: ' + t }); return false; }));
  });

  if (!config || !status || !peers) return '';

  const targetPeerOptions = [[0, 'First Active']].concat(
    (peers.peers || []).map(p => [p.rawId, `${p.id} (${p.name})`])
  );

  return html`
<div class="m-4 grid grid-cols-1 gap-4 lg:grid-cols-2">
  <div class="py-1 divide-y border rounded bg-white flex flex-col">
    <div class="font-light uppercase flex items-center text-gray-600 px-4 py-2">
      Status
    <//>
    <div class="py-2 px-5 flex-1 flex flex-col relative">
      <div class="grid grid-cols-2 gap-2 my-1">
        <label class="flex items-center text-sm text-gray-700 mr-2 font-medium">Gate<//>
        <div class="flex items-center"><${Colored} colors=${status.gateActive ? tipColors.green : tipColors.gray} text=${status.gateActive ? 'active (GCS NAV)' : 'inactive'} /><//>
      <//>
      <div class="grid grid-cols-2 gap-2 my-1">
        <label class="flex items-center text-sm text-gray-700 mr-2 font-medium">Lock State<//>
        <div class="flex items-center"><${Colored} colors=${lockStateColors[status.state] || tipColors.gray} text=${status.state} /><//>
      <//>
      <div class="grid grid-cols-2 gap-2 my-1">
        <label class="flex items-center text-sm text-gray-700 mr-2 font-medium">Locked Peer<//>
        <span class="text-sm text-gray-700">${status.lockedId ? `${status.lockedId} (${status.lockedName})` : 'none'}<//>
      <//>
      ${status.lastTarget && html`
      <div class="grid grid-cols-2 gap-2 my-1">
        <label class="flex items-center text-sm text-gray-700 mr-2 font-medium">Last Target<//>
        <span class="text-sm text-gray-700">${(status.lastTarget.lat / 1e7).toFixed(6)}, ${(status.lastTarget.lon / 1e7).toFixed(6)} @ ${(status.lastTarget.altCm / 100).toFixed(1)}m (${(status.lastTarget.ageMs / 1000).toFixed(1)}s ago)<//>
      <//>
      `}
    <//>
  <//>

  <div class="py-1 divide-y border rounded bg-white flex flex-col">
    <div class="font-light uppercase flex items-center text-gray-600 px-4 py-2 justify-between">
      <span>Follow Slot<//>
      <button type="button" class="text-xs text-blue-600 hover:underline font-normal normal-case" onclick=${() => setAdvanced(v => !v)}>
        ${advanced ? 'Use friendly grid' : 'Advanced (raw offsets)'}
      <//>
    <//>
    <div class="py-2 px-5 flex-1 flex flex-col relative">
      ${!advanced ? html`
        <div class="flex gap-4">
          <div class="flex-1"><${Setting} title="Longitudinal" tip="Whether your craft flies ahead of, behind, or level with the leader, measured along the leader's direction of travel." value=${slotFromOffset(config.ofsLongM, 'AHEAD', 'BEHIND', 'CENTER')} setfn=${mkslotfn('ofsLongM', 'AHEAD', 'BEHIND')} type="select" options=${slotLongOptions} /><//>
          <div class="flex-1"><${Setting} title="Longitudinal Gap" tip="Distance to keep ahead of or behind the leader, in meters." value=${Math.abs(config.ofsLongM)} setfn=${mkgapfn('ofsLongM', 'AHEAD', 'BEHIND')} type="number" addonRight="m" imperial=${asFt(Math.abs(config.ofsLongM))} /><//>
        <//>
        <div class="flex gap-4">
          <div class="flex-1"><${Setting} title="Lateral" tip="Whether your craft flies to the left, right, or directly in line with the leader, viewed from behind the leader looking forward." value=${slotFromOffset(config.ofsLatM, 'RIGHT', 'LEFT', 'CENTER')} setfn=${mkslotfn('ofsLatM', 'RIGHT', 'LEFT')} type="select" options=${slotLatOptions} /><//>
          <div class="flex-1"><${Setting} title="Lateral Gap" tip="Sideways distance to keep from the leader's flight path, in meters." value=${Math.abs(config.ofsLatM)} setfn=${mkgapfn('ofsLatM', 'RIGHT', 'LEFT')} type="number" addonRight="m" imperial=${asFt(Math.abs(config.ofsLatM))} /><//>
        <//>
        <div class="flex gap-4">
          <div class="flex-1"><${Setting} title="Vertical" tip="Whether your craft flies above, below, or at the same altitude as the leader." value=${slotFromOffset(config.ofsVertM, 'ABOVE', 'BELOW', 'LEVEL')} setfn=${mkslotfn('ofsVertM', 'ABOVE', 'BELOW')} type="select" options=${slotVertOptions} /><//>
          <div class="flex-1"><${Setting} title="Vertical Gap" tip="Altitude difference to keep from the leader, in meters." value=${Math.abs(config.ofsVertM)} setfn=${mkgapfn('ofsVertM', 'ABOVE', 'BELOW')} type="number" addonRight="m" imperial=${asFt(Math.abs(config.ofsVertM))} /><//>
        <//>
      ` : html`
        <${Setting} title="Longitudinal Offset" tip="Signed distance along the leader's direction of travel: positive is ahead of the leader, negative is behind. This is the same value the friendly grid's Longitudinal fields edit." value=${config.ofsLongM} setfn=${mksetfn('ofsLongM')} type="number" addonRight="m" addonLeft="+ahead" imperial=${asFt(config.ofsLongM)} />
        <${Setting} title="Lateral Offset" tip="Signed sideways distance from the leader's flight path: positive is to the right, negative is to the left." value=${config.ofsLatM} setfn=${mksetfn('ofsLatM')} type="number" addonRight="m" addonLeft="+right" imperial=${asFt(config.ofsLatM)} />
        <${Setting} title="Vertical Offset" tip="Signed altitude difference from the leader: positive is above, negative is below." value=${config.ofsVertM} setfn=${mksetfn('ofsVertM')} type="number" addonRight="m" addonLeft="+above" imperial=${asFt(config.ofsVertM)} />
      `}
    <//>
  <//>

  <div class="py-1 divide-y border rounded bg-white flex flex-col">
    <div class="font-light uppercase flex items-center text-gray-600 px-4 py-2">
      Safety Bounds
    <//>
    <div class="py-2 px-5 flex-1 flex flex-col relative">
      ${sectionError('bounds')}

      <${Setting} title="Min Separation" tip="Smallest allowed 3D distance from the leader. A follow slot that works out to less than this is rejected." value=${config.minSepM} setfn=${mksetfn('minSepM')} type="number" addonRight="m" imperial=${asFt(config.minSepM)} />
      <${Setting} title="Min Vertical Separation (when stacked)" tip="When the follow slot sits directly above or below the leader with no horizontal offset, the smallest vertical gap allowed, to keep craft from stacking too close." value=${config.minVSepM} setfn=${mksetfn('minVSepM')} type="number" addonRight="m" imperial=${asFt(config.minVSepM)} />
      <${Setting} title="Max Target Distance" tip="If the leader is ever farther away than this, following is aborted rather than letting this craft chase across an unbounded distance." value=${config.maxTargetDistM} setfn=${mksetfn('maxTargetDistM')} type="number" addonRight="m" imperial=${asFt(config.maxTargetDistM)} />
      <${Setting} title="Min Altitude Floor" tip="Lowest altitude this craft will ever be commanded to while following, regardless of the leader's altitude, so it won't be commanded into the ground." value=${config.minAltM} setfn=${mksetfn('minAltM')} type="number" addonRight="m" imperial=${asFt(config.minAltM)} />
      <${Setting} title="Min Course Speed" tip="Minimum ground speed the leader must be moving at for its direction of travel to be trusted as a heading reference. Below this speed, the last known direction is held instead of following GPS course jitter." value=${config.minCourseSpeed} setfn=${mksetfn('minCourseSpeed')} type="number" addonRight="m/s" imperial=${asMph(config.minCourseSpeed)} />
    <//>
  <//>

  <div class="py-1 divide-y border rounded bg-white flex flex-col">
    <div class="font-light uppercase flex items-center text-gray-600 px-4 py-2">
      Heading
    <//>
    <div class="py-2 px-5 flex-1 flex flex-col relative">
      <${Setting} title="Mode" tip="How this craft's nose direction is controlled while following: leave it alone, point it in the direction of travel, point it at the leader, hold a fixed compass heading, or offset it from the direction of travel." value=${config.headingMode} setfn=${mksetfn('headingMode')} type="select" options=${headingModeOptions} />
      ${(config.headingMode === 'FIXED' || config.headingMode === 'COURSE_RELATIVE') && html`
        <${Setting}
          title=${config.headingMode === 'FIXED' ? 'Heading (absolute)' : 'Heading Offset From Course'}
          tip=${config.headingMode === 'FIXED'
            ? 'Compass heading to hold, in degrees (0° = North, 90° = East).'
            : 'Offset added to the direction-of-travel heading, in degrees. Positive turns the nose to the right of the direction of travel.'}
          value=${config.headingDeg} setfn=${mksetfn('headingDeg')} type="number" addonRight="°" />
      `}
    <//>
  <//>

  <div class="py-1 divide-y border rounded bg-white flex flex-col">
    <div class="font-light uppercase flex items-center text-gray-600 px-4 py-2">
      Trigger & Target
    <//>
    <div class="py-2 px-5 flex-1 flex flex-col relative">
      ${sectionError('trigger')}
      <${Setting} title="Trigger Mode" tip="How following gets switched on. This is fixed by firmware configuration and shown here for reference only." value=${config.triggerMode} setfn=${() => {}} type="text" disabled=${true} />
      <${Setting} title="Target Peer" tip="Which other craft to follow. 'First Active' automatically locks onto the first peer heard broadcasting a valid position." value=${config.targetPeer} setfn=${mksetfn('targetPeer')} type="select" options=${targetPeerOptions} />
      <${Setting} title="Emit Rate" tip="How often this craft broadcasts its own position and speed to peers, in updates per second. Higher rates give smoother following at the cost of more radio airtime." value=${config.emitHz} setfn=${mksetfn('emitHz')} type="number" addonRight="Hz" />
      <${Setting} title="Peer Timeout" tip="How long to wait without hearing from the target peer before treating it as lost and releasing the follow lock." value=${config.peerTimeoutMs} setfn=${mksetfn('peerTimeoutMs')} type="number" addonRight="ms" />
    <//>
  <//>

  <div class="py-1 divide-y border rounded bg-white flex flex-col">
    <div class="font-light uppercase flex items-center text-gray-600 px-4 py-2">
      OSD Status (GVAR)
    <//>
    <div class="py-2 px-5 flex-1 flex flex-col relative">
      ${sectionError('gvar')}
      <${Setting} title="Status GVAR Index" tip="Which INAV Global Variable to write the follow lock-state code to (0=inactive, 1=searching, 2=locked, 3=holding, 4=id lost). Configure a matching Custom OSD element in INAV Configurator to display it." value=${config.statusGvarIndex} setfn=${mksetfn('statusGvarIndex')} type="select" options=${gvarIndexOptions} />
      <${Setting} title="Condition Flags GVAR Index" tip="Which INAV Global Variable to write a secondary condition code to. Currently the only condition is the altitude floor actively clamping the commanded altitude (0=no condition, 1=altitude floor clamped); more conditions may be added to this same slot in the future." value=${config.conditionFlagsGvarIndex} setfn=${mksetfn('conditionFlagsGvarIndex')} type="select" options=${gvarIndexOptions} />
      <div class="text-xs text-gray-500 mt-2">Requires INAV 9.0 or later on the follower FC. Values are written but ignored on older firmware.</div>
      <${Setting} cls="grid grid-cols-2 gap-2 my-1 pt-3 mt-2 border-t" title="Debug (GVARs 4-7)" tip="Writes the locked target peer's raw lat, lon, alt, and heading to GVARs 4, 5, 6, and 7 every cycle, for bench-testing in the goggles. Session-only — always resets to Off on reboot, never saved to EEPROM." value=${config.debug} setfn=${mksetfn('debug')} type="switch" />
    <//>
  <//>

  <div class="py-1 divide-y border rounded bg-white flex flex-col">
    <div class="font-light uppercase flex items-center text-gray-600 px-4 py-2">
      RC Axis Control
    <//>
    <div class="py-2 px-5 flex-1 flex flex-col relative">
      ${sectionError('rc')}
      <div class="text-xs text-gray-500 mb-2">Once an axis has a channel assigned, its configured gap becomes a live-adjustable range (stick centered = centered slot, full deflection = the configured gap in that direction) rather than a fixed point.</div>
      <${Setting} title="Longitudinal Channel" tip="RC channel that live-adjusts the longitudinal (ahead/behind) slot between -Gap and +Gap. Disabled uses the fixed configured value." value=${config.rcLongChannel} setfn=${mksetfn('rcLongChannel')} type="select" options=${rcChannelOptions} />
      ${config.rcLongChannel !== -1 && config.ofsLongM === 0 && html`<div class="text-xs text-yellow-700 mb-2">Longitudinal Gap is 0 — this channel currently has no effect.<//>`}
      <${Setting} title="Lateral Channel" tip="RC channel that live-adjusts the lateral (left/right) slot between -Gap and +Gap." value=${config.rcLatChannel} setfn=${mksetfn('rcLatChannel')} type="select" options=${rcChannelOptions} />
      ${config.rcLatChannel !== -1 && config.ofsLatM === 0 && html`<div class="text-xs text-yellow-700 mb-2">Lateral Gap is 0 — this channel currently has no effect.<//>`}
      <${Setting} title="Vertical Channel" tip="RC channel that live-adjusts the vertical (above/below) slot between -Gap and +Gap." value=${config.rcVertChannel} setfn=${mksetfn('rcVertChannel')} type="select" options=${rcChannelOptions} />
      ${config.rcVertChannel !== -1 && config.ofsVertM === 0 && html`<div class="text-xs text-yellow-700 mb-2">Vertical Gap is 0 — this channel currently has no effect.<//>`}
      <div class="text-xs text-gray-500 mt-2">Crossing a stacked or in-line axis from one side of the leader to the other requires first widening one of the other two RC-assigned axes past Min Separation — the slot won't fly through the leader to get there. This is expected behavior, not a bug.</div>
      ${status.rcSlotFrozen && html`<div class="bg-yellow-50 border border-yellow-200 text-yellow-800 rounded-md px-3 py-2 text-sm mt-2">Slot is currently frozen at its last safe position — current RC input would produce an unsafe slot.<//>`}
      ${status.liveOffset && html`<div class="text-xs text-gray-500 mt-2">Live offset: ${status.liveOffset.longM.toFixed(1)}m long, ${status.liveOffset.latM.toFixed(1)}m lat, ${status.liveOffset.vertM.toFixed(1)}m vert<//>`}
      ${status.preArmCandidateOffset && html`<div class="text-xs text-gray-500 mt-2">Candidate offset (disarmed, bench-test): ${status.preArmCandidateOffset.longM.toFixed(1)}m long, ${status.preArmCandidateOffset.latM.toFixed(1)}m lat, ${status.preArmCandidateOffset.vertM.toFixed(1)}m vert<//>`}
    <//>
  <//>

  ${status.rcPreArmCheckFailed && html`
  <div class="lg:col-span-2 py-1 border rounded bg-white flex flex-col">
    <div class="py-2 px-5 flex-1 flex flex-col relative">
      <div class="bg-red-50 border border-red-200 text-red-800 rounded-md px-4 py-2 text-sm flex items-center gap-2">
        <${Icons.info} class="w-5 h-5 shrink-0" />
        Current RC stick/channel positions would produce an unsafe slot the instant follow engages. Center your sticks (or widen another RC-assigned axis) before arming.
      <//>
    <//>
  <//>
  `}

  <div class="lg:col-span-2 py-1 border rounded bg-white flex flex-col">
    <div class="py-2 px-5 flex-1 flex flex-col relative">
      <div class="bg-yellow-50 border border-yellow-200 text-yellow-800 rounded-md px-4 py-2 text-sm flex items-center gap-2 mb-2">
        <${Icons.info} class="w-5 h-5 shrink-0" />
        Apply applies changes immediately but they're lost on reboot. Use Save to EEPROM to make them permanent.
      <//>
      ${saveResult && html`<${Notification} ok=${saveResult.status} text=${saveResult.message} close=${() => setSaveResult(null)} />`}
      <div class="flex place-content-end gap-2">
        <${Button} icon=${Icons.save} onclick=${onsave} title="Apply" />
        <${Button} icon=${Icons.shield} onclick=${onsaveEeprom} title="Save to EEPROM" />
      <//>
    <//>
  <//>
<//>`;
}
