'use strict';
import { useState, useEffect, html } from './bundle.js';
import { Icons, Setting, Button, Notification, Colored, tipColors } from './components.js';

// Permit using the web ui locally for development (mirrors main.js).
const ENDPOINT_PREFIX = window.location.host != "192.168.4.1" ? "http://192.168.4.1" : "";

// Mirrors FollowManager::offsetFromConfig() / targetSane()'s stacked-slot
// epsilon (src/lib/Follow/FollowManager.cpp) so client-side validation
// agrees with the server-side check for the same config (spec §7.4 — both
// must exist, client-side isn't a substitute for server-side).
const STACKED_HORIZONTAL_EPSILON_M = 0.5;

function resolvedOffset(cfg) {
  if (cfg.offsetMode === 'RAW') {
    return { long: +cfg.ofsLongM, lat: +cfg.ofsLatM, vert: +cfg.ofsVertM };
  }
  const long = cfg.slotLong === 'AHEAD' ? +cfg.gapLongM : cfg.slotLong === 'BEHIND' ? -cfg.gapLongM : 0;
  const lat = cfg.slotLat === 'RIGHT' ? +cfg.gapLatM : cfg.slotLat === 'LEFT' ? -cfg.gapLatM : 0;
  const vert = cfg.slotVert === 'ABOVE' ? +cfg.gapVertM : cfg.slotVert === 'BELOW' ? -cfg.gapVertM : 0;
  return { long, lat, vert };
}

// Client-side mirror of FollowManager::applyConfig()'s validation
// (spec §7.4 + basic field sanity). Blocks Save on failure; the server
// re-validates independently, so this is a UX nicety, not the guard.
function validateConfig(cfg) {
  if (!(cfg.emitHz > 0)) return 'Emit rate must be > 0';
  if (!(cfg.peerTimeoutMs > 0)) return 'Peer timeout must be > 0';
  if (cfg.gapLongM < 0 || cfg.gapLatM < 0 || cfg.gapVertM < 0) return 'Gap values must be >= 0';
  if (cfg.minSepM < 0 || cfg.minVSepM < 0 || cfg.minAltM < 0) return 'Min separation / vertical separation / altitude floor must be >= 0';
  if (!(cfg.maxTargetDistM > 0)) return 'Max target distance must be > 0';
  if (cfg.minCourseSpeed < 0) return 'Min course speed must be >= 0';

  const o = resolvedOffset(cfg);
  const horizontalMag = Math.sqrt(o.long * o.long + o.lat * o.lat);
  const mag3d = Math.sqrt(horizontalMag * horizontalMag + o.vert * o.vert);
  if (mag3d < cfg.minSepM) return 'Slot magnitude is below Min Separation (spec §7.4)';
  if (horizontalMag < STACKED_HORIZONTAL_EPSILON_M && Math.abs(o.vert) < cfg.minVSepM) {
    return 'Stacked slot\'s vertical offset is below Min Vertical Separation (spec §7.4)';
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
const stationaryModeOptions = [['HOLD_COURSE', 'Hold Last Course'], ['WORLD_FRAME', 'World Frame (N/E)']];

export default function FollowPanel() {
  const [config, setConfig] = useState(null);
  const [advanced, setAdvanced] = useState(false);
  const [status, setStatus] = useState(null);
  const [peers, setPeers] = useState(null);
  const [saveResult, setSaveResult] = useState(null);
  const [validationError, setValidationError] = useState(null);

  const applyFetchedConfig = r => { setConfig(r); setAdvanced(r.offsetMode === 'RAW'); };
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

  const mksetfn = k => (v => setConfig(x => Object.assign({}, x, { [k]: v })));

  const onsave = () => {
    const err = validateConfig(config);
    if (err) {
      setValidationError(err);
      return Promise.reject(new Error(err));
    }
    setValidationError(null);

    const body = new URLSearchParams();
    if (advanced) {
      body.append('ofsLongM', config.ofsLongM);
      body.append('ofsLatM', config.ofsLatM);
      body.append('ofsVertM', config.ofsVertM);
    } else {
      body.append('slotLong', config.slotLong);
      body.append('slotLat', config.slotLat);
      body.append('slotVert', config.slotVert);
      body.append('gapLongM', config.gapLongM);
      body.append('gapLatM', config.gapLatM);
      body.append('gapVertM', config.gapVertM);
    }
    body.append('targetPeer', config.targetPeer);
    body.append('emitHz', config.emitHz);
    body.append('peerTimeoutMs', config.peerTimeoutMs);
    body.append('minSepM', config.minSepM);
    body.append('minVSepM', config.minVSepM);
    body.append('maxTargetDistM', config.maxTargetDistM);
    body.append('minAltM', config.minAltM);
    body.append('minCourseSpeed', config.minCourseSpeed);
    body.append('stationaryMode', config.stationaryMode);

    return fetch(ENDPOINT_PREFIX + '/followmanager/config', { method: 'POST', body })
      .then(r => r.ok
        ? r.json().then(r => { applyFetchedConfig(r); setSaveResult({ status: true, message: 'Applied (live, session-only)' }); })
        : r.text().then(t => setSaveResult({ status: false, message: t })));
  };

  if (!config || !status || !peers) return '';

  const targetPeerOptions = [[0, 'First Active']].concat(
    (peers.peers || []).map(p => [p.rawId, `${p.id} (${p.name})`])
  );

  return html`
<div class="m-4 grid grid-cols-1 gap-4 lg:grid-cols-2">
  <div class="lg:col-span-2 bg-yellow-50 border border-yellow-200 text-yellow-800 rounded-md px-4 py-2 text-sm flex items-center gap-2">
    <${Icons.info} class="w-5 h-5 shrink-0" />
    Live edits only — changes apply immediately but are lost on reboot. Permanent (EEPROM) save isn't implemented yet.
  <//>

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
        <${Setting} title="Longitudinal" value=${config.slotLong} setfn=${mksetfn('slotLong')} type="select" options=${slotLongOptions} />
        <${Setting} title="Lateral" value=${config.slotLat} setfn=${mksetfn('slotLat')} type="select" options=${slotLatOptions} />
        <${Setting} title="Vertical" value=${config.slotVert} setfn=${mksetfn('slotVert')} type="select" options=${slotVertOptions} />
        <${Setting} title="Longitudinal Gap" value=${config.gapLongM} setfn=${mksetfn('gapLongM')} type="number" addonRight="m" />
        <${Setting} title="Lateral Gap" value=${config.gapLatM} setfn=${mksetfn('gapLatM')} type="number" addonRight="m" />
        <${Setting} title="Vertical Gap" value=${config.gapVertM} setfn=${mksetfn('gapVertM')} type="number" addonRight="m" />
      ` : html`
        <${Setting} title="Longitudinal Offset" value=${config.ofsLongM} setfn=${mksetfn('ofsLongM')} type="number" addonRight="m" addonLeft="+ahead" />
        <${Setting} title="Lateral Offset" value=${config.ofsLatM} setfn=${mksetfn('ofsLatM')} type="number" addonRight="m" addonLeft="+right" />
        <${Setting} title="Vertical Offset" value=${config.ofsVertM} setfn=${mksetfn('ofsVertM')} type="number" addonRight="m" addonLeft="+above" />
      `}
    <//>
  <//>

  <div class="py-1 divide-y border rounded bg-white flex flex-col">
    <div class="font-light uppercase flex items-center text-gray-600 px-4 py-2">
      Trigger & Target
    <//>
    <div class="py-2 px-5 flex-1 flex flex-col relative">
      <${Setting} title="Trigger Mode" value=${config.triggerMode} setfn=${() => {}} type="text" disabled=${true} />
      <${Setting} title="Target Peer" value=${config.targetPeer} setfn=${mksetfn('targetPeer')} type="select" options=${targetPeerOptions} />
      <${Setting} title="Emit Rate" value=${config.emitHz} setfn=${mksetfn('emitHz')} type="number" addonRight="Hz" />
      <${Setting} title="Peer Timeout" value=${config.peerTimeoutMs} setfn=${mksetfn('peerTimeoutMs')} type="number" addonRight="ms" />
    <//>
  <//>

  <div class="py-1 divide-y border rounded bg-white flex flex-col">
    <div class="font-light uppercase flex items-center text-gray-600 px-4 py-2">
      Safety Bounds
    <//>
    <div class="py-2 px-5 flex-1 flex flex-col relative">
      ${saveResult && html`<${Notification} ok=${saveResult.status} text=${saveResult.message} close=${() => setSaveResult(null)} />`}
      ${validationError && html`<div class="text-sm text-red-600 mb-2">${validationError}<//>`}

      <${Setting} title="Min Separation" value=${config.minSepM} setfn=${mksetfn('minSepM')} type="number" addonRight="m" />
      <${Setting} title="Min Vertical Separation" value=${config.minVSepM} setfn=${mksetfn('minVSepM')} type="number" addonRight="m" />
      <${Setting} title="Max Target Distance" value=${config.maxTargetDistM} setfn=${mksetfn('maxTargetDistM')} type="number" addonRight="m" />
      <${Setting} title="Min Altitude Floor" value=${config.minAltM} setfn=${mksetfn('minAltM')} type="number" addonRight="m" />
      <${Setting} title="Min Course Speed" value=${config.minCourseSpeed} setfn=${mksetfn('minCourseSpeed')} type="number" addonRight="m/s" />
      <${Setting} title="Stationary Fallback" value=${config.stationaryMode} setfn=${mksetfn('stationaryMode')} type="select" options=${stationaryModeOptions} />

      <div class="mb-1 mt-3 flex place-content-end"><${Button} icon=${Icons.save} onclick=${onsave} title="Apply" /><//>
    <//>
  <//>
<//>`;
}
