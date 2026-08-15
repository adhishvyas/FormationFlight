---
name: web-ui-preview
description: Start a local mock backend that serves html/ (FormationFlight's device web UI) in a browser for review/testing, without needing real ESP32 hardware. Use when asked to run, start, preview, or test the web UI, the dashboard, or follow.js/main.js changes locally.
---

# Web UI Preview

`html/` is FormationFlight's device web UI (Preact + htm, no build step —
served as-is by the firmware's `AsyncWebServer`). It expects to run on the
device at `192.168.4.1` and talk to REST endpoints the firmware implements
(`/system/status`, `/followmanager/config`, etc.) — it can't just be opened
as a local file, and `window.location.host` won't equal `192.168.4.1` when
served locally, so `main.js`'s hardcoded `ENDPOINT_PREFIX` would otherwise
send every fetch to a real device that isn't there.

This skill starts a small Python mock backend (`mock_server.py`, alongside
this file) that serves `html/` directly and fakes those endpoints well
enough to review UI changes in a real browser.

## Run it

```bash
python3 .claude/skills/web-ui-preview/mock_server.py [--port 8731]
```

Run it in the background (e.g. Bash tool's `run_in_background`, or `&` in a
terminal) since it blocks serving forever. Then open:

- `http://127.0.0.1:<port>/#/follow` — the Follow panel
- `http://127.0.0.1:<port>/` — the dashboard

Stop it with Ctrl+C, or `pkill -f mock_server.py` / kill the background
task.

## How it works

- Serves `html/` **directly from the repo** — no copying, no build step. Edit
  a file, refresh the browser, see the change immediately.
- On the fly, rewrites each `.js` response's `const ENDPOINT_PREFIX = ...;`
  line to `const ENDPOINT_PREFIX = "";` so relative fetches land on this
  server instead of the hardcoded device IP. The regex matches on the
  variable name, not the exact RHS, so it keeps working if that line's
  wording changes.
- Fakes JSON for every endpoint `main.js`/`follow.js` call: `/system/status`,
  `/peermanager/status`, `/gnssmanager/status`, `/cryptomanager/status`,
  `/radiomanager/status`, `/mspmanager/status`, `/followmanager/status`,
  and a **stateful** GET/POST `/followmanager/config` (POSTed values are
  validated — mirroring `FollowManager::applyConfig()` — and persist
  in-memory until the process restarts, echoed back exactly like the real
  `configJson()` response). `/followmanager/commit` and `/system/reboot`
  just return 200 OK.
- Mock config starts with `statusGvarIndex=0`, `conditionFlagsGvarIndex=1`
  and two fake peers ("Falcon", "Eagle") so panels that need data (Target
  Peer dropdown, OSD Status debug fields, etc.) aren't empty on first load.

## Limitations (this is a UI review aid, not a firmware simulator)

- All mock data is static/fabricated — lock state is always reported
  `LOCKED`, peers never go stale, EEPROM "save" is a no-op that always
  succeeds. It's for checking layout, validation, and wiring, not
  behavior that depends on real telemetry.
- `validate_config()` in `mock_server.py` is a hand-maintained mirror of
  `FollowManager::applyConfig()` in `src/lib/Follow/FollowManager.cpp` —
  if that C++ validation changes, update the mirror too, or server-side
  validation testing here will silently drift from the real firmware.
- If you add a new field to `FollowRuntimeConfig`/`configJson()`, add it to
  `DEFAULT_CONFIG` in `mock_server.py` too, or the new UI control will show
  `undefined` until a save round-trips it.
