#!/usr/bin/env python3
"""Mock FormationFlight device backend, for previewing/testing html/ in a
browser without real ESP32 hardware.

Serves html/ directly (no copying — edits to those files show up on the next
browser refresh, no rebuild step) and rewrites each response's
`const ENDPOINT_PREFIX = ...;` line on the fly to `""`, so relative fetches
in main.js/follow.js land on this same server instead of the hardcoded
192.168.4.1 device IP. Also serves fake JSON for the endpoints those files
call, including a stateful GET/POST /followmanager/config that round-trips
the same way FollowManager::applyConfig()/configJson() do on real firmware.

Usage:
    python3 .claude/skills/web-ui-preview/mock_server.py [--port 8731]

Then open http://127.0.0.1:<port>/#/follow (or just / for the dashboard).
"""
import argparse
import copy
import json
import re
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

START = time.time()

# Matches main.js/follow.js's `const ENDPOINT_PREFIX = window.location.host
# != "192.168.4.1" ? "http://192.168.4.1" : "";` line regardless of exact
# wording, so this keeps working if that line is ever reworded.
ENDPOINT_PREFIX_RE = re.compile(r"const ENDPOINT_PREFIX = .*?;")


def find_repo_root():
    d = Path(__file__).resolve().parent
    for _ in range(10):
        if (d / "platformio.ini").exists():
            return d
        if d.parent == d:
            break
        d = d.parent
    raise SystemExit("Couldn't find repo root (no platformio.ini above this script)")


REPO_ROOT = find_repo_root()
STATIC_DIR = REPO_ROOT / "html"

DEFAULT_CONFIG = {
    "ofsLongM": -15.0, "ofsLatM": 0.0, "ofsVertM": 10.0,
    "triggerMode": "GCSNAV",
    "targetPeer": 0, "emitHz": 4, "peerTimeoutMs": 1500,
    "minSepM": 8.0, "minVSepM": 13.0, "maxTargetDistM": 50.0, "minAltM": 3.0,
    "minCourseSpeed": 2.0,
    "headingMode": "POINT_LEADER", "headingDeg": 0.0,
    # Pre-populated (rather than -1/disabled) so the OSD Status panel has
    # something to show without needing to be configured first.
    "statusGvarIndex": 0, "conditionFlagsGvarIndex": 1,
    "rcLongChannel": -1, "rcLatChannel": -1, "rcVertChannel": -1,
    "debug": False,
}
CONFIG = copy.deepcopy(DEFAULT_CONFIG)

PEERS = [
    {"rawId": 1, "id": "1", "name": "Falcon", "updated": 0, "age": 120, "lost": 0,
     "lat": 47.641, "lon": -122.140, "latRaw": 476410000, "lonRaw": -1221400000,
     "alt": 85, "groundSpeed": 850, "groundCourse": 900, "distance": 22.4,
     "courseTo": 88, "relativeAltitude": 4, "packetsReceived": 5231, "lq": 98},
    {"rawId": 2, "id": "2", "name": "Eagle", "updated": 0, "age": 340, "lost": 0,
     "lat": 47.642, "lon": -122.141, "latRaw": 476420000, "lonRaw": -1221410000,
     "alt": 90, "groundSpeed": 0, "groundCourse": 0, "distance": 5.1,
     "courseTo": 10, "relativeAltitude": 1, "packetsReceived": 2011, "lq": 91},
]


def system_status():
    return {
        "target": "DIY_ESPNOW", "platform": "ESP32",
        "version": "dev", "gitHash": "mockpreview", "buildTime": "now",
        "cloudBuild": False, "heap": 123456,
        "uptimeMilliseconds": int((time.time() - START) * 1000),
        "phase": 5, "name": "MOCK1", "longName": "mock-preview-craft",
        "host": "INAV", "state": 0,
    }


def peermanager_status():
    return {"myID": "0", "count": len(PEERS), "countActive": len(PEERS),
            "maxPeers": 8, "peers": PEERS}


def gnssmanager_status():
    return {"activeProvider": "GPS", "lat": 47.6415, "lon": -122.1405, "alt": 88,
            "groundSpeed": 420, "groundCourse": 900, "numSat": 14, "fixType": 3}


def cryptomanager_status():
    return {"keyString": "mockmockmockmockmockmockmockmock", "enabled": True}


def radiomanager_status():
    return {"radios": [
        {"status": "OK", "counters": "tx=1234 rx=1230", "enabled": True},
    ]}


def mspmanager_status():
    return {"peerUpdatesSent": 4213, "gnssUpdatesSent": 8899,
            "vbat": 16.4, "mahDrawn": 812, "amps": 9.3}


def followmanager_status():
    status_val = CONFIG["statusGvarIndex"]
    cond_val = CONFIG["conditionFlagsGvarIndex"]
    doc = {
        "state": "LOCKED", "gateActive": True,
        "lockedId": 1, "lockedName": "Falcon",
        "lastTarget": {"lat": 476410500, "lon": -1221405500, "altCm": 1450,
                        "headingDeg": 270,
                        "ageMs": int((time.time() - START) * 1000) % 900},
    }
    if status_val is not None and status_val >= 0:
        doc["statusGvarValue"] = 2  # LOCKED
    if cond_val is not None and cond_val >= 0:
        doc["conditionFlagsGvarValue"] = 0  # no altitude-floor clamp active
    doc["liveOffset"] = {
        "longM": CONFIG["ofsLongM"], "latM": CONFIG["ofsLatM"], "vertM": CONFIG["ofsVertM"],
    }
    doc["rcSlotFrozen"] = False
    doc["rcPreArmCheckFailed"] = False
    # Mirrors FollowManager.cpp's havePreArmCandidateOffset gating loosely
    # (any RC axis assigned) for previewing the panel; real firmware also
    # requires the craft to be disarmed, which this mock doesn't model.
    if any(CONFIG[f] != -1 for f in ("rcLongChannel", "rcLatChannel", "rcVertChannel")):
        doc["preArmCandidateOffset"] = {
            "longM": CONFIG["ofsLongM"], "latM": CONFIG["ofsLatM"], "vertM": CONFIG["ofsVertM"],
        }
    return doc


# Hand-maintained mirror of FollowManager::applyConfig() (src/lib/Follow/
# FollowManager.cpp) for dev-only client testing — not a substitute for the
# firmware's own validation, just enough to exercise the UI's error paths.
def validate_config(cfg):
    if not (cfg.get("emitHz", 0) > 0):
        return "emitHz must be > 0"
    if not (cfg.get("peerTimeoutMs", 0) > 0):
        return "peerTimeoutMs must be > 0"
    if cfg.get("minSepM", 0) < 0 or cfg.get("minVSepM", 0) < 0 or cfg.get("minAltM", 0) < 0:
        return "minSepM/minVSepM/minAltM must be >= 0"
    if not (cfg.get("maxTargetDistM", 0) > 0):
        return "maxTargetDistM must be > 0"
    if cfg.get("minCourseSpeed", 0) < 0:
        return "minCourseSpeed must be >= 0"
    sgi = cfg.get("statusGvarIndex", -1)
    if sgi < -1 or sgi > 7:
        return "statusGvarIndex must be -1 (disabled) or 0-7"
    cgi = cfg.get("conditionFlagsGvarIndex", -1)
    if cgi < -1 or cgi > 7:
        return "conditionFlagsGvarIndex must be -1 (disabled) or 0-7"
    for f in ("rcLongChannel", "rcLatChannel", "rcVertChannel"):
        v = cfg.get(f, -1)
        if v != -1 and (v < 1 or v > 16):
            return f"{f} must be -1 (disabled) or 1-16"
    return None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # keep console quiet; errors still surface via server.log if redirected

    def _json(self, obj, status=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _text(self, text, status=200):
        body = text.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        routes = {
            "/system/status": system_status,
            "/peermanager/status": peermanager_status,
            "/gnssmanager/status": gnssmanager_status,
            "/cryptomanager/status": cryptomanager_status,
            "/radiomanager/status": radiomanager_status,
            "/mspmanager/status": mspmanager_status,
            "/followmanager/status": followmanager_status,
            "/followmanager/config": lambda: CONFIG,
        }
        if path in routes:
            self._json(routes[path]())
            return
        self._serve_static(path)

    def do_POST(self):
        path = urlparse(self.path).path
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length).decode("utf-8") if length else ""
        params = {k: v[0] for k, v in parse_qs(body).items()}

        if path == "/followmanager/config":
            new_cfg = dict(CONFIG)
            float_fields = ["ofsLongM", "ofsLatM", "ofsVertM", "minSepM", "minVSepM",
                             "maxTargetDistM", "minAltM", "minCourseSpeed", "headingDeg"]
            int_fields = ["targetPeer", "emitHz", "peerTimeoutMs",
                          "statusGvarIndex", "conditionFlagsGvarIndex",
                          "rcLongChannel", "rcLatChannel", "rcVertChannel"]
            for f in float_fields:
                if f in params:
                    new_cfg[f] = float(params[f])
            for f in int_fields:
                if f in params:
                    new_cfg[f] = int(params[f])
            if "headingMode" in params:
                new_cfg["headingMode"] = params["headingMode"]
            if "debug" in params:
                new_cfg["debug"] = params["debug"] == "true"

            err = validate_config(new_cfg)
            if err:
                self._text(err, 400)
                return
            CONFIG.clear()
            CONFIG.update(new_cfg)
            self._json(CONFIG)
            return

        if path in ("/followmanager/commit", "/system/reboot"):
            self._text("OK", 200)
            return

        self._text("not found", 404)

    def _serve_static(self, path):
        if path == "/":
            path = "/index.html"
        # Resolve-and-check keeps requests confined to STATIC_DIR even if a
        # path contains ../ — this server only ever runs against trusted
        # local content, but there's no reason to skip the check.
        fs_path = (STATIC_DIR / path.lstrip("/")).resolve()
        if STATIC_DIR.resolve() not in fs_path.parents and fs_path != STATIC_DIR.resolve():
            self._text("not found", 404)
            return
        try:
            data = fs_path.read_bytes()
        except (FileNotFoundError, IsADirectoryError):
            self._text("not found", 404)
            return

        ctype = "application/octet-stream"
        if path.endswith(".html"):
            ctype = "text/html"
        elif path.endswith(".js"):
            ctype = "application/javascript"
            data = ENDPOINT_PREFIX_RE.sub('const ENDPOINT_PREFIX = "";', data.decode("utf-8")).encode("utf-8")
        elif path.endswith(".css"):
            ctype = "text/css"
        elif path.endswith(".svg"):
            ctype = "image/svg+xml"
        elif path.endswith(".png"):
            ctype = "image/png"

        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8731)
    args = parser.parse_args()

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"Serving {STATIC_DIR} — open http://127.0.0.1:{args.port}/#/follow (Ctrl+C to stop)")
    server.serve_forever()
