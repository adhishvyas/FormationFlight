#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "MSP.h"
#include "../GNSS/GNSSManager.h"
#include "../Peers/PeerManager.h"

#define HOST_MSP_TIMEOUT 8500

enum MSPHost {
    HOST_NONE = 0,
    HOST_GCS = 1,
    HOST_INAV = 2,
    HOST_ARDU = 3,
    HOST_BTFL = 4
};

class MSPManager {
public:
    MSPManager();
    uint8_t getState();
    void getName(char *name, size_t length);
    MSPHost getFCVariant();
    static bool hostIsFlightController(MSPHost host);
    msp_fc_version_t getFCVersion();
    msp_raw_gps_t getLocation();
    msp_analog_t getAnalogValues();
    // Home/baro-relative altitude estimate from the FC, in centimeters (MSP_ALTITUDE).
    // Distinct from getLocation().alt, which is GPS/MSL altitude in meters.
    int32_t local_altitude_cm();
    // Whether the FC currently has GCS NAV active (MSP_MODE_GCSNAV), e.g. for follow-mode gating.
    bool isGCSNavActive();
    // Reads a single RC channel's value (µs) from the FC over a cached MSP_RC
    // poll (spec docs/spec/2026-08-15-FollowRcAxisControl.md §2.1/§9).
    // channel1Based is 1-16 (MSP_MAX_SUPPORTED_CHANNELS). Returns false (and
    // leaves *outUs untouched) only if not connected to a flight controller or
    // channel1Based is out of range. A transient poll miss on an otherwise-
    // connected FC is NOT one of those failure cases — it returns the last
    // successfully parsed value from the cache instead, same reasoning
    // local_altitude_cm()'s cache already rides out a single dropped frame.
    bool getRcChannelUs(uint8_t channel1Based, uint16_t *outUs);
    void sendRadar(const peer_t *peer);
    // Sends the INAV follow-me special waypoint #255 via MSP_SET_WP.
    // Requires NAV POSHOLD + GCS NAV active on the follower FC.
    // headingDeg: commanded nose heading in degrees, 1-360 (spec §7.7), or 0
    // to leave heading untouched this cycle — INAV's WP#255 handler treats
    // p1 == 0 as "no heading update," not due north, so callers must map a
    // computed heading of exactly 0 to 360 themselves (FollowManager does).
    void sendFollowWaypoint(int32_t lat_1e7, int32_t lon_1e7, int32_t alt_cm, int16_t headingDeg);
    // Writes a single INAV Global Variable over MSP2_INAV_SET_GVAR (spec
    // docs/spec/2026-08-13-FollowStatusOsdGvar.md). One-way, best-effort, no
    // ACK wait. Silently no-ops if the connected FC isn't INAV 9.0+ (§2.2) —
    // callers don't need to check support themselves.
    void sendGvar(uint8_t index, int32_t value);
    void sendLocation(GNSSLocation location);
    void begin(Stream &stream);
    void statusJson(JsonDocument *doc);
    void scheduleNextAt(unsigned long timestamp);
    void loop();

    static MSPManager* getSingleton();
private:
    MSP *msp = nullptr;
    bool ready = false;
    // Counter indicating how many peer updates have been sent over MSP
    uint32_t peerUpdatesSent = 0;
    // Counter indicating how many GPS positions have been injected into the FC via MSP
    uint32_t gnssUpdatesSent = 0;
    // Next timestamp at which we'll transmit a single peer's Radar MSP message
    unsigned long nextSendTime = 0;
    // Which peer we'll send next
    uint8_t peerIndex = 0;

    // Cached MSP_STATUS+MSP_BOXIDS active-mode bitmap (MSP::getActiveModes()
    // is itself two MSP round trips) shared by getState() and
    // isGCSNavActive() so calling both in the same cycle costs one poll, not two.
    uint32_t getActiveModesCached();

    uint8_t mapFixType2Msp(GNSS_FIX_TYPE fixType);
};