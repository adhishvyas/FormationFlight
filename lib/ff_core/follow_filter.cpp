#include "follow_filter.h"

#include <cmath>

#include "geo.h"

namespace ff {

namespace {

// Equirectangular (flat-earth) tangent-plane projection -- see follow_filter.h
// and spec SS3.1 for why raw lat/lon degrees aren't filtered directly: a
// longitude degree's length in metres shrinks by cos(latitude), so a velocity
// state derived from unprojected degrees wouldn't have a consistent
// metres/second meaning. Negligible error at typical follow distances (tens
// of metres); not re-anchored for a long flight or large-radius loiter -- see
// spec SS7's open question.
constexpr double kMetersPerDegLat = geo::kEarthRadiusM * geo::kPi / 180.0;

double metersPerDegLon(double originLatDeg) {
    return kMetersPerDegLat * std::cos(geo::toRad(originLatDeg));
}

double deg1e7(int32_t v) { return static_cast<double>(v) / 1e7; }

int32_t toDeg1e7(double v) { return static_cast<int32_t>(std::lround(v * 1e7)); }

}  // namespace

void updateAxis1D(FollowFilterAxis1D* axis, double measurement, uint32_t nowMs, double alpha,
                   double beta) {
    if (!axis->initialized) {
        axis->position = measurement;
        axis->velocity = 0.0;
        axis->lastUpdateMs = nowMs;
        axis->initialized = true;
        return;
    }
    // Cast to a signed delta before converting to seconds: nowMs/lastUpdateMs
    // are both uint32_t millis()-style counters, so an out-of-order call
    // (nowMs < lastUpdateMs) would otherwise wrap to a huge positive value
    // instead of going negative -- same wraparound hazard service()'s own
    // nextRunMs_ throttle already guards against the same way.
    const int32_t dtMs = static_cast<int32_t>(nowMs - axis->lastUpdateMs);
    if (dtMs <= 0) {
        return;  // duplicate/out-of-order timestamp -- nothing to do
    }
    const double dtS = dtMs / 1000.0;

    const double predicted = axis->position + axis->velocity * dtS;
    const double residual = measurement - predicted;
    axis->position = predicted + alpha * residual;
    axis->velocity = axis->velocity + (beta / dtS) * residual;
    axis->lastUpdateMs = nowMs;
}

void updateFilterPosition(FollowPositionFilter* filter, int32_t lat_1e7, int32_t lon_1e7,
                           double altM, uint32_t nowMs, double alpha, double beta) {
    const double lat = deg1e7(lat_1e7);
    const double lon = deg1e7(lon_1e7);
    if (!filter->haveOrigin) {
        filter->originLat = lat;
        filter->originLon = lon;
        filter->haveOrigin = true;
    }
    const double northM = (lat - filter->originLat) * kMetersPerDegLat;
    const double eastM = (lon - filter->originLon) * metersPerDegLon(filter->originLat);

    updateAxis1D(&filter->north, northM, nowMs, alpha, beta);
    updateAxis1D(&filter->east, eastM, nowMs, alpha, beta);
    updateAxis1D(&filter->alt, altM, nowMs, alpha, beta);
}

FollowFilteredLocation filteredLocation(const FollowPositionFilter& filter) {
    const double lat = filter.originLat + filter.north.position / kMetersPerDegLat;
    const double lon =
        filter.originLon + filter.east.position / metersPerDegLon(filter.originLat);

    FollowFilteredLocation loc;
    loc.lat_1e7 = toDeg1e7(lat);
    loc.lon_1e7 = toDeg1e7(lon);
    loc.altM = filter.alt.position;
    return loc;
}

double filteredCourseDeg(const FollowPositionFilter& filter) {
    double deg = geo::toDeg(std::atan2(filter.east.velocity, filter.north.velocity));
    if (deg < 0.0) {
        deg += 360.0;
    }
    return deg;
}

double filteredSpeedMps(const FollowPositionFilter& filter) {
    return std::hypot(filter.north.velocity, filter.east.velocity);
}

std::pair<double, double> resolveFilterGains(uint8_t strengthPct) {
    double alpha = 1.0 - (static_cast<double>(strengthPct) / 100.0);
    if (alpha < kFollowFilterMinAlpha) {
        alpha = kFollowFilterMinAlpha;
    }
    const double beta = (alpha * alpha) / (2.0 - alpha);
    return {alpha, beta};
}

}  // namespace ff
