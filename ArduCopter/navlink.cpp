#include "GCS_Mavlink.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_Math/AP_Math.h>

#include <navmsg/navmsg.h>

#include <random>

#include <iostream>

#define log(m) { std::cout << AP_HAL::millis() << ": " << m << std::endl; }

struct beacon_t {
    uint32_t uuid;
    float lat;
    float lon;
};

const beacon_t kBeacons[] = {
    { 1, 49.76979446411133, 24.23672103881836 },
    { 2, 49.76470947265625, 24.25130844116211 },
    { 3, 49.769405364990234, 24.249910354614258 }
};

static void latlon_to_cartesian_flat(float lat_ref, float lon_ref, float lat, float lon, float& x, float& y)
{
    static const auto kMetersPerLatDeg = 111000;

    x = kMetersPerLatDeg * (lon - lon_ref) * cosf(ToRad(lat_ref));
    y = kMetersPerLatDeg * (lat - lat_ref);
}

static uint32_t add_noice(uint32_t value, uint32_t time)
{
    const auto kJitter = 30.0 * 100; // cm
    const auto kFlux = 10.0 * 100; // cm

    static std::random_device rd;  // Seed for the random number engine
    static std::mt19937 gen(rd()); // Mersenne Twister engine for random numbers
    static std::uniform_real_distribution<> dis(-kJitter, kJitter);
    double jitter = dis(gen);

    const auto distorted = value + sin(float(time) / 13) * kFlux + jitter;

    if(distorted < 0) {
        return 0;
    }

    return distorted;
}

static uint32_t get_distance_to_beacon(float lat, float lon, const beacon_t& beacon)
{
    float x,y;

    // beacon is always a reference point for the projection
    latlon_to_cartesian_flat(beacon.lat, beacon.lon, lat, lon, x, y);

    const auto true_distance = safe_sqrt(x*x + y*y) * 100; // cm

    const auto distorted = add_noice(true_distance, AP_HAL::millis());
   
    const uint32_t true_distance_m = ( uint32_t(distorted) / 100 ) * 100;

    return true_distance_m;
}

void GCS_MAVLINK_Copter::send_navlink_beacon_info()
{
    static mavlink_tunnel_t msg;

    for(size_t i=0; i<sizeof(kBeacons)/sizeof(beacon_t); i++)
    {
        navmsg::beacon_info_t info(AP_HAL::millis(), i, kBeacons[i].uuid, kBeacons[i].lat, kBeacons[i].lon, 0);

        navmsg::msg_to_tunnel(info, msg, 1, 1);

        mavlink_msg_tunnel_send_struct(
            chan,
            &msg
        );
    }   
}

void GCS_MAVLINK_Copter::send_navlink_beacon_proximity()
{
    log("prox begin");
    if(AP::gps().status(0) < AP_GPS::GPS_OK_FIX_3D) {
        return;
    }

    const auto true_loc = AP::gps().location(0);

    navmsg::beacon_proximity_t proximity(AP_HAL::millis());

    for(size_t i=0; i<sizeof(kBeacons)/sizeof(beacon_t); i++)
    {
        const auto d = get_distance_to_beacon(float(true_loc.lat)/10000000UL, float(true_loc.lng)/10000000UL, kBeacons[i]);

        proximity.set(i, d);
    }

    static mavlink_tunnel_t msg;

    navmsg::msg_to_tunnel(proximity, msg, 1, 1);

    mavlink_msg_tunnel_send_struct(
        chan,
        &msg
    );

    log("prox end");
}

void GCS_MAVLINK_Copter::send_navlink_estimated_pos()
{
    // navmsg::estimated_pos_t pos(AP_HAL::millis(), 1111, -2222, 3333);
    
    // static mavlink_tunnel_t msg;

    // navmsg::msg_to_tunnel(pos, msg, 1, 1);

    // mavlink_msg_tunnel_send_struct(
    //     chan,
    //     &msg
    // );
}
