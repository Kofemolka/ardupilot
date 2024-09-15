#include "GCS_Mavlink.h"

#include <vector>

#include "cbor/encoder.h"

struct beacon_info_t
{
    static const uint16_t kMsgId { 60000 };
    static const uint8_t kVersion { 1 };

    struct beacon_location_t
    {
        int32_t lat;
        int32_t lon;
        int32_t alt;

        void encode(cbor_writer_t* writer) const
        {
            cbor_encode_negative_integer(writer, lat);
            cbor_encode_negative_integer(writer, lon);
            cbor_encode_negative_integer(writer, alt);
        }
    };

    uint32_t ts_us;
    std::vector<beacon_location_t> beacons;

    size_t encode(uint8_t* buff, size_t bufsize) const {
        cbor_writer_t writer;
        cbor_writer_init(&writer, buff, bufsize);

        cbor_encode_simple(&writer, kVersion);
        cbor_encode_unsigned_integer(&writer, ts_us);

        cbor_encode_array_indefinite(&writer);

        for(const auto& beacon : beacons)
        {
            beacon.encode(&writer);
        }

        cbor_encode_break(&writer);
        
        return cbor_writer_len(&writer);
    }
};

struct beacon_proximity_t
{
    static const uint16_t kMsgId { 60001 };
    static const uint8_t kVersion { 1 };

    uint32_t ts_us;
    std::vector<uint32_t> distances;

    size_t encode(uint8_t* buff, size_t bufsize) const {
        cbor_writer_t writer;
        cbor_writer_init(&writer, buff, bufsize);

        cbor_encode_simple(&writer, kVersion);
        cbor_encode_unsigned_integer(&writer, ts_us);

        cbor_encode_array_indefinite(&writer);

        for(const auto d : distances)
        {
            cbor_encode_unsigned_integer(&writer, d);
        }

        cbor_encode_break(&writer);
        
        return cbor_writer_len(&writer);
    }
};

struct estimated_pos_t
{
    static const uint16_t kMsgId { 60002 };
    static const uint8_t kVersion { 1 };

    uint32_t ts_us;
    int32_t lat;
    int32_t lon;
    int32_t alt;

    size_t encode(uint8_t* buff, size_t bufsize) const {
        cbor_writer_t writer;
        cbor_writer_init(&writer, buff, bufsize);

        cbor_encode_simple(&writer, kVersion);
        cbor_encode_unsigned_integer(&writer, ts_us);

        cbor_encode_negative_integer(&writer, lat);
        cbor_encode_negative_integer(&writer, lon);
        cbor_encode_negative_integer(&writer, alt);
        
        return cbor_writer_len(&writer);
    }
};

template<typename S>
void to_tunnel(const S& s, mavlink_tunnel_t& msg)
{
    static const uint8_t kSystemID = 100;
    static const uint8_t kComponentID = 100;

    msg.target_system = kSystemID;
    msg.target_component = kComponentID;

    msg.payload_length = s.encode(msg.payload, sizeof(msg.payload));
    msg.payload_type = s.kMsgId;
}

void GCS_MAVLINK_Copter::send_navlink_beacon_info()
{
    beacon_info_t info;
    info.ts_us = AP_HAL::millis();

    info.beacons.push_back(beacon_info_t::beacon_location_t
        {
            .lat = 1000,
            .lon = 2000,
            .alt = 3000
        });
    info.beacons.push_back(beacon_info_t::beacon_location_t
        {
            .lat = 4000,
            .lon = 5000,
            .alt = 6000
        });

    static mavlink_tunnel_t msg;

    to_tunnel(info, msg);
 
    mavlink_msg_tunnel_send_struct(
        chan,
        &msg
    );
}

void GCS_MAVLINK_Copter::send_navlink_beacon_proximity()
{
    beacon_proximity_t proximity;
    proximity.ts_us = AP_HAL::millis();

    static uint32_t fake_distance  = 100;
    proximity.distances.push_back(fake_distance);
    proximity.distances.push_back(fake_distance * 2);

    fake_distance += 100;    

    static mavlink_tunnel_t msg;

    to_tunnel(proximity, msg);
    
    mavlink_msg_tunnel_send_struct(
        chan,
        &msg
    );
}

void GCS_MAVLINK_Copter::send_navlink_estimated_pos()
{
    estimated_pos_t pos;
    pos.ts_us = AP_HAL::millis();

    pos.lat = 1111;
    pos.lon = -2222;
    pos.alt = 3333; 

    static mavlink_tunnel_t msg;

    to_tunnel(pos, msg);
    
    mavlink_msg_tunnel_send_struct(
        chan,
        &msg
    );
}
