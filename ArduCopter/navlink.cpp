#include "GCS_Mavlink.h"

#include <navmsg/navmsg.h>

void GCS_MAVLINK_Copter::send_navlink_beacon_info()
{
    static mavlink_tunnel_t msg;

    for(auto i=0; i<2; i++)
    {
        navmsg::beacon_info_t info(AP_HAL::millis(), i, i*1000, 42 + 0.1*i, 24 + 0.2*i, 1000 * i);

        navmsg::msg_to_tunnel(info, msg, 1, 1);

        mavlink_msg_tunnel_send_struct(
            chan,
            &msg
        );
    }   
}

void GCS_MAVLINK_Copter::send_navlink_beacon_proximity()
{
    static uint32_t fake_distance  = 100;

    navmsg::beacon_proximity_t proximity(AP_HAL::millis());

    proximity.set(0, fake_distance);
    proximity.set(1, fake_distance * 2);
   
    fake_distance += 5;    

    static mavlink_tunnel_t msg;

    navmsg::msg_to_tunnel(proximity, msg, 1, 1);

    mavlink_msg_tunnel_send_struct(
        chan,
        &msg
    );
}

void GCS_MAVLINK_Copter::send_navlink_estimated_pos()
{
    navmsg::estimated_pos_t pos(AP_HAL::millis(), 1111, -2222, 3333);
    
    static mavlink_tunnel_t msg;

    navmsg::msg_to_tunnel(pos, msg, 1, 1);

    mavlink_msg_tunnel_send_struct(
        chan,
        &msg
    );
}
