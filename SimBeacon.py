#!/usr/bin/env python3

import time
import math

from pymavlink import mavutil
from pymavlink.dialects.v20.common \
    import MAVLink_gps_input_message, MAVLink_attitude_message, MAVLink_global_position_int_message, MAVLink_gps_raw_int_message

latitude=37.902067
longitude=-121.498691
altitude=1
yaw_deg=30

latitude2 =37.902067
longitude2 =-121.498691
altitude2 =1
yaw_deg2=30

# dist_time

def create_mavlink_gps_input_msg(): #232
    return MAVLink_gps_input_message(
        time_usec=9999,
        gps_id=10,
        ignore_flags=0,
        time_week_ms=8,
        time_week=8,
        fix_type=4,
        lat=int(latitude * 1e7),
        lon=int(longitude * 1e7),
        alt=int(altitude),
        hdop=3,
        vdop=3,
        vn=0,
        ve=0,
        vd=0,
        speed_accuracy=2,
        horiz_accuracy=2,
        vert_accuracy=2,
        satellites_visible=15,
        yaw=int(yaw_deg))

def create_mavlink_attitude(): #30
    return MAVLink_attitude_message(
        time_boot_ms=9999,
        roll=0,
        pitch=0,
        yaw=math.radians(yaw_deg),
        rollspeed=0,
        pitchspeed=0,
        yawspeed=0)

def create_mavlink_global_position_int_message(): #33
    # lat: Latitude in 1e7 scale.
    # lon: Longitude in 1e7 scale.
    # alt: Altitude in MSL, in mm unit.
    # vx, vy, vz: In NED, cm/s unit.
    # heading: Heading in degree, range 0-360, scale 100.
    return MAVLink_global_position_int_message(
        time_boot_ms=9999,
        lat=int(latitude * 1e7),
        lon=int(longitude * 1e7),
        alt=int(altitude * 1e3),
        relative_alt=0,
        vx=0,
        vy=0,
        vz=0,
        hdg=0)

def create_mavlink_gps_raw_int_message(): #24

    return MAVLink_gps_raw_int_message(
        time_usec=9999,
        fix_type=4,
        lat=int(latitude * 1e7),
        lon=int(longitude * 1e7),
        alt=int(altitude * 1e3),
        eph=3,
        epv=3,
        vel=0,
        cog=0,
        satellites_visible=15)


if __name__=="__main__":
    connection = mavutil.mavlink_connection(
        'tcp:localhost:5760',
        source_system=10,
        autoreconnect=True,
        input=False)



    while True:

        connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

        #Loop through all the beacon positions
        # for x in

        gps_input_mav_msg = create_mavlink_gps_input_msg()
        attitude_mav_msg = create_mavlink_attitude()
        global_position_int_msg = create_mavlink_global_position_int_message()
        gps_raw_int_msg = create_mavlink_gps_raw_int_message()
        connection.mav.send(gps_input_mav_msg)
        connection.mav.send(attitude_mav_msg)
        connection.mav.send(global_position_int_msg)
        connection.mav.send(gps_raw_int_msg)

        time.sleep(0.2)

