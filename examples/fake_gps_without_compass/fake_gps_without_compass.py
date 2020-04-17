import airsim 
client = airsim.MultirotorClient("10.211.55.4")
client.confirmConnection()

from dronekit import connect, VehicleMode
vehicle = connect("127.0.0.1:14551", wait_ready=True)

import time, datetime, math
import numpy as np
from pymavlink import mavutil
from pymavlink import mavextra

from pymavlink.rotmat import Vector3
#  from MAVProxy.modules.lib import LowPassFilter2p
#  vel_filter = LowPassFilter2p.LowPassFilter2p(200.0, 30.0)

def get_gps_time(tnow):
    leapseconds = 18
    SEC_PER_WEEK = 7 * 86400

    epoch = 86400*(10*365 + (1980-1969)/4 + 1 + 6 - 2) - leapseconds
    epoch_seconds = int(tnow - epoch)
    week = int(epoch_seconds) // SEC_PER_WEEK
    t_ms = int(tnow * 1000) % 1000
    week_ms = (epoch_seconds % SEC_PER_WEEK) * 1000 + ((t_ms//200) * 200)
    return week, week_ms

last_msg_time = time.time()
last_gps_pos = None
now = time.time()

while True:
    pose = client.simGetVehiclePose()
    now = time.time()
    now_ms = int(now * 1000)
    time_us = int(now * 1.0e6)

    fix_type = 6

    position = pose.position
    pos_ned = Vector3(position.x_val, position.y_val, -1 * position.z_val)

    gps_week, gps_week_ms = get_gps_time(now)
    gps_lat, gps_lon = mavextra.gps_offset(-35.363261,
                                           149.165230,
                                           pos_ned.y, pos_ned.x)
    gps_alt = pos_ned.z
    sat_visible = 20

    ignore_flags = mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ | mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT 

    gps_vel = Vector3(0, 0, 0)

    msg = vehicle.message_factory.gps_input_encode(time_us, 0, ignore_flags, gps_week_ms, gps_week, fix_type,
                                                 int(gps_lat*1e7),int(gps_lon*1e7),gps_alt,  
                                                 1.0, 1.0,
                                                 gps_vel.x, gps_vel.y, gps_vel.z,
                                                 0.2, 1.0, 1.0,
                                                 sat_visible,
                                                 )                # Satellites visible

    vehicle.send_mavlink(msg)

    print(position.x_val, position.y_val, position.z_val)
    time.sleep(0.2)
