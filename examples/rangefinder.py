import airsim
client = airsim.MultirotorClient("192.168.50.181")
client.confirmConnection()

import time
from dronekit import connect, VehicleMode
vehicle = connect("0.0.0.0:14552", wait_ready=True)

min_depth_cm = 10
max_depth_cm = 4000
def send_single_distance_sensor_msg(distance, orientation):
    # Average out a portion of the centermost part
    msg = vehicle.message_factory.distance_sensor_encode(
        0,                  # ms Timestamp (UNIX time or time since system boot) (ignored)
        min_depth_cm,       # min_distance, uint16_t, cm
        max_depth_cm,       # min_distance, uint16_t, cm
        distance,           # current_distance,	uint16_t, cm	
        0,	                # type : 0 (ignored)
        0,                  # id : 0 (ignored)
        orientation,        # orientation
        0                   # covariance : 0 (ignored)
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

while True:
    rangefinder1 = client.getDistanceSensorData(vehicle_name="Copter")
    distance = rangefinder1.distance
    print(distance)
    if distance > 0:
        send_single_distance_sensor_msg(int(distance*100), 25)
    else:
        send_single_distance_sensor_msg(0, 25)

    time.sleep(0.02)
