from pymavlink import mavutil
from droneControl import VehicleMode, drone_takeoff
import time


# Connect to the vehicle
master = mavutil.mavlink_connection('tcp:127.0.0.1:5763')

# Wait for the heartbeat message to find the system ID
master.wait_heartbeat()

# Define geofence vertices (example coordinates)
fence_pos = [-35.3633835, 149.1613984]  # Vertex 1 (latitude, longitude)
fence_radius = 100
 


# Arm the drone (if not already armed)
def arm_drone():
    master.arducopter_arm()
    master.motors_armed_wait()


def enable_fence():
    master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
                0,
                1, #enable:1 diable:0
                2, #type:2 circle
                0,
                0,
                0,
                0,
                0)


def circular_exclusive_geofence():
    msg=master.mav.command_long_encode(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
        0,  # Confirmation
        fence_radius,  # Param 1: Radius of the circle
        0,       # Param 2: Unused
        0,       # Param 3: Unused
        0,       # Param 4: Unused
        float(fence_pos[0]),  # Param 5: Latitude of the center
        float(fence_pos[1]), # Param 6: Longitude of the center
        0        # Param 7: Altitude (set to 0 for simplicity)
    )
    master.mav.send(msg)
    
enable_fence()

while True:
    message = master.recv_match(type="COMMAND_ACK", blocking=True).to_dict()
    print(message)
    if message['command'] == mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE:
        circular_exclusive_geofence()
        if message['result'] == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Circular exclusion geofence set successfully")
        else:
            print("Failed to set circular exclusion geofence, result:", message['result'])
        break





