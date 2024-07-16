import time
import pymavlink.mavutil as utility

# Define the center and radius of the circular exclusion geofence
center_lat = -35.363152
center_lng = 149.164795
fence_radius = 100  # in meters

# Connect to the vehicle
vehicle = utility.mavlink_connection("tcp:127.0.0.1:5763")

# Wait for a heartbeat
vehicle.wait_heartbeat()

# Inform the user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# Save the original FENCE_ACTION parameter
vehicle.mav.param_request_read_send(vehicle.target_system, vehicle.target_component, "FENCE_ACTION".encode(), -1)
message = vehicle.recv_match(type="PARAM_VALUE", blocking=True).to_dict()
fence_action_original = message["param_value"]

# Disable the fence action
vehicle.mav.param_set_send(vehicle.target_system, vehicle.target_component, "FENCE_ACTION".encode(), 0, utility.mavlink.MAV_PARAM_TYPE_REAL32)
time.sleep(1)

# Send the command to set the circular exclusion fence
vehicle.mav.command_long_send(
    vehicle.target_system,
    vehicle.target_component,
    utility.mavlink.MAV_CMD_DO_FENCE_ENABLE,
    0,  # Confirmation
    0,  # Param 1: Enable/Disable
    1,  # Param 2: Exclusion Fence (1 = Exclusion, 0 = Inclusion)
    0,  # Param 3: Unused
    0,  # Param 4: Unused
    float(center_lat),  # Param 5: Latitude of the center
    float(center_lng),  # Param 6: Longitude of the center
    fence_radius  # Param 7: Radius of the circle
)

# Wait for the command acknowledgment
while True:
    message = vehicle.recv_match(type="COMMAND_ACK", blocking=True).to_dict()
    if message['command'] == utility.mavlink.MAV_CMD_DO_FENCE_ENABLE:
        if message['result'] == utility.mavlink.MAV_RESULT_ACCEPTED:
            print("Circular exclusion geofence set successfully")
        else:
            print("Failed to set circular exclusion geofence, result:", message['result'])
        break

# Enable the fence action back to its original value
vehicle.mav.param_set_send(vehicle.target_system, vehicle.target_component, "FENCE_ACTION".encode(), fence_action_original, utility.mavlink.MAV_PARAM_TYPE_REAL32)
time.sleep(1)

print("FENCE_ACTION set to original value successfully")
