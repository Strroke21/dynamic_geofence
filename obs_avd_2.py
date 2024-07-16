import cv2
import pyrealsense2 as rs
import numpy as np
import math
from pymavlink import mavutil
import time


# Define the real-world dimensions at 4 meters
distance = 4  # meters
horizontal_fov = 86  # degrees
vertical_fov = 57  # degrees

total_width = 2 * distance * math.tan(math.radians(horizontal_fov / 2))
total_height = 2 * distance * math.tan(math.radians(vertical_fov / 2))

cell_width = total_width / 3
cell_height = total_height / 3


#guided take off function


def drone_takeoff_altitude(vehicle, altitude):
    
    # Send MAVLink command to takeoff
    vehicle.mav.command_long_send(
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
        0,                          # confirmation
        0,                          # param1 (min pitch, not used)
        0,                          # param2 (empty for now, not used)
        0,                          # param3 (empty for now, not used)
        0,                          # param4 (yaw angle in degrees, not used)
        0,                          # param5 (latitude, not used)
        0,                          # param6 (longitude, not used)
        altitude                    # param7 (altitude in meters)
    )
    


#velocity function

def send_velocity_setpoint(vehicle, pos_x, pos_y, pos_z):

    # Send MAVLink command to set velocity
    vehicle.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (not used)
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,        # type_mask (only vx, vy, vz, yaw_rate)
        pos_x, pos_y, pos_z,                    # position (not used)
        0, 0, 0,                 # velocity in m/s
        0, 0, 0,                    # acceleration (not used)
        0, 0                        # yaw, yaw_rate (not used)
    )
 

# Calculate target coordinates for each cell in NED frame
def calculate_target_coordinates(cell_number):
    cell_positions = {
        1: (4, -cell_width, cell_height),
        2: (4, 0, cell_height),
        3: (4, cell_width, cell_height),
        4: (4, -cell_width, 0),
        5: (4, 0, 0),
        6: (4, cell_width, 0),
        7: (4, -cell_width, -cell_height),
        8: (4, 0, -cell_height),
        9: (4, cell_width, -cell_height)
    }
    return cell_positions[cell_number]

# Function to send position target to UAV

def send_position_target_local_ned(vehicle, relative_x, relative_y, relative_z):
    msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        0,                          # time_boot_ms (not used)
        1,                          # target system ID (arbitrary)
        1,                          # target component ID (arbitrary)
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111111000,        # type_mask (only positions enabled)
        relative_x,                 # x position in meters
        relative_y,                 # y position in meters
        relative_z,                 # z position in meters
        0, 0, 0,                    # x, y, z velocity in m/s (not used)
        0, 0, 0,                    # x, y, z acceleration (not used)
        0, 0                        # yaw, yaw_rate (not used)
    )
    vehicle.mav.send(msg)

def move_uav_to_target_cell(vehicle, max_distance_index):
    if max_distance_index != 4:  # Central cell is index 4 in 0-based index (cell number 5)
        target_coordinates = calculate_target_coordinates(max_distance_index + 1)
        send_position_target_local_ned(vehicle, target_coordinates[0], target_coordinates[1], target_coordinates[2])



# Depth Processing Code
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

MAX_DISTANCE = 8.0
THRESHOLD_DISTANCE = 5.0

vehicle =  mavutil.mavlink_connection("tcp:127.0.0.1:5763")
vehicle.wait_heartbeat()
print("Connected to the UAV...")
time.sleep(1)

##### changing to guided mode #####
mode_id = vehicle.mode_mapping()["GUIDED"]  # Get mode ID
vehicle.mav.set_mode_send(
        vehicle.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)

print("Vehicle in GUIDED")



#arm the drone
vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
    	                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
#print(vehicle.target_system,vehicle.target_component)

print("arming the UAV...")
time.sleep(3)

drone_takeoff_altitude(vehicle,10)
print("taking off to 10m.")
time.sleep(10)

send_velocity_setpoint(vehicle,100,0,0)
print("going in forward direction for 100m.")
time.sleep(15)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        frame = np.asanyarray(depth_frame.get_data())
        mask = frame * depth_frame.get_units() <= MAX_DISTANCE
        frame = frame * mask

        height, width = frame.shape
        third_height = height // 3
        third_width = width // 3

        segments = [
            frame[:third_height, :third_width], frame[:third_height, third_width:2*third_width], frame[:third_height, 2*third_width:],
            frame[third_height:2*third_height, :third_width], frame[third_height:2*third_height, third_width:2*third_width], frame[third_height:2*third_height, 2*third_width:],
            frame[2*third_height:, :third_width], frame[2*third_height:, third_width:2*third_width], frame[2*third_height:, 2*third_width:]
        ]

        min_distances = []
        for segment in segments:
            min_distance = np.min(segment) * depth_frame.get_units()
            min_distances.append(min_distance)

        min_distance_central_cell = min_distances[4]

        if min_distance_central_cell <= THRESHOLD_DISTANCE:
            max_distance_index = min_distances.index(max(min_distances))
            move_uav_to_target_cell(vehicle, max_distance_index)
            print("sending target location...")

        frame_colored = cv2.applyColorMap(cv2.convertScaleAbs(frame, alpha=0.03), cv2.COLORMAP_JET)

        cv2.line(frame_colored, (third_width, 0), (third_width, height), (255, 255, 255), 2)
        cv2.line(frame_colored, (2*third_width, 0), (2*third_width, height), (255, 255, 255), 2)
        cv2.line(frame_colored, (0, third_height), (width, third_height), (255, 255, 255), 2)
        cv2.line(frame_colored, (0, 2*third_height), (width, 2*third_height), (255, 255, 255), 2)

        for i, distance in enumerate(min_distances):
            if distance <= MAX_DISTANCE:
                top_left_x = (i % 3) * third_width
                top_left_y = (i // 3) * third_height
                cv2.putText(frame_colored, f"{distance:.2f}m", (top_left_x + 5, top_left_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            else:
                min_distances[i] = -1

        if min_distance_central_cell <= THRESHOLD_DISTANCE and max_distance_index != 4:
            top_left_x = (max_distance_index % 3) * third_width
            top_left_y = (max_distance_index // 3) * third_height
            bottom_right_x = top_left_x + third_width
            bottom_right_y = top_left_y + third_height
            cv2.rectangle(frame_colored, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (0, 255, 0), 2)

        cv2.imshow("Disparity Grid", frame_colored)

        if cv2.waitKey(1) == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()



