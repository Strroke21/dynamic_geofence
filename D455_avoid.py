#!/usr/bin/env python3

import cv2
import pyrealsense2 as rs
import numpy as np
import math
from pymavlink import mavutil
import time

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Define the real-world dimensions at 4 meters
distance = 4  # meters
horizontal_fov = 86  # degrees
vertical_fov = 57  # degrees

total_width = 2 * distance * math.tan(math.radians(horizontal_fov / 2))
total_height = 2 * distance * math.tan(math.radians(vertical_fov / 2))

cell_width = total_width / 3
cell_height = total_height / 3


#functions

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
        0, 0, 0,                    # x, y, z velocity in m/s (using velocity control)
        0, 0, 0,                    # x, y, z acceleration (not used)
        0, 0                        # yaw, yaw_rate (not used)
    )
    vehicle.mav.send(msg)

def get_local_position(vehicle):
    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    100,  # Request every 10 Hz
    1)
    msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    pos_x = msg.x # Degrees
    pos_y = msg.y  # Degrees
    pos_z = msg.z  # Meters
    return [pos_x,pos_y,pos_z]

# Function to check if the UAV has reached the target cell
def has_reached_target(vehicle, target_coordinates):
    current_position = get_local_position(vehicle)
    distance = math.sqrt(
        (current_position[0] - target_coordinates[0]) ** 2 +
        (current_position[1] - target_coordinates[1]) ** 2 +
        (current_position[2] - target_coordinates[2]) ** 2
    )
    return distance

def move_uav_to_target_cell(vehicle, max_distance_index):
    if max_distance_index != 5:  # Central cell is index 4 in 0-based index (cell number 5)
        # Calculate target coordinates in the body frame
        target_coordinates = calculate_target_coordinates(max_distance_index)
        
        # Send the target coordinates to the UAV
        send_position_target_local_ned(vehicle, target_coordinates[0], target_coordinates[1], target_coordinates[2])
        print("target sent...")
        
        # Wait until the UAV has reached the target cell
        while True:
            # Convert target coordinates to the local frame
            current_position = get_local_position(vehicle)
            print(current_position)
            target_north = current_position[0] + target_coordinates[0]
            target_east = current_position[1] + target_coordinates[1]
            target_down = current_position[2] + target_coordinates[2]
            target_local_coordinates = (target_north, target_east, target_down)
            target_distance = has_reached_target(vehicle, target_local_coordinates)
            print(target_distance)
            if target_distance<=0.5:
                break
            
        time.sleep(0.1)



#target waypoint function

def target_waypoint(vehicle,latitude, longitude, altitude):
    msg = vehicle.mav.set_position_target_global_int_encode(
        time_boot_ms=10,
        target_system=vehicle.target_system,       # Target system (usually 1 for drones)
        target_component=vehicle.target_component,    # Target component (usually 1 for drones)
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame of reference for the coordinate system
        type_mask=0b0000111111111000,        # Bitmask to indicate which dimensions should be ignored (0b0000111111111000 means all ignored except position)
        lat_int=int(latitude * 1e7),       # Latitude in degrees * 1e7 (to convert to integer)
        lon_int=int(longitude * 1e7),      # Longitude in degrees * 1e7 (to convert to integer)
        alt=altitude * 1000,           # Altitude in meters (converted to millimeters)
        vx=0,                         # X velocity in m/s (not used)
        vy=0,                         # Y velocity in m/s (not used)
        vz=0,                         # Z velocity in m/s (not used)
        afx=0, afy=0, afz=0,                   # Accel x, y, z (not used)
        yaw=0, yaw_rate=0                       # Yaw and yaw rate (not used)
    )
    vehicle.mav.send(msg)


vehicle =  mavutil.mavlink_connection("tcp:127.0.0.1:5763")
vehicle.wait_heartbeat()
print("Connected to the UAV...")
time.sleep(1)

##### changing to guided mode #####
mode_id = vehicle.mode_mapping()["GUIDED"]  # Get mode ID
vehicle.mav.set_mode_send(
        vehicle.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)

print("Vehicle in GUIDED mode...")


#arm the drone
vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,
    	                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
#print(vehicle.target_system,vehicle.target_component)

print("arming the UAV...")
time.sleep(3)

drone_takeoff_altitude(vehicle,10)
print("taking off to 10m.")
time.sleep(10)

#send_velocity_setpoint(vehicle,50,0,0)
#print("going in forward direction for 50m.")
#time.sleep(5)

counter=0

# Start streaming
pipeline.start(config)

# Threshold distance in meters
THRESHOLD_DISTANCE = 2.0

try:
    while True:
        counter=counter+1
        if counter==1:
            target_waypoint(vehicle,-35.3618874,149.1629861,0)
            print("Going to target waypoint...")
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        # Convert depth frame to numpy array
        frame = np.asanyarray(depth_frame.get_data())

        # Mask out values greater than the maximum distance we are interested in
        mask = frame * depth_frame.get_units() <= THRESHOLD_DISTANCE
        frame = frame * mask

        height, width = frame.shape
        third_height = height // 3
        third_width = width // 3

        # Split the frame into 3x3 grid
        segments = [
            frame[:third_height, :third_width], frame[:third_height, third_width:2*third_width], frame[:third_height, 2*third_width:],
            frame[third_height:2*third_height, :third_width], frame[third_height:2*third_height, third_width:2*third_width], frame[third_height:2*third_height, 2*third_width:],
            frame[2*third_height:, :third_width], frame[2*third_height:, third_width:2*third_width], frame[2*third_height:, 2*third_width:]
        ]

        # Calculate the minimum distance in each segment
        min_distances = []
        for segment in segments:
            valid_distances = segment[segment > 0] * depth_frame.get_units()
            min_distance = np.min(valid_distances) if valid_distances.size > 0 else float('inf')
            min_distances.append(min_distance)

        # Central cell distance
        central_distance = min_distances[4]

        # Check if the central cell has an obstacle within the threshold distance
        if central_distance <= THRESHOLD_DISTANCE:
            # Find the cell with the maximum distance among the other cells
            other_distances = min_distances[:4] + min_distances[5:]
            max_distance = max(other_distances)
            max_distance_index = other_distances.index(max_distance)

            # Adjust index to account for missing central cell (cell 5)
            if max_distance_index >= 4:
                max_distance_index += 1

            cell_to_move = max_distance_index  # Convert to 1-based indexing
            
            move_uav_to_target_cell(vehicle, cell_to_move)   #move to cell without obstacles
            print("Target Cell Reached.")
            #time.sleep(1/10)
            target_waypoint(vehicle,-35.3618874,149.1629861,0)
            print("Continuing with target waypoint...")

        else:
            cell_to_move = 5  # Stay in the central cell

        # Apply color map to the original frame for visualization
        frame_colored = cv2.applyColorMap(cv2.convertScaleAbs(frame, alpha=0.03), cv2.COLORMAP_JET)

        # Draw grid lines
        cv2.line(frame_colored, (third_width, 0), (third_width, height), (255, 255, 255), 2)
        cv2.line(frame_colored, (2*third_width, 0), (2*third_width, height), (255, 255, 255), 2)
        cv2.line(frame_colored, (0, third_height), (width, third_height), (255, 255, 255), 2)
        cv2.line(frame_colored, (0, 2*third_height), (width, 2*third_height), (255, 255, 255), 2)

        # Annotate each segment with the minimum distance in meters
        for i, distance in enumerate(min_distances):
            if distance != float('inf'):
                top_left_x = (i % 3) * third_width
                top_left_y = (i // 3) * third_height
                cv2.putText(frame_colored, f"{distance:.2f}m", (top_left_x + 5, top_left_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Draw bounding box around the segment with the maximum distance (consider valid values only)
        if cell_to_move != 5:
            top_left_x = ((cell_to_move - 1) % 3) * third_width
            top_left_y = ((cell_to_move - 1) // 3) * third_height
            bottom_right_x = top_left_x + third_width
            bottom_right_y = top_left_y + third_height
            cv2.rectangle(frame_colored, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (0, 255, 0), 2)

        # Display the frame with grid and bounding box
        cv2.imshow("Disparity Grid", frame_colored)

        if cv2.waitKey(1) == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
