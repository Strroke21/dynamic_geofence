import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import math

# introduce FENCE_TOTAL and FENCE_ACTION as byte array and do not use parameter index
FENCE_TOTAL = "FENCE_TOTAL".encode(encoding="utf-8")
FENCE_ACTION = "FENCE_ACTION".encode(encoding="utf-8")
PARAM_INDEX = -1

# define the center and radius of the circular geofence
center_lat = -35.3633835
center_lng = 149.1613984
radius = 50 # in meters
num_points = 36  # number of points to form the circle

# function to create circular geofence points
def create_circle_points(center_lat, center_lng, radius, num_points):
    points = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        lat_offset = (radius / 6378137.0) * (180 / math.pi)
        lng_offset = lat_offset / math.cos(center_lat * math.pi / 180)
        point_lat = center_lat + (lat_offset * math.sin(angle))
        point_lng = center_lng + (lng_offset * math.cos(angle))
        points.append((point_lat, point_lng))
    points.append(points[0])  # close the loop by repeating the first point
    return points

# generate the geofence points
fence_list = create_circle_points(center_lat, center_lng, radius, num_points)

# connect to vehicle
vehicle = utility.mavlink_connection(device="tcp:127.0.0.1:5763")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# create PARAM_REQUEST_READ message
message = dialect.MAVLink_param_request_read_message(target_system=vehicle.target_system,
                                                     target_component=vehicle.target_component,
                                                     param_id=FENCE_ACTION,
                                                     param_index=PARAM_INDEX)

# send PARAM_REQUEST_READ message to the vehicle
vehicle.mav.send(message)

# wait until get FENCE_ACTION
while True:
    # wait for PARAM_VALUE message
    message = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname, blocking=True)
    message = message.to_dict()

    if message["param_id"] == "FENCE_ACTION":
        fence_action_original = int(message["param_value"])
        break

# debug parameter value
print("FENCE_ACTION parameter original:", fence_action_original)

# run until parameter set successfully
while True:
    message = dialect.MAVLink_param_set_message(target_system=vehicle.target_system,
                                                target_component=vehicle.target_component,
                                                param_id=FENCE_ACTION,
                                                param_value=0,
                                                param_type=dialect.MAV_PARAM_TYPE_REAL32)
    vehicle.mav.send(message)

    message = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname, blocking=True)
    message = message.to_dict()

    if message["param_id"] == "FENCE_ACTION" and int(message["param_value"]) == 0:
        print("FENCE_ACTION reset to 0 successfully")
        break

# run until parameter reset successfully
while True:
    message = dialect.MAVLink_param_set_message(target_system=vehicle.target_system,
                                                target_component=vehicle.target_component,
                                                param_id=FENCE_TOTAL,
                                                param_value=0,
                                                param_type=dialect.MAV_PARAM_TYPE_REAL32)
    vehicle.mav.send(message)

    message = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname, blocking=True)
    message = message.to_dict()

    if message["param_id"] == "FENCE_TOTAL" and int(message["param_value"]) == 0:
        print("FENCE_TOTAL reset to 0 successfully")
        break

# run until parameter set successfully
while True:
    message = dialect.MAVLink_param_set_message(target_system=vehicle.target_system,
                                                target_component=vehicle.target_component,
                                                param_id=FENCE_TOTAL,
                                                param_value=len(fence_list),
                                                param_type=dialect.MAV_PARAM_TYPE_REAL32)
    vehicle.mav.send(message)

    message = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname, blocking=True)
    message = message.to_dict()

    if message["param_id"] == "FENCE_TOTAL" and int(message["param_value"]) == len(fence_list):
        print("FENCE_TOTAL set to {0} successfully".format(len(fence_list)))
        break

# initialize fence item index counter
idx = 0

# run until all the fence items uploaded successfully
while idx < len(fence_list):
    message = dialect.MAVLink_fence_point_message(target_system=vehicle.target_system,
                                                  target_component=vehicle.target_component,
                                                  idx=idx,
                                                  count=len(fence_list),
                                                  lat=fence_list[idx][0],
                                                  lng=fence_list[idx][1])
    vehicle.mav.send(message)

    message = dialect.MAVLink_fence_fetch_point_message(target_system=vehicle.target_system,
                                                        target_component=vehicle.target_component,
                                                        idx=idx)
    vehicle.mav.send(message)

    message = vehicle.recv_match(type=dialect.MAVLink_fence_point_message.msgname, blocking=True)
    message = message.to_dict()

    latitude = message["lat"]
    longitude = message["lng"]

    if latitude != 0.0 and longitude != 0:
        idx += 1

print("All the fence items uploaded successfully")

# run until parameter set successfully
while True:
    message = dialect.MAVLink_param_set_message(target_system=vehicle.target_system,
                                                target_component=vehicle.target_component,
                                                param_id=FENCE_ACTION,
                                                param_value=fence_action_original,
                                                param_type=dialect.MAV_PARAM_TYPE_REAL32)
    vehicle.mav.send(message)

    message = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname, blocking=True)
    message = message.to_dict()

    if message["param_id"] == "FENCE_ACTION" and int(message["param_value"]) == fence_action_original:
        print("FENCE_ACTION set to original value {0} successfully".format(fence_action_original))
        break
