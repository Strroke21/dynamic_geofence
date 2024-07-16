

from droneControl import connect, get_local_position, get_global_position, VehicleMode, condition_yaw

vehicle = connect("tcp:127.0.0.1:5763")
counter = 0

while True:
    counter += 1
    if counter==1:
        VehicleMode(9,vehicle)
    condition_yaw(vehicle,100,1)
    local_pos = get_local_position(vehicle)
    global_pos = get_global_position(vehicle)
    print(global_pos[0],global_pos[1],global_pos[2])
    #print(local_pos[0],local_pos[1],local_pos[2])
