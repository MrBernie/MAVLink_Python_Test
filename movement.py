from pymavlink import mavutil
import time

# Start a connection using the simulator's serial port
# the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
the_connection = mavutil.mavlink_connection('udp:localhost:14550')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link

the_connection.wait_heartbeat()
print(the_connection)
print("Heartbeat from system (system %u component %u)" 
      % (the_connection.target_system, the_connection.target_component))

msg = the_connection.recv_match(type = 'HEARTBEAT',blocking=True)
print(msg)


# Set Mode to GUIDED
# MAV_CMD_DO_SET_MODE = 176
# Guided = 4
# The flight mode is corresponding to the flight mode table in the ArduPilot Parameter list not in the MavLink.
the_connection.mav.command_long_send(the_connection.target_system, 
                                     the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                                     0, # The confirmation
                                     1, 4, 0, 0, 0, 0, 0)

# Send the message
the_connection.mav.send(msg)

msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True)
print(msg)
if msg.result != 0:
    exit(1)


# Arm the drone
# COMMAND_LONG = 76
# MAV_CMD_COMPONENT_ARM_DISARM = 400
the_connection.mav.command_long_send(the_connection.target_system, 
                                     the_connection.target_component,
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                    0, 1, 0, 0, 0, 0, 0, 0)
# COMMAND_ACK = 77
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True)
print(msg)
if msg.result != 0:
    exit(1)


# Take off to 10m height
# COMMAND_LONG = 76
# MAV_CMD_NAV_TAKEOFF = 22
the_connection.mav.command_long_send(the_connection.target_system, 
                                     the_connection.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
                                    0, 0, 0, 0, 0, 0, 0, 10)
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True, timeout=20)
print(msg)
if msg.result != 0:
    exit(1)


# Take off to 10m height
# COMMAND_LONG = 76
# MAV_CMD_NAV_TAKEOFF = 22
# the_connection.mav.command_int_send(the_connection.target_system, 
#                                      the_connection.target_component,
#                                     #Still need to figure out why MAV_FRAME_LOCAL_NED is not supported here.
#                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
#                                     0, 0, 0, 0, 0, 0, 0, 0, 10)
# msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True, timeout=20)
# print(msg)

while True:
    # MAV_CMD_REQUEST_MESSAGE = 512
    # Sending a request for LOCAL_POSITION_NED = 32
    the_connection.mav.command_long_send(the_connection.target_system,
                                         the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                         0, 
                                         mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 
                                         0, 0, 0, 0, 0, 0)
    msg = the_connection.recv_match(type='LOCAL_POSITION_NED',blocking=True)
    print(msg)
    if msg.z < -9:
        break
    time.sleep(0.5)
print("Reached target altitude")


# Set the destination to be 50, -50, -15 in meters relative to the home position
# 50, -50, -15 is corresponding to North, East, Down
# SET_POSITION_TARGET_LOCAL_NED = 84
# MAV_FRAME_LOCAL_NED = 1
# cmd_msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, 
#                                                                   the_connection.target_system,
#                                                                   the_connection.target_component,
#                                                                   mavutil.mavlink.MAV_FRAME_LOCAL_NED,
#                                                                     int(0b110111111000), # type_mask
#                                                                     50, -50, -15, # x, y, z positions
#                                                                     # for NED, x is North, y is East, z is Down
#                                                                     0, 0, 0, # x, y, z velocity in m/s
#                                                                     0, 0 ,0 , # x, y, z acceleration
#                                                                     0 ,0 # yaw, yaw_rate
#                                                                   )
# the_connection.mav.send(cmd_msg)
the_connection.mav.set_position_target_local_ned_send(10, 
                                                the_connection.target_system,
                                                the_connection.target_component,
                                                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                int(0b110111111000), # type_mask
                                                50, -50, -15, # x, y, z positions
                                                # for NED, x is North, y is East, z is Down
                                                0, 0, 0, # x, y, z velocity in m/s
                                                0, 0 ,0 , # x, y, z acceleration
                                                0 ,0 # yaw, yaw_rate
                                                )


# Wait for the drone to reach the target position

flag = False
while True:
    # MAV_CMD_REQUEST_MESSAGE = 512
    # Sending a request for NAV_CONTROLLER_OUTPUT = 62
    the_connection.mav.command_long_send(the_connection.target_system,
                                         the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                         0, 62, 0, 0, 0, 0, 0, 0)

    # NAV_CONTROLLER_OUTPUT = 62
    msg = the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT',blocking=True)
    print(msg)
    if msg.wp_dist > 0:
        flag = True
    if msg.wp_dist < 0.5 and flag:
        print("Reached target waypoint")
        break
    time.sleep(0.5)




# Land the drone
# COMMAND_LONG = 76
# MAV_CMD_NAV_LAND = 21
the_connection.mav.command_long_send(the_connection.target_system,
                                     the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_LAND,
                                        0, 0, 0, 0, 0, 0, 0, 0)
while True:
    # MAV_CMD_REQUEST_MESSAGE = 512
    # Sending a request for LOCAL_POSITION_NED = 32
    the_connection.mav.command_long_send(the_connection.target_system,
                                         the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                         0,
                                         32,
                                         0, 0, 0, 0, 0, 0)
    
    # LOCAL_POSITION_NED = 32
    msg = the_connection.recv_match(type='LOCAL_POSITION_NED',blocking=True)
    # print(msg)
    if msg.z > -0.5:
        break
    time.sleep(0.1)
print("Landed")