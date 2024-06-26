from pymavlink import mavutil
import time

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" 
      % (the_connection.target_system, the_connection.target_component))




# Set Mode to GUIDED
# MAV_CMD_DO_SET_MODE = 176
# MAV_MODE_GUIDED_DISARMED = 4
the_connection.mav.command_long_send(the_connection.target_system, 
                                     the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                                     0, 1, 4, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True)
print(msg)




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




# Take off to 10m height
# COMMAND_LONG = 76
# MAV_CMD_NAV_TAKEOFF = 22
the_connection.mav.command_long_send(the_connection.target_system, 
                                     the_connection.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
                                    0, 0, 0, 0, 0, 0, 0, 10)
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True, timeout=20)
print(msg)




time.sleep(15)
# Land the drone
# COMMAND_LONG = 76
# MAV_CMD_NAV_LAND = 21
the_connection.mav.command_long_send(the_connection.target_system,
                                        the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                                        0, 0, 0, 0, 0, 0, 0, 0)
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True, timeout=30)
print(msg)




# DIsarm the drone
# COMMAND_LONG = 76
# MAV_CMD_COMPONENT_ARM_DISARM = 400
the_connection.mav.command_long_send(the_connection.target_system, 
                                     the_connection.target_component,
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                    0, 0, 0, 0, 0, 0, 0, 0)
# COMMAND_ACK = 77
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True, timeout=30)
print(msg)