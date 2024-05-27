from pymavlink import mavutil
import time

# Start a connection using the simulator's serial port
# the_connection = mavutil.mavlink_connection('tcp:localhost:5762',source_system=2)
the_connection = mavutil.mavlink_connection('udp:localhost:14550',source_system=127,source_component=0)

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link

the_connection.wait_heartbeat()
print(the_connection)
print("Heartbeat from system (system %u component %u)" 
      % (the_connection.target_system, the_connection.target_component))

msg = the_connection.recv_match(type = 'HEARTBEAT',blocking=True)
print(msg)

while(True):
    the_connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,  # type
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # autopilot: this is not a autopilot
        0,  # base_mode
        0,  # custom_mode
        mavutil.mavlink.MAV_STATE_BOOT,  # system_status
        3  # mavlink_version
    )
#     msg = the_connection.recv_match(type = "COMMAND_LONG",blocking=True)
#     print(msg)