from pymavlink import mavutil
import time
import setup_connection

# Start a connection using the simulator's serial port, opening MavProxy
# in the linux shell
# mavproxy.py --master=/dev/ttyACM1 --baudrate 115200 --out 127.0.0.1:14550 --console

# # Start a connection using the simulator's serial port
# the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
# the_connection = mavutil.mavlink_connection('udp:localhost:14550')
the_connection = mavutil.mavlink_connection('/dev/ttyACM0', 115200, timeout = 5)
# the_connection = mavutil.mavlink_connection('/dev/ttyUSB0',timeout = 5)
# the_connection = setup_connection.scan_connections(115200,5,20,0)
# For wireless radio
# the_connection = mavutil.mavlink_connection('/dev/ttyACM1', 115200, timeout = 3)

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link

the_connection.wait_heartbeat()
print(the_connection)
print("Heartbeat from system (system %u component %u)" 
      % (the_connection.target_system, the_connection.target_component))

msg = the_connection.recv_match(type = 'HEARTBEAT',blocking=True)
print(msg)


# msg = the_connection.recv_match(type = 'RAW_IMU',blocking=True)
# print(msg)

the_connection.mav.command_long_send(the_connection.target_system,
                                the_connection.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0,
                                mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
                                10000, #in microseconds
                                0,0,0,0,0)
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True,timeout=1)
print(f"RAW_IMU {msg}",end = "\n")


the_connection.mav.command_long_send(the_connection.target_system,
                                the_connection.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0,
                                mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                                500000, #in microseconds
                                0,0,0,0,0)
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True,timeout=1)
print(f"GPS_RAW_INT {msg}",end = "\n")

the_connection.mav.command_long_send(the_connection.target_system,
                                the_connection.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0,
                                mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,
                                500000, #in microseconds
                                0,0,0,0,0)
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True,timeout=1)
print(f"SCALED_IMU {msg}",end = "\n")

the_connection.mav.command_long_send(the_connection.target_system,
                                the_connection.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0,
                                mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,
                                500000, #in microseconds
                                0,0,0,0,0)
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True,timeout=1)
print(f"SCALED_IMU2 {msg}",end = "\n")

the_connection.mav.command_long_send(the_connection.target_system,
                                the_connection.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0,
                                mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3,
                                500000, #in microseconds
                                0,0,0,0,0)
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True,timeout=1)
print(f"SCALED_IMU3 {msg}",end = "\n")

# the_connection.mav.command_long_send(the_connection.target_system,
#                                 the_connection.target_component,
#                                 mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
#                                 0,
#                                 mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
#                                 500000, #in microseconds
#                                 0,0,0,0,0)
# msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True)
# print(f"LOCAL_POSITION_NED {msg}",end = "\n")

the_connection.mav.command_long_send(the_connection.target_system,
                                the_connection.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0,
                                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                                500000, #in microseconds
                                0,0,0,0,0)
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True,timeout=1)
print(f"ATTITUDE {msg}",end = "\n")

the_connection.mav.command_long_send(the_connection.target_system,
                                the_connection.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0,
                                mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
                                1000000, #in microseconds
                                0,0,0,0,0)
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True,timeout=1)
print(f"HIGHRES_IMU {msg}",end = "\n")
# Create a COMMAND_LONG message

# Send the message
# the_connection.mav.send(msg)

# the_connection.mav.command_long_send(the_connection.target_system,
#                                 the_connection.target_component,
#                                 mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
#                                 0,
#                                 mavutil.mavlink.MAVLINK_MSG_ID_HIL_SENSOR,
#                                 1000000, #in microseconds
#                                 0,0,0,0,0)
# msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True)
# print(f"HIL_SENSOR {msg}",end = "\n")

the_connection.mav.command_long_send(the_connection.target_system,
                                the_connection.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0,
                                mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                                500000, #in microseconds
                                0,0,0,0,0)
msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True,timeout=1)
print(f"LOCAL POSITION NED {msg}",end = "\n")

while(1):
      msg = the_connection.recv_match(blocking = True)
      
      # if msg.get_srcSystem() == 155:

      # if(msg.get_type() == 'SYSTEM_TIME'):
      #       print(msg, end = '\n')
      if(msg.get_type() == 'SCALED_IMU'):
            print(msg, end = '\n')
      # if(msg.get_type() == 'SCALED_IMU2'):
      #       print(msg, end = '\n')
      # if(msg.get_type() == 'SCALED_IMU3'):
      #       print(msg, end = '\n')
      # if(msg.get_type() == 'RAW_IMU'):
            # print(msg, end = '\n') 
      # if(msg.get_type() == 'HIGHRES_IMU'):
      #       print(f"HIGHRES_IMU from {msg.get_srcSystem()} ",msg)
      # if(msg.get_type() == 'GPS_RAW_INT'):
      #       print(msg, end = '\n')
      # if(msg.get_type() == 'LOCAL_POSITION_NED'):
            # print(msg, end = '\n')
      # if(msg.get_type() == 'ATTITUDE'):
      #       print(msg, end = '\n')
      # if(msg.get_type() == 'HIL_SENSOR'):
      #       print(msg, end = '\n')    