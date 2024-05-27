from pymavlink import mavutil
import time
import setup_connection

# Open the Mavproxy with the following command to forward the telemetry message to this Python Script.
# mavproxy.py --master=/dev/ttyUSB0 --console --out=udp:127.0.0.1:14550

# Start a connection using the simulator's serial port
# the_connection = mavutil.mavlink_connection('tcp:localhost:5762',source_system=2)
the_connection = mavutil.mavlink_connection('udp:localhost:14550',source_system=180)
# the_connection = setup_connection.scan_connections(115200,5,20,0)
# the_connection = mavutil.mavlink_connection('/dev/ttyACM0', timeout = 2)
# the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', timeout = 2, source_system=200)
# print("connected")

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" 
      % (the_connection.target_system, the_connection.target_component))

msg = the_connection.recv_match(type = 'HEARTBEAT',blocking=True)
print(msg)

while(True):
    msg = the_connection.recv_match(blocking=True)
    # print("Received message from system %d: "%msg.get_srcSystem())
    # if msg.get_type() == "SYSTEM_TIME":
        # print(f"Received system time from system {msg.get_srcSystem()}",msg)
    if msg.get_type() == "COMMAND_LONG":
        # print(msg)
        if msg.param1 == 2:
            the_connection.mav.system_time_send(
                int(time.time()),  # time_boot_us
                int(time.time())  # time_unix_ms
            )
            print(f"send system time {int(time.time())}")