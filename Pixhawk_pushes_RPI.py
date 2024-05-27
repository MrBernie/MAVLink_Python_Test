from pymavlink import mavutil
import time
import setup_connection

# # Start a connection using the simulator's serial port
# the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
# the_connection = mavutil.mavlink_connection('udp:localhost:14550')
the_connection = mavutil.mavlink_connection('/dev/ttyACM0', timeout = 5)
# the_connection = setup_connection.scan_connections()

msg = the_connection.recv_match(type = 'HEARTBEAT',blocking=True)
print(f"From system : {msg.get_srcSystem()} ",msg)

while True:
    msg = the_connection.recv_match(blocking=True)
    if msg.get_srcSystem() == 155:
        print(f"From system : {msg.get_srcSystem()} ",msg)