from pymavlink import mavutil
import time

# Start a connection using the simulator's serial port
the_connection = mavutil.mavlink_connection('tcp:localhost:5762',source_system=2)
# the_connection = mavutil.mavlink_connection('udp:localhost:14550',source_system=127,source_component=0)

while(True):
    msg = the_connection.recv_match(blocking = True)
    if msg.get_srcSystem() == 2:
        print("Received message from system %d: "%msg.get_srcSystem(), msg)