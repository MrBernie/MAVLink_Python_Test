from pymavlink import mavutil
import time

# # Start a connection using the simulator's serial port
the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
# the_connection = mavutil.mavlink_connection('udp:localhost:14550')
# the_connection = mavutil.mavlink_connection('/dev/ttyACM0', 9600, timeout = 5)
connected = []

while True:
    msg = the_connection.recv_match(blocking=True)
    systemID = msg.get_srcSystem()
    if systemID not in connected:
        connected.append(systemID)
        print(connected)