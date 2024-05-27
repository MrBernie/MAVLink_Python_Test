import time
import serial
from pymavlink import mavutil
import setup_connection

# Open the Mavproxy with
# mavproxy.py --master=/dev/ttyACM0 --console --out=udp:127.0.0.1:14550
# Create a MAVLink connection
# the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
# the_connection = mavutil.mavlink_connection('udp:localhost:14550',source_system=155)
the_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
# the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=115200)
# the_connection = setup_connection.scan_connections(115200,5,20,0)

def send_system_time_message():
    time_unix_usec = int(time.time())
    time_boot_ms = int(time.time())

    # send the SYSTEM_TIME message
    the_connection.mav.system_time_send(time_unix_usec, time_boot_ms)
    print(f"Sending SYSTEM_TIME: {time_unix_usec}")

def send_change_uart_protocol_message():
    # Send a message to change the protocol of the telemetry link
    the_connection.mav.param_set_send(
        the_connection.target_system,
        the_connection.target_component,
        'SERIAL1_PROTOCOL',
        -1,
        mavutil.mavlink.MAV_PARAM_TYPE_INT8
    )

# Read MAVLink Raw IMU
def read_mavlink_IMU():
    # message = connection.recv_match(type = 'RAW_IMU')
    message = the_connection.recv_match(type = 'HIGHRES_IMU',blocking = True)
    if message:
        # Process the received message
        # print("Received message: ", message)
        print("X Acceleration: ", message.xacc)
        print("Y Acceleration: ", message.yacc)
        print("Z Acceleration: ", message.zacc)
        print("X Gyro: ", message.xgyro)
        print("Y Gyro: ", message.ygyro)
        print("Z Gyro: ", message.zgyro)
        print("X Magnetic field: ", message.xmag)
        print("Y Magnetic field: ", message.ymag)
        print("Z Magnetic field: ", message.zmag)


# Example usage
    # Send a heartbeat message
    # heartbeat = mavutil.mavlink.MAVLink_heartbeat_message(
        # mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0,mavlink_version=3)

# send_change_uart_protocol_message()
if  __name__ == "__main__":
    while True:
        send_system_time_message()
        # read_mavlink_IMU()
        # msg = the_connection.recv_match(type='SYSTEM_TIME',blocking=True)
        # if msg.get_srcSystem() == 5:
        #     print(f"Received system time from {msg.get_srcSystem()}",msg)
        time.sleep(0.5)

