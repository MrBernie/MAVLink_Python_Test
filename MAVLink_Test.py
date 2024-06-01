from pymavlink import mavutil
import time
import setup_connection

# Open the Mavproxy with the following command to forward the telemetry message to this Python Script.
# mavproxy.py --master=/dev/ttyUSB0 --console --out=udp:127.0.0.1:14550

# Wait for the first heartbeat 
# This sets the system and component ID of remote system for the link

def setup_connection(Wireless = False):
    if(Wireless == True):
        the_connection = mavutil.mavlink_connection('udp:localhost:14550')
    else:
        the_connection = mavutil.mavlink_connection('/dev/ttyACM0', timeout = 2)


    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" 
        % (the_connection.target_system, the_connection.target_component))

    msg = the_connection.recv_match(type = 'HEARTBEAT',blocking=True)
    print(msg)
    return the_connection

def send_customized_command_int(the_connection):
    print("Sending command_int")
    the_connection.mav.command_int_send(the_connection.target_system,
                                the_connection.target_component,
                                0,
                                32001,
                                0,0,
                                1.1,2.2,3.3,4.4,0,0,0)

def send_customized_command_long(the_connection):
    print("Sending command_long")
    the_connection.mav.command_long_send(the_connection.target_system,
                                         the_connection.target_component,
                                         32001,
                                         0,
                                         1,2,3,4,5,6,7)
    
def send_raw_imu_msg_request(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system,
                                        the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                        0,
                                        mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
                                        1000000, #in microseconds
                                        0,0,0,0,0)
    
def get_raw_imu_msg(the_connection):
    msg = the_connection.recv_match(type = 'RAW_IMU',blocking=True)
    if(msg!=None):
        print(f"System : {msg.get_srcSystem()}",msg)

def get_highres_imu_msg(the_connection):
    msg = the_connection.recv_match(type = 'HIGHRES_IMU',blocking=True)
    if(msg!=None):
        print(f"System : {msg.get_srcSystem()}",msg)

def get_msg(the_connection,systemID):
    while True:
        msg = the_connection.recv_match(blocking=True)
        if msg is not None and msg.get_srcSystem() == systemID:
            print(f"(System : {msg.get_srcSystem()}) (Component : {msg.get_srcComponent()}) ", msg)
            return

def get_ack_msg(the_connection):
    msg = the_connection.recv_match(type = 'COMMAND_ACK',blocking=True)
    if(msg!=None):
        print(f"RAW_IMU {msg}",end = "\n")

def get_command_msg(the_connection):
    msg = the_connection.recv_match(type = 'COMMAND_LONG',blocking=True)
    if(msg!=None):
        print(f"(System : {msg.get_srcSystem()}) (Component : {msg.get_srcComponent()}) ",msg)
    return [msg.param1,msg.param2,msg.param3,msg.param4,msg.param5,msg.param6,msg.param7]

def send_systime_msg(the_connection):
    time_unix_usec = int(time.time())
    time_boot_ms = int(time.time())
    the_connection.mav.system_time_send(time_unix_usec, time_boot_ms)
    print(f"Sending SYSTEM_TIME: {time_unix_usec}")

def get_systime_msg(the_connection, systemID):
    while True:
        msg = the_connection.recv_match(type = 'SYSTEM_TIME', blocking=True)
        if msg is not None and msg.get_srcSystem() == systemID:
            print(f"(System : {msg.get_srcSystem()}) (Component : {msg.get_srcComponent()}) ", msg)
            return

# Set up TEST constant to test different cases
# Test1: RPI requests data from the Pixhawk.
# Test2: Pixhawk sends the data to the RPI.
# Test3: Pixhawk requests data from the RPI, and RPI replies.
# Test4: RPI sends data to the Pixhawk.
# Following Tests are for the wireless communication, MAVProxy is required.
# command line input: 
# mavproxy.py --master=/dev/ttyUSB0 --console --out=udp:127.0.0.1:14550
# Test5: GCS requests data from the Pixhawk.
# Test6: Pixhawk sends data to the GCS.
# Test7: Pixhawk requests data from the GCS, and GCS replies.
# Test8: GCS sends data to the Pixhawk.
# Test-1: To test other mavlink messages

TEST = 4
if __name__=='__main__':
    
    if TEST == 1:
        # 1. RPI request and Pixhawk Replies (system default)
        the_connection = setup_connection(False)
        send_raw_imu_msg_request(the_connection)
        get_ack_msg(the_connection)
        while True:
            get_raw_imu_msg(the_connection)

    elif TEST == 2:
        # 2. Pixhawk Pushes to RPI
        the_connection = setup_connection(False)    
        while True:
            get_msg(the_connection,155)

    elif TEST == 3:
        # 3. Pixhawk Request and RPI Replies
        the_connection = setup_connection(False)
        while True:
            command = get_command_msg(the_connection)
            if(command[0]==2):
                send_systime_msg(the_connection)
            get_systime_msg(the_connection,155)

    elif TEST == 4:
        # 4. RPI Pushes to Pixhawk
        the_connection = setup_connection(False)
        while True:
            send_customized_command_long(the_connection)
            get_command_msg(the_connection)
            time.sleep(1)

    elif TEST == 5:
        # 5. GCS Request and Pixhawk Replies
        the_connection = setup_connection(True)
        send_raw_imu_msg_request(the_connection)
        get_ack_msg(the_connection)
        while True:
            get_raw_imu_msg(the_connection)

    elif TEST == 6:
        # 6. Pixhawk Pushes to GCS
        the_connection = setup_connection(True)
        while True:
            get_msg(the_connection,155)

    elif TEST == 7:
        # 7. Pixhawk Request and GCS Replies
        the_connection = setup_connection(True)
        while True:
            command = get_command_msg(the_connection)
            if(command[0]==2):
                send_systime_msg(the_connection)
            get_systime_msg(the_connection,155)

    elif TEST == 8:
        # 8. GCS Pushes to Pixhawk
        the_connection = setup_connection(True)
        while True:
            send_customized_command_long(the_connection)
            get_command_msg(the_connection)
            time.sleep(1)