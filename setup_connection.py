from pymavlink import mavutil

def check_connection(port,baud=None,timeout=None,sysID=None,compID=None):
    try:
        # Attempt to create a connection
        if baud==None or timeout==None:
            connection = mavutil.mavlink_connection(port)
        else:
            connection = mavutil.mavlink_connection(port, baud, timeout, sysID, compID)
        
        # Wait for the first heartbeat
        connection.wait_heartbeat()
        print(f"Successfully connected on port {port}")
        
        return connection
    except Exception as e:
        # print(f"Failed to connect on port {port}")
        return None

def scan_connections(baud = 9600, timeout = 5, sysID = 2, compID = 0):
    # List of ports to check
    ports = [['/dev/ttyACM0',baud,timeout,sysID,compID], 
             ['/dev/ttyACM1',baud,timeout,sysID,compID],
             ['/dev/ttyACM2',baud,timeout,sysID,compID],
             ['/dev/ttyACM3',baud,timeout,sysID,compID],
             ['tcp:localhost:5762',None,None,sysID,compID],
             ['udp:localhost:14550',None,None,sysID,compID]]
    
    for port in ports:
        connection = check_connection(*port)
        if connection != None:
            return connection
    return None

if __name__ == "__main__":
    scan_connections()