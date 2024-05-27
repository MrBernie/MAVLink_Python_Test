from numpy import uint8
import serial
import string
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)

while(1):
	# s = ser.read(1000)
	# int_value = int.from_bytes(s, byteorder='big')
	# print(int_value)

	# s = ser.read(1000) 
	# print(s)

	# message = input("Input: ")
	# ser.write(message.encode('utf-8')) 
	# msg_to_pixhawk = "hello from raspberry pi"
	# msg_to_pixhawk = msg_to_pixhawk.encode('ascii')
	# str_len = len(msg_to_pixhawk)
	# msg_to_pixhawk = b'\xFC' + str_len.to_bytes(1,'little') + msg_to_pixhawk

	# ser.write(msg_to_pixhawk)

	s = ser.readline()
	try:
		print(s.decode('ascii').strip())
	except:
		pass
