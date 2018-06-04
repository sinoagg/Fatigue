import socket
import os
import urllib.request
from urllib.request import urlopen
import cv2
import time
import serial

def check4G():
	sock_p_counter = 0
	sock_counter = 0
	while (sock_counter <30):
		try:
			urlopen("https://www.baidu.com/")
		except urllib.error.URLError as e:
			sock_counter += 1
		else:
			sock_counter += 1
			sock_p_counter += 1
	if sock_p_counter > 20:
		print("4G ---------------- Check")
	else:
		print("4G ---------------- Failed")


#def checkCam():
#	vc = cv2.VideoCapture(0)
#	time.sleep(1.0)
#	tm = time.time()
#	failed = False
#	while (time.time() - tm < 3):
#		try:
#			ret,frame = vc.read()
#		except exception as e:
#			failed = True
#	vc.release()
#	if failed:
#		print("Camera ------------- Failed")
#	else:
#		print("Camera ------------- Check")


def serialRead(serial_handle):
	serial_handle.flushInput()
	counter = 0
	while 1:
		counter += 1
		din = serial_handle.read(28)
#		print("din in serialRead----------",len(din))
		if len(din) == 0:
			time.sleep(0.5)
			if counter == 30:
				print("Can not read data from serial port, please check port connection")
				break
			else:
				continue
		if din[0]==52 and din[1] == 52 and din[2] == 53 and din[3] == 57 :
			break
		if counter == 30 :
			print("Can not read data from serial port, please check port connection")
			break
		serial_handle.flushInput()

	dout = din.decode('utf-8')
	return dout


def getVelocity(serial_handle):
	din = serialRead(serial_handle)
	status_byte = int(din[8:10],16)
	print("Din for speed: ", din)
	velocity_source = ""
	if (status_byte & 128) == 128:
		velocity_source = "CAN"
		print("Speed source: " ,velocity_source)
		velocity = int(din[10:12],16)
		print("Speed is: ", velocity)
	elif (status_byte & 64) == 64:
		velocity_source = "Yingxian"
		print("Speed source: ",velocity_source)
		velocity = int(din[10:12],16)
		print("Speed is: ", velocity)
	elif (status_byte & 32) == 32:
		velocity_source = "Yingxian"
		print("Speed source: " ,velocity_source)
		velocity = int(din[10:12],16)
		print("Speed is: ", velocity)
	else:
		print("Failed to get speed from CAN, Yingxian, GPS")
		velocity = 0

def checkGPS(serial_handle):
	din = serialRead(serial_handle)
	print("Din for GPS: ",din)
	status_byte = int(din[8:10],16)
	if (status_byte & 16) == 16:
		print("GPS -------------- Check")
	else:
		print("GPS -------------- Failed")


serial_handle = serial.Serial(port='/dev/ttyAMA0',baudrate=19200,bytesize=8,timeout=1)
check4G()
getVelocity(serial_handle)
checkGPS(serial_handle)

