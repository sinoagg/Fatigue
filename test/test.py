import socket
import os
import urllib.request
from urllib.request import urlopen
import cv2
import time
import serial
import struct

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
	cnt = 0
	while 1:
		din = serial_handle.read(28)
		cnt += 1
		if len(din) == 0 and cnt < 8:
			#			print("cnt is ",cnt)
			continue
		if len(din) ==28 and  din[0]==52 and din[1] == 52 and din[2] == 53 and din[3] == 57:
			return din.decode('utf-8')
		elif cnt == 8:
			print("Can not receive data from power board")
			return ""
		serial_handle.flushInput()


def getVelocity(serial_handle):
	din = serialRead(serial_handle)
	if din == '':
		print("Can not read data from serial port, please check port connection")
		print("Failed to get speed from CAN, Yingxian or GPS")
		return
	status_byte = int(din[8:10],16)
	print("Din for speed: ",din)
	if (status_byte & 128) == 0:
		velocity_source = "CAN"
		print("Speed source: " ,velocity_source)
		velocity = int(din[10:12],16)
		print("velocity is: ",velocity)
	elif (status_byte & 64) == 0:
		velocity_source = "Yingxian"
		print("Speed source: ",velocity_source)
		velocity = int(din[10:12],16)
		print("velocity is: ",velocity)
	elif (status_byte & 32) == 0:
		velocity_source = "GPS"
		print("Speed source: " ,velocity_source)
		velocity = int(din[10:12],16)
		print("velocity is: ",velocity)
	else:
		print("Failed to get speed from CAN, Yingxian or GPS")

def getL(lstr):
	lv = list()
	for i in  range(4):
		t = int(lstr[6-i*2:8-i*2],16)
		lv.append(t)
	data = struct.unpack('<f',struct.pack('4B',*lv))[0]
	return  data



def checkGPS(serial_handle):
	din = serialRead(serial_handle)
	if din == '':
		print("Can not read data from serial port, please check port connection")
		print("GPS -------------- Failed")
		return
	print("Din for GPS: ",din)
	status_byte = int(din[8:10],16)
	if (status_byte & 16) == 16:
		print("GPS -------------- Check")
		latitude = din[12:20]
		longitude = din[20:28]
		print("lattitude is ", getL(latitude))
		print("longitude is ",getL(longitude))
	else:
		print("GPS -------------- Failed")



serial_handle = serial.Serial(port='/dev/ttyAMA0',baudrate=19200,bytesize=8,timeout=1)
check4G()
getVelocity(serial_handle)
checkGPS(serial_handle)

