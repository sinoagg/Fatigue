import socket
import os
import urllib.request
from urllib.request import urlopen
import cv2
import time

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

def checkPowerBoard()


def serialRead(serial_handle):
	serial_handle.flushInput()
	while 1:
		din = serial_handle.read(28)
		if din[0]==52 and din[1] == 52 and din[2] == 53 and din[3] == 57:
			break
		serial_handle.flushInput()
	dout = din.decode('utf-8')
	return dout


def getVelocity():
	din = serialRead()
	status_byte = int(din[8:10])
	velocity_source = ""
	if (status_byte & 128) == 128:
		velocity_source = "CAN"
		print("Speed source: " velocity_source)
		velocity = int(din[10:12])
		print("Speed is: ", velocity)
	elif (status_byte & 64) == 64:
		velocity_source = "Yingxian"
		print("Speed source: " velocity_source)
		velocity = int(din[10:12])
		print("Speed is: ", velocity)
	elif (status_byte & 32) == 32:
		velocity_source = "Yingxian"
		print("Speed source: " velocity_source)
		velocity = int(din[10:12])
		print("Speed is: ", velocity)
	else:
		print("Failed to get speed from CAN, Yingxian, GPS")
		velocity = 0

def checkGPS():
	din = serialRead()
	status_byte = int(din[8:10])
	if (status_byte & 16) == 16:
		print("GPS -------------- Check")
	else:
		print("GPS ------------ Failed")


check4G()
checkCam()
getVelocity()
checkGPS()

