import numpy as np
import time
import dlib
import cv2
import socket
from multiprocessing import Process, Value
import serial
import os
import datetime
import base64
import errno


def myrecv(conn,count):
	buf=b''
	while count:
		newbuf = conn.recv(count)
		if not newbuf: return None
		buf += newbuf
		count -= len(newbuf)
	return buf


def shape_to_np(shape, dtype="int"):
	coords = np.zeros((shape.num_parts,2),dtype=dtype)
	for i in range(0,shape.num_parts):
		coords[i] = (shape.part(i).x,shape.part(i).y)
	return coords

#Calculate euclidean distance between two points
def euclidean_dist(ptA,ptB):
	return np.linalg.norm(ptA - ptB)

#Calculate eye aspect ratio
def eye_aspect_ratio(eye):
	#Compute the euclidean distances between the two sets of
	#Vertical eye landmarks (x,y)-coordinates
	A = euclidean_dist(eye[1], eye[5])
	B = euclidean_dist(eye[2], eye[4])
	C = euclidean_dist(eye[0], eye[3])
	ear = (A+B) / (2.0*C)

	#Return the eye aspect ratio
	return ear

#Calculate mouth aspect ratio for fra
def mouth_aspect_ratio(mouth):
	A = euclidean_dist(mouth[1],mouth[7])
	B = euclidean_dist(mouth[2],mouth[6])
	C = euclidean_dist(mouth[3],mouth[5])
	D = euclidean_dist(mouth[0],mouth[4])
	mar = (A+B+C)/(3*D)
	return mar



#Send surveillance video
def sendSur(videoHandler,conn):
#	print("timeout value after setting blocking: ",sock.gettimeout())
	ret,frame = videoHandler.read()
	result,imgencode = cv2.imencode(".jpg",frame, encode_param)
	data = np.array(imgencode)
	stringData = data.tostring()
	stringData = base64.b64encode(stringData)
	stringData +="==EOF==".encode("utf-8")
	slen = len(stringData)
	conn.send(stringData)

#send drowsiness detecting image
def sendImg(frame,conn):
	result,imgencode = cv2.imencode(".jpg",frame, encode_param)
	data = np.array(imgencode)
	stringData = data.tostring()
	stringData = base64.b64encode(stringData)
	stringData +="==EOF==".encode("utf-8")
	conn.send(stringData)


# change cpuserial to 16bits
def getCPUSN():
	f = open('/proc/cpuinfo','r')
	print("cpu info open sucessfully")
	for line in f:
		if line[0:6] == 'Serial':
			cpuserial = line[10:26]
	f.close()
	return cpuserial+"1.1"


#read data from power board serial port
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


#obtain and parse velocity
#if failed to obtain return ""; if succeed return velocity value in string
def getVelocity(serial_handle):
	din = serialRead(serial_handle)
	if din == '':
		return ""
	status_byte = int(din[8:10],16)
	if (status_byte & 128) == 0:
		velocity_source = "CAN"
		print("Speed source: " ,velocity_source)
		velocity = int(din[10:12],16)
		return str(velocity)
	elif (status_byte & 64) == 0:
		velocity_source = "Yingxian"
		print("Speed source: ",velocity_source)
		velocity = int(din[10:12],16)
		return str(velocity)
	elif (status_byte & 32) == 0:
		velocity_source = "GPS"
		print("Speed source: " ,velocity_source)
		velocity = int(din[10:12],16)
		return str(velocity)
	else:
		print("Failed to get speed from CAN, Yingxian or GPS")
		return ""

#send drowsiness information
def sendDrowInfo(drow_par,serial_handle,sock):
	din = serialRead(serial_handle)
	if din == "":
		return False
	else:
		dout = drow_par+din+"".zfill(34)
		sock.send(dout.encode('utf-8'))
		return True

#non blockging receive function
def nonblockingRecv(sock):
	sock.setblocking(0)
	surDet = ""
	try:
		msg = sock.recv(3)
	except socket.error as e:
		sock.setblocking(1)
		err = e.args[0]
		if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
#			print("did not receive any instruction from server")
			pass
		else:
			print(e)
			sys.exit(1)
	else:
		sock.setblocking(1)
		surDet = msg.decode('utf-8')
#		print("received data: ",msg)
	return surDet



EAR_THRESH = 0.23
EAR_CONSEC_FRAMES = 6
MAR_THRESH = 0.5
MAR_CONSEC_FRAMES = 15
PIN_NO = 17

ALM_THR = 15
ALM_CNT_THR = 3
VEL_THR = 30
CE_TM_THR = 200
CE_TM_MAX_THR = 3600
IS_TM_THR = 180

ear_counter = 0
mar_counter = 0
ear = 0
mar = 0
frameSizePara = 2





host = "106.14.160.95"
port =9501

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY),90]
#Flags
FR2R = True
FI2I = False
FIM = False
EYE_BEEP = False
MOU_BEEP = False
PHO_BEEP = False
FAC_BEEP = False
SMK_BEEP = False
# bit destinguishing drowsiness in 3 min for sending drowsiness img
DBIT = False

print("[INFO]Loading resources.........................")
phone_det = cv2.CascadeClassifier(".haarPHN.xml")
face_pre = dlib.shape_predictor("landmark_pre.dat")
face_det = cv2.CascadeClassifier(".haarFace.xml")
smoke_det = cv2.CascadeClassifier(".lbpSMK.xml")
print("[INFO]Opening camera............................")
vc = cv2.VideoCapture(0)
time.sleep(1.0)


ser_intf = serial.Serial(port='/dev/ttyAMA0',baudrate=19200,bytesize=8,timeout=1)
drow_par = 0x4459000100000000

#setup sockets
sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
print("New socket connecting")
sock.connect((host,port))
print("New socket connected")
# send cpu serial number to server
sock.send((getCPUSN()+',').encode('utf-8'))
sock.send("man,".encode('utf-8'))

		
#counter = 0
#set camera exception timer and image send timer
ce_tm = time.time()
is_tm = time.time()
alarm_tm = time.time()
alarm_cnt = 0

while True:
#	t_start = time.time()

	drow_par = 0x4459000100000000
	sock.send("drow,".encode('utf-8'))
#	print("drow branch instr sent, receiving surDet signal......")
	surDet = nonblockingRecv(sock)
#	print('surDet instr received: ',surDet)

#		surveillance branch
	if surDet == "sur":
		print("[INFO]In surveillance mode.......................")
		while True:
			sendSur(vc,sock)
			if nonblockingRecv(sock) == "stp":
				ce_tm = time.time()
				is_tm = time.time()
				alarm_tm = time.time()
				print("[INFO]Return from surveillance mode..............")
				break

#		detecting drowssiness
	else:
		ret,frame = vc.read()
		gray_orig = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		gray = cv2.resize(gray_orig,(0,0),fx=0.5,fy=0.5)
		rects = face_det.detectMultiScale(gray,scaleFactor=1.1,minNeighbors=3,minSize=(30,30),
										  flags=cv2.CASCADE_SCALE_IMAGE)
		if len(rects):
			FAC_BEEP = False
			ce_tm = time.time()
			for (x, y, w, h) in rects:
				left = int(x - 0.52*w)
				left_smoke = x
				top = int(y)
				top_smoke =int(y+0.6*h)
				right = int(x+1.52*w)
				right_smoke = x+w
				bottom = int(y+1.4*h)
				bottom_smoke = int(y+1.4*h)
				
				phone_gray = gray[top:bottom,left:right]
				smoke_gray = gray[top_smoke:bottom_smoke,left_smoke:right_smoke]
				phones = phone_det.detectMultiScale(phone_gray, scaleFactor=1.1,
					minNeighbors=6, minSize=(30, 30),flags=cv2.CASCADE_SCALE_IMAGE)
				smoke_gsts = smoke_det.detectMultiScale(smoke_gray,scaleFactor=1.1,minNeighbors=7,
									minSize=(40,40),flags=cv2.CASCADE_SCALE_IMAGE)
									
				if len(phones):
					for (x1,y1,w1,h1) in phones:
						if w1 > 0.8*w:
							continue
						x1 = frameSizePara*(x1+left)
						y1 = frameSizePara*(y1+top)
						w1 = frameSizePara*w1
						h1 = frameSizePara*h1
#					cv2.putText(frame,"Calling behavior detected",(0,60),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
					PHO_BEEP = True
					drow_par = drow_par|0x20000000
				else:
					PHO_BEEP = False
				if len(smoke_gsts):
#					cv2.putText(frame,"Smoking behavior detected!",(0,120),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
					for (x2,y2,w2,h2) in smoke_gsts:
						x2 = frameSizePara*(x2+left_smoke)
						y2 = frameSizePara*(y2+top_smoke)
						w2 = frameSizePara*w2
						h2 = frameSizePara*h2
						cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (255,0, 0), 2)
					SMK_BEEP = True
					drow_par = drow_par|0x10000000
				else:
					SMK_BEEP = False



				x = frameSizePara*x
				y = frameSizePara*y
				w = frameSizePara*w
				h = frameSizePara*h

				rect = dlib.rectangle(int(x), int(y), int(x + w),
					int(y + h))
				# determine the facial landmarks for the face region, then
				# convert the facial landmark (x, y)-coordinates to a NumPy
				# array
				shape = face_pre(gray_orig, rect)
				shape = shape_to_np(shape)

				#extract the left and right eye coordinates, then use the
				#coordinates to compute the eye aspect ratio for both eyes
				#do the same thing to cal mar
				leftEye = shape[42:48]
				rightEye = shape[36:42]
				mouth = shape[60:68]
				mar = mouth_aspect_ratio(mouth)
				leftEAR = eye_aspect_ratio(leftEye)
				rightEAR = eye_aspect_ratio(rightEye)
				#average the eye aspect ratio together for both eyes
				ear = (leftEAR + rightEAR) / 2.0
			
			if  ear < EAR_THRESH:
				ear_counter +=  1
				if ear_counter >= EAR_CONSEC_FRAMES:
#					cv2.putText(frame,"Eye closed for 1 sec",(0,80),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
					EYE_BEEP = True
					drow_par = drow_par|0x40000000
				else:
					EYE_BEEP = False
			else:
				ear_counter = 0
				EYE_BEEP = False
			if mar >  MAR_THRESH :
				mar_counter += 1
				if mar_counter >= MAR_CONSEC_FRAMES:
#					cv2.putText(frame,"Yawn for 2 secs",(0,100),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
					MOU_BEEP = True
					drow_par = drow_par|0x80000000
				else:
					MOU_BEEP = False
			else:
				mar_counter = 0
				MOU_BEEP = False


		else:
			FAC_BEEP = True
			drow_par = drow_par|0x08000000
#			cv2.putText(frame,"No face detected",(0,20),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)


		if MOU_BEEP or EYE_BEEP or PHO_BEEP or FAC_BEEP :
			resized_frame = cv2.resize(frame,(0,0),fx=0.5,fy=0.5)
#			counter += 1
#			print("counter is: ",counter)
			DBIT = True
			alarm_cnt += 1
#		else:
#			print("No illeal driving detected:)")

#           set alarm on conditions
		if time.time() - alarm_tm > ALM_THR:
			print("setting alarm ")
			velocity = getVelocity(ser_intf)
			if velocity == "":
				print("getVelocity failed, check the connection with power board")
			else:
				v = int(velocity)
				if alarm_cnt > ALM_CNT_THR and velocity > VEL_THR:
					str_dp = str(hex(drow_par))
					ser_intf.write(str_dp[2:].encode('utf-8'))
			alarm_tm = time.time()
			alarm_cnt = 0


#           send detecting results to server
		if time.time()-ce_tm< CE_TM_THR:#190
#			every 3min send an image in which fatigue detected
			if time.time()-is_tm >= IS_TM_THR and DBIT:#180ss
				#IDNC illegal driving behavior with no camera exception
				sock.send("IDNC,".encode('utf-8'))
#				check if drowsiness info sent successfully or not. if not suceeded, it probably because can't get info from power board.
				if sendDrowInfo(drow_par,ser_intf,sock) == False:
					print("Drowsiness info sending failed !")
				sendImg(resized_frame,sock)
				print("Fatigue img sent")
				is_tm = time.time()
				DBIT = False
#				print("RBP: IDNC sent.......................................")
			else:
				# leganl driving behavior
				sock.send("LDXX,".encode('utf-8'))
#				print("RBP:LDXX sent")
		elif time.time()-ce_tm >= CE_TM_MAX_THR:#1440
			#illegal driving behavior sending camera image
			sock.send("IDCI,".encode('utf-8'))
			drow_par = drow_par|0x01000000
#				check if drowsiness info sent successfully or not. if not suceeded, it probably because can't get info from power board.
			if sendDrowInfo(drow_par,ser_intf,sock) == False:
				print("Drowsiness info sending failed !")
			sendImg(resized_frame,sock)
			print("Camera exception img sent")
			is_tm = time.time()
			ce_tm = time.time()
#			print("RBP:IDCI sent............................................")
		else:
			#illegal driving behavior with camera exception no sending image
			sock.send("IDCE,".encode('utf-8'))
#			print("RBP:IDCE sent")

#	cv2.imshow("Frame",frame)
#	key = cv2.waitKey(1) & 0xFF
#	if key == ord("q"):
#		break


#	fps = 1 / (time.time() - t_start)
#	print("FPS: {:.3f}".format(fps))
