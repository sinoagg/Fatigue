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
import binascii
import sys
import struct
from math import sin, cos, radians

clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(3,3))
EYE_BEEP = False
MOU_BEEP = False
PHO_BEEP = False
FAC_BEEP = False
SMK_BEEP = False
CAM_NBEEP = False
# bit destinguishing drowsiness in 3 min for sending drowsiness img
DBIT = False
drow_par = 0x00

scaler = 4

print("[INFO]Loading resources.........................")
phone_det = cv2.CascadeClassifier(".haarPHN.xml")
face_pre = dlib.shape_predictor("landmark_pre.dat")
face_det = cv2.CascadeClassifier(".haarFace.xml")
smoke_det = cv2.CascadeClassifier(".lbpSMK.xml")
print("[INFO]Opening camera............................")
vc = cv2.VideoCapture(0)
time.sleep(1.0)


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
	resized_f = cv2.resize(frame,(0,0),fx=0.5,fy=0.5)
	gray = cv2.cvtColor(resized_f,cv2.COLOR_BGR2GRAY)
	result,imgencode = cv2.imencode(".jpg",gray, encode_param)
	data = np.array(imgencode)
	stringData = data.tostring()
	stringData = base64.b64encode(stringData)
	stringData +="D5KSN".encode("utf-8")
	slen = len(stringData)
	conn.send(stringData)

#send drowsiness detecting image
def sendImg(frame,conn,tm_stmp):
	resized_f = cv2.resize(frame,(0,0),fx=0.5,fy=0.5)
	gray = cv2.cvtColor(resized_f,cv2.COLOR_BGR2GRAY)
	result,imgencode = cv2.imencode(".jpg",gray, encode_param)
	data = np.array(imgencode)
	stringData = data.tostring()
	stringData = base64.b64encode(stringData)
#	tailStr = base64.b64encode("stmp"+tm_stmp+"D5")
	tailStr = ("stmp"+tm_stmp+"D5KSN").encode('utf-8')
	stringData += tailStr
	conn.send(stringData)


# change cpuserial to 16bits
def getCPUSN():
	f = open('/proc/cpuinfo','r')
	print("cpu info open sucessfully")
	for line in f:
		if line[0:6] == 'Serial':
			cpuserial = line[10:26]
	f.close()
	return cpuserial


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
#	T ---
#for test comment next line, and add a second line
#	din = serialRead(serial_handle)
	din = "44590188003042203BC342E85605"
	
	
	
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

def convertGPS(gps):
	gps_la = gps[0:8]
	gps_lo = gps[8:16]
	list_la = []
	list_lo = []
	for i in range(4):
		list_la.append(int(gps_la[6-i*2:8-i*2],16))
	la = str(struct.unpack('<f', struct.pack('4B', *list_la))[0]).zfill(20)
#	print("function ",la)
	for i in range(4):
		list_lo.append(int(gps_lo[6-i*2:8-i*2],16))
	lo = str(struct.unpack('<f', struct.pack('4B', *list_lo))[0]).zfill(20)
#	print("function ",lo)
	return la,lo


##send drowsiness information
#def sendDrowInfo(drow_par,serial_handle,sock):
#	din = serialRead(serial_handle)
#	if din == "":
#		return False
#	else:
#		dp_hex = str(hex(drow_par))[2:]
#		dout = dp_hex +din[8:]+"".zfill(34)+','
#		sock.send(dout.encode('utf-8'))
#		return True

#non blockging receive function
def nonblockingRecv(sock):
	sock.setblocking(0)
	surDet = ""
	try:
		msg = myrecv(sock,38)
#		msg = sock.recv(38)
	except socket.error as e:
		sock.setblocking(1)
		err = e.args[0]
		if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
			print("did not receive any instruction from server")
#			T ---
#
			print(e)
		
		
#			pass
		else:
			print(e)
			sys.exit(1)
	else:
		sock.setblocking(1)
		surDet = msg.decode('utf-8')
#		print("received data: ",msg)
	print("msg received in msg ++++++++++++++++++++++++++++++++++")
	print(surDet)
	return surDet

#package length is 43 bytes
def CRC(msg):
	msg_crc = msg[-9:-5]
	before_crc = msg[:-9]
	crc_cal = str(hex(binascii.crc_hqx(before_crc.encode(),0x0000)))[2:]
	crc_cal = crc_cal.zfill(4)
	if crc_cal == msg_crc:
		return True
	else:
		return False

def packParse(msg):
	if CRC(msg) and msg[4] == "1":
		print("/////////////////////////////////////////////////////")
		print("/////////////////////////////////////////////////////")
		print("/////////////////////////////////////////////////////")
		print(msg[5:9])
		return msg[5:9]
	else:
		print("CRC check failed or wrong package")
		return ""



# with CRC
def setSendPack(cpu_no,plen,pro_ver,instr,par_list,track_ID,sock):
	pack_head = "DY18"
	pack_tail = "D5KSN"
	msg_before_crc = pack_head+plen+cpu_no+pro_ver+instr+par_list+track_ID
	crc_cal = str(hex(binascii.crc_hqx(msg_before_crc.encode(),0x0000)))[2:]
	crc = crc_cal.zfill(4)
	msg = msg_before_crc+crc+pack_tail
	print(msg)
	sock.send(msg.encode('utf-8'))


def rotateImage(image, angle):
	if angle == 0: return image
	height, width = image.shape[:2]
	rot_mat = cv2.getRotationMatrix2D((width/2, height/2), angle, 0.9)
	result = cv2.warpAffine(image, rot_mat, (width,height),flags=cv2.INTER_LINEAR)
	return result

def rotatePoint(pos, img, angle):
	if angle == 0: return pos
	x = pos[0] - img.shape[1]*0.4
	y = pos[1] - img.shape[0]*0.4
	nx = x*cos(radians(angle)) + y*sin(radians(angle)) + img.shape[1]*0.4
	ny = -x*sin(radians(angle)) + y*cos(radians(angle)) + img.shape[0]*0.4
	return int(nx), int(ny), pos[2], pos[3]

def rotateShape(shape,img, angle):
	if angle == 0: return shape
	result =  np.zeros_like(shape)
	index = 0
	height, width = img.shape[:2]
	rot_mat = cv2.getRotationMatrix2D((width/2, height/2), angle, 1.111)
	for (x,y) in shape:
		nx = rot_mat[0,0]*x+rot_mat[0,1]*y+rot_mat[0,2]
		ny = rot_mat[1,0]*x+rot_mat[1,1]*y+rot_mat[1,2]
		result[index] = [nx,ny]
		index += 1
	return result



def faceDetAndLandmark(resized_gray,clahe,frame):
	for angle in [0, 20, -20]:
		rmouth_eye = ()
		rimg = rotateImage(gray, angle)
		detected = face_det.detectMultiScale(rimg,scaleFactor = 1.2,minNeighbors= 4,minSize=(25, 25),flags=cv2.CASCADE_SCALE_IMAGE )
		if len(detected):
			for x,y,w,h in detected[-1:]:
				bottom = int(y + 1.2*h)
				roi = gray[y:bottom,x:x+w]
				gray[y:bottom,x:x+w] = clahe.apply(roi)
				rect = dlib.rectangle(int(x), int(y), int(x + w),int(y + 1.2*h))
				shape = face_pre(rimg,rect)
			
			shape = shape_to_np(shape)
			rmouth_eye = rotateShape(shape[36:68],gray,-angle)
			detected = [rotatePoint(detected[-1], gray, -angle)]
			rightEye = rmouth_eye[0:6]
			leftEye = rmouth_eye[6:12]
			mouth = rmouth_eye[12:32]
			for x, y, w, h in detected[-1:]:
				x = x * scaler
				y = y * scaler
				w = w * scaler
				h = h * scaler
				leftEye *= scaler
				rightEye *= scaler
				mouth *= scaler
				cv2.rectangle(frame, (x, y), (x+w, int(y+1.2*h)), (0,255,0), 2)
				for (x,y) in leftEye:
					cv2.circle(frame,(x,y),3,(0,0,255),-1)
				for (x,y) in rightEye:
					cv2.circle(frame,(x,y),3,(0,0,255),-1)
				for(x,y) in mouth:
					cv2.circle(frame,(x,y),3,(0,0,255),-1)
			break

	return rmouth_eye,detected



def phoneDet(face_rect, gray,phone_det):
	for (x, y, w, h) in face_rect:
		left = int(x - 0.52*w)
		top = int(y)
		right = int(x+1.52*w)
		bottom = int(y+1.4*h)
		phone_gray = gray[top:bottom,left:right]
		phones_rect = phone_det.detectMultiScale(phone_gray, scaleFactor=1.1,minNeighbors=4, minSize=(18, 18),flags=cv2.CASCADE_SCALE_IMAGE)

	return phones_rect[-1:]

def smkDet(face_rect, gray,smoke_det):
	for (x, y, w, h) in face_rect:
		left = x
		right = x+w
		top =int(y+0.7*h)
		bottom = int(y+1.3*h)
		smk_gray = gray[top:bottom,left:right]
		smk_rect = smoke_det.detectMultiScale(smk_gray, scaleFactor=1.1,minNeighbors=4, minSize=(22, 22),flags=cv2.CASCADE_SCALE_IMAGE)
	
	return smk_rect[-1:]

def initPar():
	EYE_BEEP = False
	MOU_BEEP = False
	PHO_BEEP = False
	FAC_BEEP = False
	SMK_BEEP = False
	CAM_NBEEP = False
	DBIT = False
	drow_par = 0x00






PIN_NO = 17
pro_ver = "0220"

ear_counter = 0
mar_counter = 0
ear = 0
mar = 0

phn_cnt = 0
phn_alarm_thresh = 3
smk_cnt = 0
smk_alarm_thresh = 3
fac_cnt = 0
fac_alarm_thresh = 2
cam_cnt = 0
cam_exc_thresh = 3
ear_cnt = 0
ear_thresh = 0.23
ear_consec_frames = 6
mar_cnt = 0
mar_thresh = 0.5
mar_consec_frames = 15
velocity_thresh = 30



host = "106.14.160.95"
port = 9501

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY),90]


ser_intf = serial.Serial(port='/dev/ttyAMA0',baudrate=19200,bytesize=8,timeout=1)
cpu_no = getCPUSN()
track_ID = str(int(time.time()))

#setup sockets
sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
print("New socket connecting")
sock.connect((host,port))
print("New socket connected")



# T ---
#
print("sending man instr")

setSendPack(cpu_no,"0035",pro_ver,"04","1002", track_ID,sock)
			   
#recognize successfully

# T ---
#
print("sending RCRD instr")

setSendPack(cpu_no,"0035",pro_ver,"04","4001", track_ID,sock)
pack_par = "00000000"+str(int(time.time()))

# T ---
#
print("sending face recognition package")

setSendPack(cpu_no,"0043",pro_ver,"02",pack_par,track_ID,sock)

		
#counter = 0
#set camera exception timer and image send timer
ce_tm = time.time()
cam_exc_tm_thresh = 4*60*60
ids_tm = time.time()
ill_send_tm_thresh = 3*60
gps_tm = time.time()


while True:
	initPar()
	
# T ---
#
	print("sending drow instr")

	setSendPack(cpu_no,"0035",pro_ver,"04","2001", track_ID,sock)
#	print("drow branch instr sent, receiving surDet signal......")
	rec_pack = nonblockingRecv(sock)
#	T ---
#
	print("non blocking receiving ------------------------------------")
	print(rec_pack)
	
	if len(rec_pack) == 38:
		surDet = packParse(rec_pack)
	else:
		surDet = ""
#	print('surDet instr received: ',surDet)

#		surveillance branch
	if surDet == "3001":
		sur_tm = time.time()
		print("[INFO]In surveillance mode..............................................")
		print("[INFO]In surveillance mode..............................................")
		print("[INFO]In surveillance mode..............................................")
		print("[INFO]In surveillance mode..............................................")
		print("[INFO]In surveillance mode..............................................")
		while True:
			sendSur(vc,sock)
			rec_pack = nonblockingRecv(sock)
#	T ---
#
			print("non blocking receiving ------------------------------------")
			print(rec_pack)
			
			if len(rec_pack) == 38:
				surDet = packParse(rec_pack)
			else:
				surDet = ""
			if surDet == "3002" or (time.time() - sur_tm) > 60:
				ce_tm = time.time()
				ids_tm = time.time()
				gps_tm = time.time()
				print("[INFO]Return from surveillance mode....................")
				print("[INFO]Return from surveillance mode....................")
				print("[INFO]Return from surveillance mode....................")
				if surDet == "3002":
					with open("log.txt",'w') as f:
						f.write(" received stop signal from server")
						f.close()
				else:
					with open("log.txt",'w') as f:
						f.write("time out ")
						f.close()
				
				break

#		detecting drowssiness
	else:
		ret,frame = vc.read()
		resized_frame = cv2.resize(frame,(0,0),fx=0.25,fy=0.25)
		gray = cv2.cvtColor(resized_frame,cv2.COLOR_BGR2GRAY)
		shape, rect = faceDetAndLandmark(gray,clahe,frame)
		
		
		if len(rect):
			
#			calling warning
			phone_rect = phoneDet(rect,gray,phone_det)
			if len(phone_rect):
				for (x1,y1,w1,h1) in phone_rect:
#					filter some misdetection
					if w1 > 0.8*rect[0][2] or x1>0.52*rect[0][2] and x1 < rect[0][2]*1.52 and y1>0 and y1 < rect[0][3]*0.2:
						phn_cnt = 0
						break
					
#					------------------------
#					------------------------

					x1 = scaler*(x1+int(rect[0][0]-0.52*rect[0][2]))
					y1 = scaler*(y1+rect[0][1])
					w1 = scaler*w1
					h1 = scaler*h1
					cv2.putText(frame,"Calling behavior detected",(0,60),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
					cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (255,0, 0), 2)
					
#					------------------------
#					------------------------
					phn_cnt += 1
				if phn_cnt == phn_alarm_thresh:
					PHO_BEEP = True
					drow_par = drow_par|0x20
					phn_cnt = 0

#			smoking warning
			smk_rect = smkDet(rect,gray,smoke_det)
			if len(smk_rect):
				smk_cnt += 1
				if smk_cnt == smk_alarm_thresh:
					SMK_BEEP = True
					drow_par = drow_par|0x10
					smk_cnt = 0
				
#				----------------------------
#				----------------------------
				print(smk_rect)
				for(x2,y2,w2,h2) in smk_rect:
					x2 = scaler*(x2+rect[0][0])
#					print(rect[0][0])
					y2 = scaler*(y2+int(rect[0][1]+0.6*rect[0][3]))
					w2 = scaler*w2
					h2 = scaler*h2
					cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (255,0, 0), 2)
#				---------------------------
#				---------------------------

			MAR = mouth_aspect_ratio(shape[24:32])
			rightEAR = eye_aspect_ratio(shape[0:6])
			leftEAR = eye_aspect_ratio(shape[6:12])
			EAR = (leftEAR+rightEAR)/2
			
			if EAR < ear_thresh:
				ear_cnt += 1
				if ear_cnt >= ear_consec_frames:
					EYE_BEEP = True
					drow_par = drow_par|0x40
					ear_cnt = 0
			else:
				ear_cnt = 0
					
			if mar > mar_thresh:
				mar_cnt += 1
				if mar_cnt > mar_consec_frames:
					MOU_BEEP = True
					drow_par = drow_par|0x80
					mar_cnt = 0
			else:
				mar_cnt = 0
	

			fac_cnt = 0
			cam_cnt = 0
			ce_tm = time.time()
		else:
			fac_cnt += 1
			cam_cnt += 1
			if fac_cnt == fac_alarm_thresh:
				FAC_BEEP = True
				drow_par = drow_par | 0x08
				fac_cnt = 0

			if cam_cnt > cam_exc_thresh:
				CAM_NBEEP = True
				drow_par = drow_par | 0x01
				cam_cnt = cam_exc_thresh + 1
		
		
#		setting alarm signal on condition
		if CAM_NBEEP:
			DBIT = True
		elif FAC_BEEP or MOU_BEEP or EYE_BEEP or PHO_BEEP or SMK_BEEP and int(getVelocity(ser_intf)) > velocity_thresh:
			drow_par_str = str(hex(drow_par))[2:].zfill(2)
			str_dp = "44590001"+drow_par_str+"000000"
			ser_intf.write(str_dp.encode('utf-8'))
			DBIT = True
		else:
			ids_tm = time.time()
	
	


#send drowsiness detection result to server

		if DBIT:
			if time.time() - ce_tm > cam_exc_tm_thresh and CAM_NBEEP:
				print("sending IDCI instr")
				setSendPack(cpu_no,"0035",pro_ver,"04","2003", track_ID,sock)
				drow_par = drow_par|0x01
	#				check if drowsiness info sent successfully or not. if not suceeded, it probably because can't get info from power board.
	#	T ---
	#  for test comment next line, and add a second line
	#			PB_data = serialRead(ser_intf)
				PB_data = "44590188003042203BC342E85605"
				drow_tm_stmp = str(int(time.time()))
				if len(PB_data) > 0:
					la,lo = convertGPS(PB_data[12:28])
					drow_par_str = str(hex(drow_par))[2:].zfill(2)
					drow_data = drow_par_str+PB_data[8:12]+la+lo+drow_tm_stmp
				else:
					drow_par_str = str(hex(drow_par))[2:].zfill(2)
					drow_data = drow_par_str+"f".zfill(43)+drow_tm_stmp
		# T ---
		#
				print("sending drowsiness alter package")
		
				setSendPack(cpu_no,"0069",pro_ver,"01",drow_data,track_ID,sock)
		# T ---
		#
				print("sending drowsiness image")
		
				sendImg(resized_frame,sock,drow_tm_stmp)
				print("Camera exception img sent")
				ce_tm = time.time()

			elif time.time() - ids_tm > ill_send_tm_thresh:
				print("sending IDNC instr")
				setSendPack(cpu_no,"0035",pro_ver,"04","2002", track_ID,sock)
#				check if drowsiness info sent successfully or not. if not suceeded, it probably because can't get info from power board.
#		T ---
#for test comment next line, and add a second line
#				PB_data = serialRead(ser_intf)
				PB_data = "44590188003042203BC342E85605"
				drow_tm_stmp = str(int(time.time()))
				if len(PB_data) > 0:
					la,lo = convertGPS(PB_data[12:28])
					drow_par_str = str(hex(drow_par))[2:].zfill(2)
					drow_data = drow_par_str+PB_data[8:12]+la+lo+drow_tm_stmp
				else:
					drow_par_str = str(hex(drow_par))[2:].zfill(2)
					drow_data = drow_par_str+"f".zfill(43)+drow_tm_stmp
		# T ---
		#
				print("sending drowsiness alert package")
		
				setSendPack(cpu_no,"0069",pro_ver,"01",drow_data,track_ID,sock)
				sendImg(resized_frame,sock,drow_tm_stmp)
				print("Fatigue img sent")
				ids_tm = time.time()

			else:
				print("sending IDCE instr")
				setSendPack(cpu_no,"0035",pro_ver,"04","2004", track_ID,sock)
				
		else:
			print("legal driving, sending LDXX")
			setSendPack(cpu_no,"0035",pro_ver,"04","2005", track_ID,sock)


#		send GPS info every min

#		T ---
#		for testing, change 60 to 5
	if time.time() - gps_tm > 5:
#		T ---
#for test comment next line, and add a second line
#		PB_data = serialRead(ser_intf)
		PB_data = "44590188003042203BC342E85605"


		tm = str(int(time.time()))
		if len(PB_data) > 0:
			la,lo = convertGPS(PB_data[12:28])
			gps_info = PB_data[8:12]+la+lo+tm
		else:
			gps_info = "f".zfill(43)+tm
#		T ---
#
		print("sending GPS package......... ")


		setSendPack(cpu_no,"0067",pro_ver,"03",gps_info,track_ID,sock)
		gps_tm = time.time()

	cv2.imshow("Frame",frame)
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		break
