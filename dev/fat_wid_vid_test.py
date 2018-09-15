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
import urllib.request
from urllib.request import urlopen
import face_recognition
from collections import deque

pgm_vsn = "0251"
clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(3,3))


ear_counter = 0
mar_counter = 0
ear = 0
mar = 0

phn_cnt = 0
phn_alarm_thresh = 3
smk_cnt = 0
smk_alarm_thresh = 3
fac_cnt = 0
fac_alarm_thresh = 24
cam_cnt = 0
cam_exc_thresh = 45
ear_cnt = 0
ear_thresh = 0.24
ear_consec_frames = 4
mar_cnt = 0
mar_thresh = 0.4
mar_consec_frames = 4
velocity_thresh = 30

drow_par = 0x00
scaler = 4

print("[INFO]Loading resources.........................")
#phone_det = cv2.CascadeClassifier(".haarPHN.xml")
phone_det = cv2.CascadeClassifier(".haarPHN.xml")
face_pre = dlib.shape_predictor("landmark_pre.dat")
face_det = cv2.CascadeClassifier(".haarFace.xml")
#smoke_det = cv2.CascadeClassifier(".lbpSMK.xml")
print("[INFO]Opening camera............................")


host = "106.14.160.95"
port = 9501
PIN_NO = 17


rho = 1
theta = (np.pi/180) * 1
threshold = 15
min_line_length = 20
max_line_gap = 10

#ill_send_tm_thresh = 3*60
ill_send_tm_thresh = 15
#ce_tm_thresh = 4*60
ce_tm_thresh = 20

#cam_exc_tm_thresh = 4*60*60
cam_exc_tm_thresh = 120

RCG_FREQ = 60
GPS_SEND_FREQ = 5

fra_que = deque([],300)
width = 640
height = 480

CE_Flag = False
ID_Flag = False

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY),90]
ser_intf = serial.Serial(port='/dev/ttyAMA0',baudrate=19200,bytesize=8,timeout=1)
track_ID = str(int(time.time()))



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
#	resized_f = cv2.resize(frame,(0,0),fx=0.5,fy=0.5)
	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
#	print("shape of img----------------")
#	print(gray.shape)
	result,imgencode = cv2.imencode(".jpg",gray,encode_param)
	data = np.array(imgencode)
	stringData = data.tostring()
	stringData = base64.b64encode(stringData)
	tailStr = ("stmp"+tm_stmp+"D5KSN").encode('utf-8')
	stringData += tailStr
	conn.send(stringData)


# change cpuserial to 16bits
def getCPUSN():
	f = open('/proc/cpuinfo','r')
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
	din = serialRead(serial_handle)
#	din = "44590188003042203BC342E85605"

	
	
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
		return "NA"

def convertGPS(gps):
	gps_la = gps[0:8]
	gps_lo = gps[8:16]
	list_la = []
	list_lo = []
	for i in range(4):
		list_la.append(int(gps_la[6-i*2:8-i*2],16))
	la = str(struct.unpack('<f', struct.pack('4B', *list_la))[0]).zfill(20)
	for i in range(4):
		list_lo.append(int(gps_lo[6-i*2:8-i*2],16))
	lo = str(struct.unpack('<f', struct.pack('4B', *list_lo))[0]).zfill(20)
	return la,lo


#non blockging receive function
def nonblockingRecv(sock):
	sock.setblocking(0)
	surDet = ""
	try:
		msg = myrecv(sock,33)
	except socket.error as e:
		sock.setblocking(1)
		err = e.args[0]
		if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
			pass
		else:
			print(e)
			sys.exit(1)
	else:
		sock.setblocking(1)
		surDet = msg.decode('utf-8')
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

#		print(msg[5:9])
		return msg[5:9]
	else:
		print("CRC check failed or wrong package")
		return ""



# with CRC
def setSendPack(cpu_no,plen,pgm_vsn,instr,par_list,track_ID,sock):
	pack_head = "DY18"
	pack_tail = "D5KSN"
	msg_before_crc = pack_head+plen+cpu_no+pgm_vsn+instr+par_list+track_ID
	crc_cal = str(hex(binascii.crc_hqx(msg_before_crc.encode(),0x0000)))[2:]
	crc = crc_cal.zfill(4)
	msg = msg_before_crc+crc+pack_tail
	sock.send(msg.encode('utf-8'))
#	print("sent msg: ",msg)


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
		shape = ()
		rimg = rotateImage(gray, angle)
		detected = face_det.detectMultiScale(rimg,scaleFactor = 1.2,minNeighbors= 4,minSize=(24, 24),flags=cv2.CASCADE_SCALE_IMAGE )
		if len(detected):
			max_rect_index = np.argmax(detected[:,2])
			(x,y,w,h) = detected[max_rect_index]
			bottom = int(y + 1.02*h)
			roi = gray[y:bottom,x:x+w]
			gray[y:bottom,x:x+w] = clahe.apply(roi)
			rect = dlib.rectangle(int(x), int(y), int(x + w),int(y + 1.02*h))
			shape = face_pre(rimg,rect)
			
			
			shape = shape_to_np(shape)
			shape = rotateShape(shape,gray,-angle)
			detected = [rotatePoint(detected[max_rect_index], gray, -angle)]
			(x,y,w,h) = detected[0]
			cv2.rectangle(frame, (int(x*scaler), int(y*scaler)), (int(x+w)*scaler, int(y+h)*scaler), (0,255,0), 2)
			break

	return shape, detected



def phoneDet(face_rect, gray,phone_det):
	(x,y,w,h) = face_rect[0]
	left = int(x - 0.52*w)
	top = int(y)
	right = int(x+1.52*w)
	bottom = int(y+1.4*h)
	phone_gray = gray[top:bottom,left:right]
	phones_rect = phone_det.detectMultiScale(phone_gray, scaleFactor=1.2,minNeighbors=3, minSize=(18, 18),flags=cv2.CASCADE_SCALE_IMAGE)
	
	for (x1,y1,w1,h1) in phones_rect:
		if w1 >  0.8*w:
			continue
		if (x1 < 0.52*w*0.75 and y1 < 0.8*h) or ( x1>(0.52*w+0.6*w) and x1 < (1.52*w + 0.33*0.52*w) and y1 < 0.8*h):
			return True

	return False


#Face recognition
def faceRecg(vc,face_det,Known_names, Known_face_encodings):
	ret,frame = vc.read()
	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	#using haar classifier to locate face location
	rects = face_det.detectMultiScale(gray,scaleFactor=1.2,minNeighbors=5,minSize=(3,28),flags=cv2.CASCADE_SCALE_IMAGE)
	face_locations=[]
	for(x,y,w,h) in rects:
		face_locations.append((y,x+w,y+h,x))
	rgb_frame = frame[:,:,::-1]
	print("found {} faces in image".format(len(face_locations)))
	face_encodings = face_recognition.face_encodings(rgb_frame,face_locations)
	name = ""
	for face_encoding in face_encodings:
		matches = face_recognition.compare_faces(Known_face_encodings,face_encoding,tolerance=0.4)
		if True in matches:
			name_index = matches.index(True)
			name = Known_names[name_index]
			print("It's {}!".format(name))
	return name

#Load name and facial encodings from txt file into RAM for recognition
def loadencodings():
	with open('encodings.txt') as f:
		encoding_list = f.readlines()
		num = int(len(encoding_list)/129)
		known_names = []
		known_face_encodings = []
		
		#split name and facial encoding info
		for i in range(num):
			arr = encoding_list[(i*129+1):((i+1)*129)]
			arr = np.array(arr)
			arr = arr.astype(float)
			a = encoding_list[i*129].strip('\n')
			known_names.append(a)
			known_face_encodings.append(arr)
		f.close()
	return known_names, known_face_encodings



def rcrdAndSendFtgVid(fra_que,sock):

	if os.path.exists("fatigue.mp4"):
		os.remove("fatigue.mp4")
	vid_out = cv2.VideoWriter("fatigue.mp4",0x00000021,15.0,(320,240),isColor=0)

	while len(fra_que):
		poped_fra = fra_que.popleft()
		vid_out.write(poped_fra)
	vid_out.release()
	print("-------------video written--------------")
	f = open("fatigue.mp4","rb")
	data = f.read(1024)
	while(data):
		sock.send(data)
		data = f.read(1024)
	print("-------------Video sent-----------------")
	f.close()








# check update. if updated set alarm
def checkUpdate(pgm_vsn,serial_handle):
	try:
		f = open("fv.txt",'r')
	except FileNotFoundError as err:
		#		if not found fv.txt(version file), create fv.txt
		print("fv.txt not found. creating and initializing fv.txt")
		f = open("fv.txt",'w')
		f.write('## This file contains version of update file\n')
		f.write(pgm_vsn)
		f.close()
		ser_intf.write("4459000100020000".encode('utf-8'))
	else:
		lines = f.readlines()
		if int(lines[1]) < int(pgm_vsn):
			#			set alarm
			ser_intf.write("4459000100020000".encode('utf-8'))
			f.close()
			f = open("fv.txt","w")
			f.write('## This file contains version of update file\n')
			f.write(pgm_vsn)
			f.close()


# self check 4g, camera, powerboard connection and gps
def selfCheck(serial_handle):
	while 1 :
		net_failed = False
		cam_failed = False
		gps_failed = False
		pbd_failed = False
		
		sck_cnt = 0
		scd_cnt = 0
		while (sck_cnt <30):
			try:
				urlopen("https://www.baidu.com/")
			except urllib.error.URLError as e:
				sck_cnt += 1
			else:
				sck_cnt += 1
				scd_cnt += 1
		if scd_cnt < 20:
			net_failed = True
			print("4g failed")
		
		for i in range(2):
			vc = cv2.VideoCapture(0)
			time.sleep(1)
			cam_failed = not vc.isOpened()
			vc.release()
		
		pb_data = serialRead(serial_handle)
		print(pb_data)
		if pb_data == "":
			pbd_failed = True
			print("power board failed")
		else:
			status_by = int(pb_data[8],16)
			if  status_by & 1:
				gps_failed = True
				print("print gps failed")


		if net_failed or cam_failed or gps_failed or pbd_failed:
			#			set alarm
			ser_intf.write("4459000100050000".encode('utf-8'))
		else:
			ser_intf.write("4459000100040000".encode('utf-8'))
			break


def hough_transform(canny_img, rho, theta, threshold, min_line_len, max_line_gap):
	return cv2.HoughLinesP(canny_img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)

def isolateWhite(roi):
	#	generate white mask
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(9, 9))
	lower_white = np.array([180,160,160], dtype=np.uint8)
	upper_white = np.array([255,255,255], dtype=np.uint8)
	white_mask = cv2.inRange(roi, lower_white, upper_white)
	eroded = cv2.erode(white_mask,kernel)
	dilated = cv2.dilate(eroded,kernel)
	
	return dilated

def draw_lines(img, lines, color=[255, 0, 0], thickness=4, make_copy=True):
	
	img_copy = np.copy(img) if make_copy else img
	for line in lines:
		for x1,y1,x2,y2 in line:
			cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def smkDet(frame,jaw):
	roi_left = jaw[5][0]
	roi_right = jaw[11][0]
	roi_top = jaw[2][1]
	roi_bottom = jaw[7][1]
	roi = frame[roi_top:roi_bottom,roi_left:roi_right,:]
	morphed_white_mask = isolateWhite(roi)
	hough_lines = hough_transform(morphed_white_mask, rho, theta, threshold, min_line_length, max_line_gap)
	if hough_lines is None:
		flag = False
	else:
		cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (255,0, 0), 2)
		return True




cpu_no = getCPUSN()
checkUpdate(pgm_vsn,ser_intf)
selfCheck(ser_intf)
vc = cv2.VideoCapture(0)
time.sleep(1.0)
#setup sockets
sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
print("New socket connecting")
sock.connect((host,port))
print("New socket connected")
setSendPack(cpu_no,"0035",pgm_vsn,"04","1002", track_ID,sock)
setSendPack(cpu_no,"0035",pgm_vsn,"04","4001", track_ID,sock)
known_names,known_face_encodings = loadencodings()
rcg_tm = time.time()
# comment temporarily
time.sleep(5)


#while 1:
#
#	name = faceRecg(vc,face_det,known_names,known_face_encodings)
#	if name :
#		print("{} is driving, initial state and going to drowsiness detection state".format(name))
#		pack_par = "11"+name+str(int(time.time()))
#		setSendPack(cpu_no,"0043",pgm_vsn,"02",pack_par,track_ID,sock)
#		str_dp = "44590001000000c0"
#		ser_intf.write(str_dp.encode('utf-8'))
#		break
#	else:
#		if time.time() - rcg_tm > 10:
#			pack_par = "10000000"+str(int(time.time()))
#			setSendPack(cpu_no,"0043",pgm_vsn,"02",pack_par,track_ID,sock)
#			str_dp = "4459000100000000"
#			ser_intf.write(str_dp.encode('utf-8'))
#			rcg_tm = time.time()
#		print("Unregister people")



#set camera exception timer and image send timer
ce_tm = time.time()
gps_tm = time.time()
rcg_prd_tm = time.time()
ids_tm = time.time()

while 1:
	EYE_BEEP = False
	MOU_BEEP = False
	PHO_BEEP = False
	FAC_BEEP = False
	SMK_BEEP = False
	CAM_NBEEP = False
	drow_par = 0x00

	setSendPack(cpu_no,"0035",pgm_vsn,"04","2001", track_ID,sock)
	rec_pack = nonblockingRecv(sock)
	
	if len(rec_pack) == 33:
		surDet = packParse(rec_pack)
	else:
		surDet = ""


#		surveillance branch
	if surDet == "3001":
		sur_tm = time.time()
		print("[INFO]In surveillance mode..............................")
		while True:
			sendSur(vc,sock)
			rec_pack = nonblockingRecv(sock)
			
			if len(rec_pack) == 33:
				surDet = packParse(rec_pack)
			else:
				surDet = ""
			if surDet == "3002" or (time.time() - sur_tm) > 180:
				ce_tm = time.time()
				ids_tm = time.time()
				gps_tm = time.time()
				print("[INFO]Return from surveillance mode................")
				break

#		detecting drowssiness
	else:
		ret,frame = vc.read()
		gray_orig = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		gray = cv2.resize(gray_orig,(0,0),fx=0.25,fy=0.25)
		shape, rect = faceDetAndLandmark(gray,clahe,frame)
	
		if len(rect):
			
#			calling warning
			phn_res = phoneDet(rect,gray,phone_det)
			if phn_res:
				phn_cnt += 1
				if phn_cnt == phn_alarm_thresh:
					PHO_BEEP = True
					drow_par = drow_par|0x20
					cv2.putText(frame,"Calling behavior detected",(0,60),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
					phn_cnt = 0
			else:
				phn_cnt = 0


#			smoking warning
			jaw = shape[0:17]*scaler
			smk_result = smkDet(frame,jaw)
			if smk_result:

				smk_cnt += 1
				if smk_cnt == smk_alarm_thresh:
					cv2.putText(frame,"Smoking detected",(0,60),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
					SMK_BEEP = True
					drow_par = drow_par|0x10
					smk_cnt = 0
			else:
				smk_cnt = 0

			leftEye = shape[42:48]*scaler
			rightEye = shape[36:42]*scaler
			mouth = shape[60:68]*scaler
			mar = mouth_aspect_ratio(mouth)
			leftEAR = eye_aspect_ratio(leftEye)
			rightEAR = eye_aspect_ratio(rightEye)
			EAR = (leftEAR + rightEAR) / 2.0
			
			if EAR < ear_thresh:
				ear_cnt += 1
				if ear_cnt >= ear_consec_frames:
					EYE_BEEP = True
					drow_par = drow_par|0x40
					ear_cnt = 0
					cv2.putText(frame,"Eye closed",(0,80),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)

			else:
				ear_cnt = 0
					
			if mar > mar_thresh:
				mar_cnt += 1
				if mar_cnt >=mar_consec_frames:
					MOU_BEEP = True
					drow_par = drow_par|0x80
					mar_cnt = 0
					cv2.putText(frame,"Yawn for 2 secs",(0,100),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
			else:
				mar_cnt = 0
	
#			CAM_NBEEP = False
			fac_cnt = 0
			cam_cnt = 0
			ce_tm = time.time()
			CE_Flag = False
		else:
			fac_cnt += 1
			cam_cnt += 1
			if fac_cnt == fac_alarm_thresh:
				FAC_BEEP = True
				drow_par = drow_par | 0x08
				fac_cnt = 0
				cv2.putText(frame,"No face detected",(0,20),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)

			if cam_cnt > cam_exc_thresh:
				CE_Flag = True
				CAM_NBEEP = True
				drow_par = 0x00
				drow_par = drow_par | 0x01
				cam_cnt = cam_exc_thresh + 1
				cv2.putText(frame,"camera exception",(0,20),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)

		if FAC_BEEP or MOU_BEEP or EYE_BEEP or PHO_BEEP or SMK_BEEP:
			if MOU_BEEP or EYE_BEEP or PHO_BEEP or SMK_BEEP:
				if not ID_Flag:
					drow_par_vid = drow_par
					ftg_img = frame.copy()
					ids_tm = time.time()
				
				ID_Flag = True
			v = getVelocity(ser_intf)
			if v == "":
				print("can not obtain data from power board")
			elif v == "NA" or int(v) > velocity_thresh:
				drow_par_str = str(hex(drow_par))[2:].zfill(2)
				str_dp = "44590001"+drow_par_str+"000000"
				ser_intf.write(str_dp.encode('utf-8'))

		que_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		que_gray = cv2.resize(que_gray,(0,0),fx=0.5,fy=0.5)
		if not ID_Flag and len(fra_que) == 150:
			fra_que.popleft()
		fra_que.append(que_gray)
	

#		ill driving.
		if ID_Flag:
			print("ID_flag is true")
			print("len of video queque: ",len(fra_que))
#			ill driving instr and img will be sent every 3 min
			if time.time() - ids_tm > ill_send_tm_thresh and len(fra_que) == 300:
				print("sending IDNC instr")
				setSendPack(cpu_no,"0035",pgm_vsn,"04","2002", track_ID,sock)
				PB_data = serialRead(ser_intf)
				drow_tm_stmp = str(int(time.time()))
				drow_par_vid = drow_par_vid & 0xfe
				if len(PB_data) :
					la,lo = convertGPS(PB_data[12:28])
					drow_par_str = str(hex(drow_par_vid))[2:].zfill(2)
					drow_data = drow_par_str+"0"+PB_data[9:12]+la+lo+drow_tm_stmp
				else:
					drow_par_str = str(hex(drow_par_vid))[2:].zfill(2)
					drow_data = drow_par_str+"1".zfill(43)+drow_tm_stmp
				
				setSendPack(cpu_no,"0069",pgm_vsn,"01",drow_data,track_ID,sock)
				
				
				sendImg(ftg_img,sock,drow_tm_stmp)
				print("fatigue image sent")
				setSendPack(cpu_no,"0035",pgm_vsn,"04","6001", track_ID,sock)
				rcrdAndSendFtgVid(fra_que,sock)
				vid_tail = "stmp"+drow_tm_stmp+"D5KSN"
				sock.send(vid_tail.encode("utf-8"))
				ids_tm = time.time()
				ID_Flag = False


		elif CE_Flag:
			print("CE_flag is true")
			#			camera exception over 3h, send ce instr and img
			if time.time() - ce_tm > cam_exc_tm_thresh:
				drow_par = 0x00
				print("sending IDCI instr")
				setSendPack(cpu_no,"0035",pgm_vsn,"04","2003", track_ID,sock)
				drow_par = 0x00
				drow_par = drow_par|0x01
				PB_data = serialRead(ser_intf)
				drow_tm_stmp = str(int(time.time()))
				if len(PB_data) > 0:
					la,lo = convertGPS(PB_data[12:28])
					drow_par_str = str(hex(drow_par))[2:].zfill(2)
					drow_data = drow_par_str+"0"+PB_data[9:12]+la+lo+drow_tm_stmp
				else:
					drow_par_str = str(hex(drow_par))[2:].zfill(2)
					drow_data = drow_par_str+"1".zfill(43)+drow_tm_stmp
				
				setSendPack(cpu_no,"0069",pgm_vsn,"01",drow_data,track_ID,sock)
#				print("sending camera exception img")
				sendImg(frame,sock,drow_tm_stmp)
				ce_tm = time.time()
#			camera exception less than 3h, just send ce instr
			else:
				setSendPack(cpu_no,"0035",pgm_vsn,"04","2004", track_ID,sock)
	


#		legal driving. send LD instr
		else:
			setSendPack(cpu_no,"0035",pgm_vsn,"04","2005", track_ID,sock)


#		send GPS info every min
		if time.time() - gps_tm > GPS_SEND_FREQ:
			if ID_Flag and len(fra_que) < 300:
				continue
			
			PB_data = serialRead(ser_intf)
			tm = str(int(time.time()))
			if len(PB_data) > 0:
				la,lo = convertGPS(PB_data[12:28])
				gps_info = "0"+PB_data[9:12]+la+lo+tm
			else:
				gps_info = "1".zfill(43)+tm
			setSendPack(cpu_no,"0067",pgm_vsn,"03",gps_info,track_ID,sock)
			gps_tm = time.time()


		#	recognizing every 10 mins
		if time.time()-rcg_prd_tm > RCG_FREQ:
			if ID_Flag and len(fra_que) < 300:
				continue
			rcg_tm = time.time()
			while 1:
				name = faceRecg(vc,face_det,known_names,known_face_encodings)
				if name :
					print("{} is driving".format(name))
					pack_par = "11"+name+str(int(time.time()))
					setSendPack(cpu_no,"0043",pgm_vsn,"02",pack_par,track_ID,sock)
					rcg_prd_tm = time.time()
					break
				elif time.time() - rcg_tm > 20 :
					pack_par = "10000000"+str(int(time.time()))
					setSendPack(cpu_no,"0043",pgm_vsn,"02",pack_par,track_ID,sock)
					print("recognition time out")
					rcg_prd_tm = time.time()
					break

