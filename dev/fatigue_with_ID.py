#
import numpy as np
import time
import dlib
import cv2
import face_recognition
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


#Load name and facial encodings from txt file into RAM for recognition
def loadencodings():
	with open('newEncodings.txt') as f:
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


#Face recognition 
def faceRecg(vc,face_det,Known_names, Known_face_encodings):
	ret,frame = vc.read()
	#frame = imutils.resize(frame,width=350)
	frame = cv2.resize(frame,(0,0),fx=0.5,fy=0.5)
	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	#using haar classifier to locate face location 
	rects = face_det.detectMultiScale(gray,scaleFactor=1.1,minNeighbors=5,minSize=(30,30),flags=cv2.CASCADE_SCALE_IMAGE)
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
		else:
			print("Not registered face")
	return name


#Send surveillance video
def sendSur(videoHandler,conn):
	print("timeout value after setting blocking: ",sock.gettimeout())
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

def recgResl(ID):
	resl = "44590002"
	hID = str(hex(ID))[2:]
	hID = hID.zfill(6)
	resl = resl+hID
	time_str = datetime.datetime.now().strftime('%Y%m%d%H%M%S')[2:]
	for i in range(6):
		resl = resl+str(hex(int(time_str[i*2:i*2+2])))[2:].zfill(2)
	return resl+"".zfill(38)

#read data from power board serial port
def serialRead(serial_handle):
	serial_handle.flushInput()
	dout = b''
	cnt = 0
	while 1:
		din = serial_handle.read(28)
		cnt += 1
		if len(din) == 0 and cnt < 5:
			print("cnt is ",cnt)
			continue
		if len(din) > 4 and  din[0]==52 and din[1] == 52 and din[2] == 53 and din[3] == 57:
			break
		elif cnt == 5:
			print("Can not receive data from power board")
			break
		serial_handle.flushInput()
	dout = din.decode('utf-8')
	return dout

#obtain and parse velocity
#if failed to obtain return ""; if succeed return velocity value in string
def getVelocity(serial_handle):
	din = serialRead(serial_handle)
	if din == '':
		return ""
	status_byte = int(din[8:10],16)
	if (status_byte & 128) == 128:
		velocity_source = "CAN"
		print("Speed source: " ,velocity_source)
		velocity = int(din[10:12],16)
		return str(velocity)
	elif (status_byte & 64) == 64:
		velocity_source = "Yingxian"
		print("Speed source: ",velocity_source)
		velocity = int(din[10:12],16)
		return str(velocity)
	elif (status_byte & 32) == 32:
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
			print("did not receive any instruction from server")
		else:
			print(e)
			sys.exit(1)
	else:
		sock.setblocking(1)
		surDet = msg.decode('utf-8')
		print("received data: ",msg)
	return surDet



EAR_THRESH = 0.23
EAR_CONSEC_FRAMES = 6
MAR_THRESH = 0.5
MAR_CONSEC_FRAMES = 15
PIN_NO = 17
ear_counter = 0
mar_counter = 0
ear = 0
mar = 0
frameSizePara = 4

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
phone_det = cv2.CascadeClassifier("phone15G.xml")
face_pre = dlib.shape_predictor("landmark_predictor.dat")
face_det = cv2.CascadeClassifier("face.xml")
smoke_det = cv2.CascadeClassifier("lbpC.xml")
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
known_names,known_face_encodings = loadencodings()
rcg_tm = time.time()
while True:
	#add surveillance interface
#	name = faceRecg(vc,face_det,known_names,known_face_encodings)
	name = "Chuanliang"
	if name :
		print("{} is driving, initial state and going to drowsiness detection state".format(name))
		sock.send("rcrd,".encode('utf-8')) # send record instruction
		tmp = (recgResl(30)+',').encode('utf-8')
		print(tmp)
		sock.send(tmp)
		break
	else:
		if time.time() - rcg_tm > 10:
			sock.send("rezi,,".encode('utf-8')) # tell server the client is conducting initial recognization
			rcg_tm = time.time()
		print("Unregister people")

		
counter = 0
Ivl = time.time()
#set camera exception timer and image send timer
ce_tm = time.time()
is_tm = time.time()
alarm_tm = time.time()
alarm_cnt = 0

while True:
	t_start = time.time()
	
#	drowsiness detection branch
	if time.time()-Ivl < 300:
		drow_par = 0x4459000100000000
		sock.send("drow,".encode('utf-8'))
		print("drow branch instr sent, receiving surDet signal......")
		surDet = nonblockingRecv(sock)
		print('surDet instr received: ',surDet)
		
#		surveillance branch
		if surDet == "sur":
			print("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
			print("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
			print("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
			while True:
				sendSur(vc,sock)
				if nonblockingRecv(sock) == "stp":
					ce_tm = time.time()
					is_tm = time.time()
					alarm_tm = time.time()
					break
	
#		detecting drowssiness
		else:
			ret,frame = vc.read()
			gray_orig = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
			gray = cv2.resize(gray_orig,(0,0),fx=0.25,fy=0.25)
			rects = face_det.detectMultiScale(gray, scaleFactor=1.1, 
					minNeighbors=3, minSize=(30, 30),
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
						minNeighbors=6, minSize=(30, 30),
						flags=cv2.CASCADE_SCALE_IMAGE)
					smoke_gsts = smoke_det.detectMultiScale(smoke_gray,scaleFactor=1.1,minNeighbors=1,
										minSize=(30,30),flags=cv2.CASCADE_SCALE_IMAGE)
										
					if len(phones):
						for (x1,y1,w1,h1) in phones:
							if w1 > 0.8*w:
								continue
							x1 = frameSizePara*(x1+left)
							y1 = frameSizePara*(y1+top)
							w1 = frameSizePara*w1
							h1 = frameSizePara*h1
						cv2.putText(frame,"Calling behavior detected",(0,60),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
						PHO_BEEP = True
						drow_par = drow_par|0x20000000
					else:
						PHO_BEEP = False
					if len(smoke_gsts):
						cv2.putText(frame,"Smoking behavior detected!",(0,120),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
						for (x2,y2,w2,h2) in smoke_gsts:
							x2 = frameSizePara*(x2+left_smoke)
							y2 = frameSizePara*(y2+top_smoke)
							w2 = frameSizePara*w2
							h2 = frameSizePara*h2
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
						cv2.putText(frame,"Eye closed for 1 sec",(0,80),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
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
						cv2.putText(frame,"Yawn for 2 secs",(0,100),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
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
				cv2.putText(frame,"No face detected",(0,20),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)


			if MOU_BEEP or EYE_BEEP or PHO_BEEP or FAC_BEEP :
				resized_frame = cv2.resize(frame,(0,0),fx=0.5,fy=0.5)
				counter += 1
				print("counter is: ",counter)
				DBIT = True
				alarm_cnt += 1
			else:
				print("No illeal driving detected:)")

#           set alarm on conditions
			if time.time() - alarm_tm > 15:
				print("setting alarm ")
				velocity = getVelocity(ser_intf)
				if velocity == "":
					pass
				else:
					v = int(velocity)
					if alarm_cnt > 3 and velocity > 30:
						str_dp = str(hex(drow_par))
						ser_intf.write(str_dp[2:].encode('utf-8'))
				alarm_tm = time.time()
				alarm_cnt = 0

			
#           send detecting results to server
			if time.time()-ce_tm< 200:#190
				print(time.time()-ce_tm)
				print(time.time()-is_tm)
				if time.time()-is_tm >= 180 and DBIT:#180ss
					#IDNC illegal driving behavior with no camera exception
					sock.send("IDNC,".encode('utf-8'))
#					comment next line for now
#					sendDrowInfo(drow_par,ser_intf,sock)
					sendImg(resized_frame,sock)
					is_tm = time.time()
					DBIT = False
					print("RBP: IDNC sent.......................................")
				else:
					# leganl driving behavior
					sock.send("LDXX,".encode('utf-8'))
					print("RBP:LDXX sent")
			elif time.time()-ce_tm >=700:#1440
				#illegal driving behavior sending camera image
				sock.send("IDCI,".encode('utf-8'))
				drow_par = drow_par|0x01000000
				
#				commont next line for now ,
#				sendDrowInfo(drow_par,ser_intf,sock)
				sendImg(resized_frame,sock)
				is_tm = time.time()
				ce_tm = time.time()
				print("RBP:IDCI sent............................................")
			else:
				#illegal driving behavior with camera exception no sending image
				sock.send("IDCE,".encode('utf-8'))
				print("RBP:IDCE sent")

#	face recognition branch
	else:
		#recognize for 10s ...
		print("[INFO] Face recognizing ..............................................................")
		sock.send("recg,".encode("utf-8")) 
		print("recg branch instr sent,receiving surRcg instr")
		recg_ivl = time.time()
		while True:
			surDet = nonblockingRecv(sock)
			print('surDet instr received: ',surDet)
			if surDet == "sur":
				while True:
					sendSur(vc,sock)
					surDet = nonblockingRecv(sock)
					if surDet == 'stp':
						recg_ivl = time.time()
						break
			else:
				if time.time() - recg_ivl > 10:
					sock.send("norg,".encode('utf-8')) # send didn't recognize signal
					break
				name = faceRecg(vc,face_det,known_names,known_face_encodings)
				if name:
					sock.send("rcrd,".encode('utf-8')) # send record instruction to server
					driver_info =recgResl(30)+','
					sock.send(driver_info.encode('utf-8'))
					break
		Ivl = time.time()


	fps = 1 / (time.time() - t_start)
	print("FPS: {:.3f}".format(fps))
