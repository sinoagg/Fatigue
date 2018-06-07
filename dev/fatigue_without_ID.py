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
def myrecv(conn,count):
	buf=b''
	while count:
		newbuf = conn.recv(count)
		#		print("len of img buf: ",len(newbuf))
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

	#Compute the euclidean distance between the horizontal
	#Eye landmark (x,y)-coordinates
	C = euclidean_dist(eye[0], eye[3])

	#Compute the eye aspect ratio
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
	with open('./.newEncodings.txt') as f:
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

#Send surveillance video
def sendSur(videoHandler,conn):
	ret,frame = videoHandler.read()
	result,imgencode = cv2.imencode(".jpg",frame, encode_param)
	data = np.array(imgencode)
	stringData = data.tostring()
	stringData = base64.b64encode(stringData)
	stringData +="==EOF==".encode("utf-8")
	slen = len(stringData)
	conn.send(stringData)
	conn.send(','.encode('utf-8'))
	print("survilliance image sent, len is ", slen)
	cv2.imwrite("img.png",frame)


def sendImg(frame,conn):
	result,imgencode = cv2.imencode(".jpg",frame, encode_param)
	data = np.array(imgencode)
	stringData = data.tostring()
	stringData = base64.b64encode(stringData)
	stringData +="==EOF==".encode("utf-8")
	conn.send(stringData)
	conn.send(','.encode('utf-8'))


# change cpuserial to 16bits
def getCPUSN():
	f = open('/proc/cpuinfo','r')
	print("cpu info open sucessfully")
	for line in f:
		if line[0:6] == 'Serial':
			cpuserial = line[10:26]
	f.close()
	return cpuserial+"1.1"

#def recgResl(ID):
#	resl = "44590002"
#	hID = str(hex(ID))[2:]
#	hID = hID.zfill(6)
#	resl = resl+hID
#	time_str = datetime.datetime.now().strftime('%Y%m%d%H%M%S')[2:]
#	for i in range(6):
#		resl = resl+str(hex(int(time_str[i*2:i*2+2])))[2:].zfill(2)
#	return resl+"".zfill(38)

def serialRead(serial_handle):
	serial_handle.flushInput()
	while 1:
		din = serial_handle.read(28)
		if din[0]==52 and din[1] == 52 and din[2] == 53 and din[3] == 57:
			break
		serial_handle.flushInput()
	dout = din.decode('utf-8')
	return dout


def sendDrowInfo(drow_par,serial_handle,sock):
	din = serialRead(serial_handle)
	dout = drow_par+din+"".zfill(34)
	sock.send(dout.encode('utf-8'))
	return True
	
def getVelocity():
	din = serialRead()
	status_byte = int(din[8:10])
	velocity_source = ""
	if (status_byte & 128) == 128:
		velocity_source = "can"
		velocity = int(din[10:12],16)
	elif (status_byte & 64) == 64:
		velocity_source = "GPS"
		velocity = int(din[10:12],16)
	else:
		velocity = 0
	return velocity


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
phone_det = cv2.CascadeClassifier(".phone15G.xml")
face_pre = dlib.shape_predictor("landmark_pre.dat")
face_det = cv2.CascadeClassifier("face.xml")
smoke_det = cv2.CascadeClassifier(".lbpC.xml")
print("[INFO]Opening camera............................")
vc = cv2.VideoCapture(0)
time.sleep(1.0)


ser_intf = serial.Serial(port='/dev/ttyAMA0',baudrate=19200,bytesize=8,timeout=1)
drow_par = 0x4459000100000000

#setup sockets
sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sockRecv = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
print("New socket connecting")
sock.connect((host,port))
sockRecv.connect((host,port))
sockRecv.setblocking(0)
print("New socket connected")

sock.send((getCPUSN()+',').encode('utf-8'))
sock.send("man,".encode('utf-8'))

#set camera exception timer and image send timer
ce_tm = time.time()
is_tm = time.time()
alarm_tm = time.time()
alarm_cnt = 0
counter = 0
while True:
	t_start = time.time()
	drow_par = 0x4459000100000000

#	sock.send("drow,".encode('utf-8'))
#	print("drow branch instr sent, receiving surDet signal......")
	surDet = myrecv(sock,3).decode("utf-8")
	print('surDet instr received: ',surDet)
	print(surDet)
	if surDet == "sur":
		print("in surveillance mode ")
		while True:
			sendSur(vc,sock)
			try:
				msg = sockRecv.recv(3)
			except socket.error as e:
				err = e.args[0]
				if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
					continue
				else:
					print(e)
					sys.exit(1)
			else:
				surDet = msg.decode('utf-8')
				print("received data: ",msg)
				if surDet == "det":
					ce_tm = time.time()
					is_tm = time.time()
					alarm_tm = time.time()
					break
	else:
		ret,frame = vc.read()
		gray_orig = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		gray = cv2.resize(gray_orig,(0,0),fx=0.25,fy=0.25)
#			gray = cv2.cvtColor(resized_frame,cv2.COLOR_BGR2GRAY)

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
					#print("Onphone detected")
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
				#print("ear closure detected")
				ear_counter +=  1
				if ear_counter >= EAR_CONSEC_FRAMES:
					#sock.send("eycl".encode('utf-8'))
					cv2.putText(frame,"Eye closed for 1 sec",(0,80),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
					#sendImg(frame,sock)
					EYE_BEEP = True
					drow_par = drow_par|0x40000000
					#print("Eye closed exceed threshold and img of eye closure sent!")
				else:
					EYE_BEEP = False
			else:
				ear_counter = 0
				EYE_BEEP = False
				#print("No eye-closure")
			if mar >  MAR_THRESH :
				#print("Yawn detected")
				mar_counter += 1
				if mar_counter >= MAR_CONSEC_FRAMES:
					#sock.send("yawn".encode('utf-8'))
					cv2.putText(frame,"Yawn for 2 secs",(0,100),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
					#sendImg(frame,sock)
					#print("Yawn time exceed threshold and yawn img sent")
					MOU_BEEP = True
					drow_par = drow_par|0x80000000
				else:
					MOU_BEEP = False
					#sock.send("noya".encode('utf-8'))
			else:
				#sock.send("noya".encode('utf-8'))
				mar_counter = 0
				MOU_BEEP = False
				#print("No yawn")
		else:
			FAC_BEEP = True
			drow_par = drow_par|0x08000000
			cv2.putText(frame,"No face detected",(0,20),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)


		if MOU_BEEP or EYE_BEEP or PHO_BEEP or FAC_BEEP :
			resized_frame = cv2.resize(frame,(0,0),fx=0.5,fy=0.5)
			counter += 1
			print("counter is: ",counter)
#				print("Illegal driving detected:(  img captured and sent.")
			DBIT = True
			alarm_cnt += 1
		else:
#				is_tm = time.time()
			print("No illeal driving detected:)")
		
		if time.time() - alarm_tm > 15:
			if alarm_cnt > 3 :
				str_dp = str(hex(drow_par))
				ser_intf.write(str_dp[2:].encode('utf-8'))
			else:
				pass
			alarm_tm = time.time()
			alarm_cnt = 0
		
		else:
			pass

		if time.time()-ce_tm< 25:#190
			print(time.time()-ce_tm)
			print(time.time()-is_tm)
			if time.time()-is_tm >= 10 and DBIT:#180
				#IDNC illegal driving behavior with no camera exception
				sock.send("IDNC,".encode('utf-8'))
				sendDrowInfo(drow_par,ser_intf,sock)
				sendImg(resized_frame,sock)
				is_tm = time.time()
				DBIT = False
				print("RBP: IDNC sent.......................................")
			else:
				# leganl driving behavior
				sock.send("LDXX,".encode('utf-8'))
				print("RBP:LDXX sent")
		elif time.time()-ce_tm >=40:#1440
			#illegal driving behavior sending camera image
			sock.send("IDCI,".encode('utf-8'))
			drow_par = drow_par|0x01000000
			sendDrowInfo(drow_par,ser_intf,sock)
			sendImg(resized_frame,sock)
			is_tm = time.time()
			ce_tm = time.time()
			print("RBP:IDCI sent............................................")
		else:
			#illegal driving behavior with camera exception no sending image
			sock.send("IDCE,".encode('utf-8'))
			print("RBP:IDCE sent")

	fps = 1 / (time.time() - t_start)
	print("FPS: {:.3f}".format(fps))
