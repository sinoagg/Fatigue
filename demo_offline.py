#Detection and surveillance(test version. using specific time to simualte speed)
#This script can detect drowsiness, recognize face and send surveillance video while receiving command from server
#The flow goes as: firstly, it runs recognization function  until registered face detect. It goes to main while loop if driving speed over 2m/s. 
#The main loop has three states: R2R, IM and I2I. In R2R state, it runs drowsiness detection functionality. At the same time, it check the speed continuely.
#If the speed lower 2m/s, it stops drowsiness detection and gos to IM state.
#In IM state, it runs face recognition for 30s. Then check driving speed. If the speed over than 2 m/s, it goes to R2R state. if the speed less than 2m/s, it goes to I2I state.
#IN I2I state, it runs face recognition every minute until the speed over 2m/s. Then it goes to IM state.

#from imutils import face_utils
import numpy as np
#import imutils
import time
import dlib
import cv2
import face_recognition
import socket
from multiprocessing import Process, Value
#import getch
import RPi.GPIO as GPIO
import os
import datetime
import serial
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

def imgSave(frame):
	imgPathName = './img/'+datetime.datetime.now().strftime('%Y-%m-%dT%H.%S.%f')+'.jpg'
	cv2.imwrite(imgPathName,frame)

#Load name and facial encodings from txt file into RAM for recognition
def loadencodings():
	with open('./newEncodings.txt') as f:
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
	return name


def gpioInit(PN):
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(PN,GPIO.OUT)
	GPIO.output(PN,GPIO.LOW)
	print("[INFO] GPIO is initialized")

EAR_THRESH = 0.24
EAR_CONSEC_FRAMES = 6
MAR_THRESH = 0.5
MAR_CONSEC_FRAMES = 10
PIN_NO = 17
ear_counter = 0
mar_counter = 0
ear = 0
mar = 0
frameSizePara = 4
sp = 4
FR2R = True
FI2I = False
FIM = False
EYE_BEEP = False
MOU_BEEP = False
PHO_BEEP = False
FAC_BEEP = False

print("[INFO]Loading resources.........................")
phone_det = cv2.CascadeClassifier("phone15G.xml")
face_pre = dlib.shape_predictor("landmark_predictor.dat")
face_det = cv2.CascadeClassifier("face.xml")
smoke_det = cv2.CascadeClassifier("lbpC.xml")
print("[INFO]Opening camera............................")
#time.sleep(10.0)
vc = cv2.VideoCapture(0)
time.sleep(1.0)
known_names,known_face_encodings = loadencodings()
ser_intf = serial.Serial(port='/dev/ttyAMA0',baudrate=19200,bytesize=8,timeout=1)
drow_par = 0x4459000100000000
#Setup and initialize socket
while 1:
	#add surveillance interface
	name = faceRecg(vc,face_det,known_names,known_face_encodings)
	if name :
		print("{} is driving, initial state and going to drowsiness detection state".format(name))
		break
	else:
		print("Unregister people")


gpioInit(PIN_NO)
print("[INFO]GPIO initialized ")
Ivl = time.time()
while 1:
	t_start = time.time() 
	if time.time()-Ivl < 120:
		drow_par = 0x4459000100000000
		ret,frame = vc.read()
		resized_frame = cv2.resize(frame,(0,0),fx=0.25,fy=0.25)
		gray = cv2.cvtColor(resized_frame,cv2.COLOR_BGR2GRAY)
		gray_orig = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) 
		#detect faces and pose of calling in the grayscale frame using haar classifier
		rects = face_det.detectMultiScale(gray, scaleFactor=1.1, 
				minNeighbors=3, minSize=(30, 30),
				flags=cv2.CASCADE_SCALE_IMAGE)
			#print("Onphone not detected")
		if len(rects):
			FAC_BEEP = False
			#loop over the face detections
			for (x, y, w, h) in rects:
				left = int(x-0.52*w)
				left_smoke = x
				top = y
				top_smoke =int(y+0.7*h) 
				right = int(x+1.52*w)
				right_smoke = x+w
				bottom = int(y+1.4*h)
				bottom_smoke = int(y+1.3*h)
				cv2.rectangle(frame,(left*sp,top*sp),(right*sp,bottom*sp),(0,0,255),2)
				cv2.rectangle(frame,(left_smoke*sp,top_smoke*sp),(right_smoke*sp,bottom_smoke*sp),(0,0,255),2)
				phone_gray = gray[top:bottom,left:right]
				smoke_gray = gray[top_smoke:bottom_smoke,left_smoke:right_smoke]
				phones = phone_det.detectMultiScale(phone_gray, scaleFactor=1.1, 
					minNeighbors=6, minSize=(30, 30),
					flags=cv2.CASCADE_SCALE_IMAGE)
				smoke_gsts = smoke_det.detectMultiScale(smoke_gray,scaleFactor=1.1,minNeighbors=2,
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
						cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (255,0, 0), 2)
						cv2.putText(frame,"Calling behavior detected!",(0,60),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
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
						cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (255,0, 0), 2)
					PHO_BEEP = True
					drow_par = drow_par|0x10000000
				else:
					PHO_BEEP = False

				x = frameSizePara*x
				y = frameSizePara*y
				w = frameSizePara*w
				h = frameSizePara*h
				cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
					# construct a dlib rectangle object from the Haar cascade
					# bounding box
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

					#compute the convex hull for the left and right eye, then
					#visualize each of the eyes and mouth
				leftEyeHull = cv2.convexHull(leftEye)
				rightEyeHull = cv2.convexHull(rightEye)
				mouthHull = cv2.convexHull(mouth)
				for (x,y) in shape[48:68]:
					cv2.circle(frame,(x,y),2,(0,0,255),-1)
				for (x,y) in shape[36:42]:
					cv2.circle(frame,(x,y),2,(0,0,255),-1)
				for(x,y) in shape[42:48]:
					cv2.circle(frame,(x,y),2,(0,0,255),-1)
				cv2.drawContours(frame,[mouthHull],-1,(0,255,0),1)
				cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
				cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
					#draw the computed eye aspect ratio on the frame to help
					#with debugging and setting the correct eye aspect ratio
				cv2.putText(frame,"MAR: {:.3f}".format(mar),(0,20),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
				cv2.putText(frame,"EAR: {:.3f}".format(ear),(0,40),cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255),2)
			print("EAR: {:.3f}".format(mar))
			print("EAR: {:.2f}".format(ear))
			if  ear < EAR_THRESH:
				#print("ear closure detected")
				ear_counter +=  1
				if ear_counter >= EAR_CONSEC_FRAMES:
					EYE_BEEP = True
					drow_par = drow_par|0x40000000
					#print("Eye closed exceed threshold and img of eye closure sent!")
					cv2.putText(frame,"Eye closed for 1 sec",(0,80),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
				else:
					EYE_BEEP = False
					print("Eye closed below threshold")
			else:
				EYE_BEEP = False
				ear_counter = 0
				print("No eye-closure")
			if mar >  MAR_THRESH :
				#print("Yawn detected")
				mar_counter += 1
				if mar_counter >= MAR_CONSEC_FRAMES:
					MOU_BEEP = True
					cv2.putText(frame,"Yawn for 2 secs",(0,100),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
					drow_par = drow_par|0x80000000
					#print("Yawn time exceed threshold and yawn img sent")
				else:
					MOU_BEEP = False
					print("Yawn below threshold")
			else:
				mar_counter = 0
				MOU_BEEP = False
				print("No yawn")
		else:
			FAC_BEEP = True
			drow_par = drow_par|0x08000000
			cv2.putText(frame,"No face detected",(0,20),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
			print("NO face")
		#frame_big = cv2.resize(frame,(0,0),fx=1.5,fy=1.5)
		cv2.imshow("Frame",frame)
		key = cv2.waitKey(1) & 0xFF
		if key == ord("q"):
			break
		if MOU_BEEP or EYE_BEEP or PHO_BEEP or FAC_BEEP :
#			GPIO.output(PIN_NO,GPIO.HIGH)
			str_dp = str(hex(drow_par))
			ser_intf.write(str_dp[2:].encode('utf-8'))
#			print(ser_intf.read(28).decode('utf-8'))
			#str_pwrB = ''
			#while 1:
			#	ser_intf.flushInput()
			#	ser_intf.flushInput()
			#	din=str(ser_intf.read(64))
			#	n = din.find('44590180')
			#	if n!=-1:
			#		str_pwrB = din[n:n+28]
			#		print(str_pwrB)
			#		break
			if MOU_BEEP or EYE_BEEP or PHO_BEEP:
				imgSave(frame)
#		else:
#			GPIO.output(PIN_NO,GPIO.LOW)
	else:
		#recognize for 10s ...
		print("[INFO] Face recognizing ..............................................................")
		GPIO.output(PIN_NO,GPIO.LOW)
		cv2.destroyAllWindows()
		IDimg = np.zeros((550,670,3),np.uint8)
		caption = "No registerd person"
		registeredPerson = False
		recg_ivl = time.time()
		while 1:
			if time.time() - recg_ivl > 30:
				print("Can not identified driver for 10s")
				cv2.destroyAllWindows()
				break
			name = faceRecg(vc,face_det,known_names,known_face_encodings)
			if name:
				print("The drive is {}!".format(name))
				img_name = name+'.jpg'
				IDimg = cv2.imread(img_name)
				caption = "It's " + name
				tt = time.time()
				while  time.time() -tt < 5:
					cv2.putText(IDimg,caption,(0,20),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
					cv2.putText(IDimg,"Driver identified, exit in 5 seconds".format(fps),(0,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
					cv2.imshow("Identifying driver", IDimg)
					key = cv2.waitKey(1) & 0xFF
					if key == ord("q"):
						break
				cv2.destroyAllWindows()
				break
			
			cv2.putText(IDimg,caption,(0,20),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
			cv2.imshow("Identifying driver", IDimg)
			key = cv2.waitKey(1) & 0xFF
		Ivl = time.time()

	fps = 1 / (time.time() - t_start)
	print("FPS: {:.3f}".format(fps))
vc.release()
cv2.destroyAllWindows()
