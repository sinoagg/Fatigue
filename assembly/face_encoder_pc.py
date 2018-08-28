import face_recognition
import dlib
import cv2
import argparse
import errno



def getRecordID():
	exist_id = ()
	try:
		f = open("encodings.txt",'r')
	except FileNotFoundError as err:
		return exist_id
	else:
		encoding_list = f.readlines()
		num = int(len(encoding_list)/129)
		exist_id = []
		for i in range(num):
			id = encoding_list[i*129].strip('\n')
			exist_id.append(id)
		print(exist_id)
		return exist_id




ap = argparse.ArgumentParser()
ap.add_argument("-i", "--img", required=True,
				help="face img to encode")
args = vars(ap.parse_args())
img_name = args["img"]
id = img_name.split(".")[0]
exist_id = getRecordID()
if id in exist_id:
	print("img already encoded")
else:
	img = face_recognition.load_image_file(img_name)
	encodings = face_recognition.face_encodings(img)[0]
	ef = open("encodings.txt","a+")
	ef.write(id+"\n")
	for item in encodings:
		ef.write("%s\n" % item)
	ef.close()
	print("finish encoding")


