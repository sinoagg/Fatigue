import time
import socket
import pyAesCrypt as pc

def myrecv(conn,count):
	buf=b''
	while count:
		newbuf = conn.recv(count)
		#		print("len of img buf: ",len(newbuf))
		if not newbuf: return None
		buf += newbuf
		count -= len(newbuf)
	return buf
def getCPUSN():
	f = open('/proc/cpuinfo','r')
	for line in f:
		if line[0:6]  == "Serial" :
			cpuserial = line[10:26]
	f.close()
	return cpuserial+"1.1"

def recvEn(sock):
	f = open('.newEncodings.txt','wb')
	rec = myrecv(sock,8)
	rec = rec.decode('utf-8')
	dlen = int(rec)
	enco_data = myrecv(sock,dlen)
	f.write(enco_data)
	f.close()
#	print("encoding.txt stored and socket closed")

def recvO(sock):
	f = open('.update.o','wb')
	rec = myrecv(sock,8).decode('utf-8')
	dlen = int(rec)
	enco_data=myrecv(sock,dlen)
	f.write(enco_data)
	f.close()
#	print("update file stored")

	buffSize = 64*1024
	pw = "sinoagg"
	pc.decryptFile(".update.o",".update.py",pw,buffSize)

def recvFiles(sock):
	buffSize = 64*1024
	pw = "sinoagg"
	file_list = [".lbpSMK",".haarPHN",".haarFace"]
	for ite in file_list:
		defile = ite+".xml"
		fsize = myrecv(sock,8).decode('utf-8')
#		print("fsize: ",fsize)
		if fsize[-2:] != "no":
			f = open(ite,'wb')
			dlen = int(fsize)
			enco_data = myrecv(sock,dlen)
			f.write(enco_data)
			f.close()
			print(ite)
			pc.decryptFile(ite,defile,pw,buffSize)
	sock.close()

time.sleep(15)
#print("sleep for 15 seca")
host = "106.14.160.95"
port = 9501
#Setup and initialize socket
sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
cpuinfo = "00000000c3da3c1c1.1"
sock.connect((host,port))
print("socket connected")
#sock.send(getCPUSN().encode("utf-8"))
sock.send(cpuinfo.encode('utf-8'))
sock.send("ota".encode('utf-8'))
recvO(sock)
recvEn(sock)
print("Files stored")
recvFiles(sock)
