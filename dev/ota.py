import time #line:1
import socket #line:2
import pyAesCrypt as pc #line:3
def getCPUSN ():#line:4
	O0OO00O00OO0OOO0O =open ('/proc/cpuinfo','r')#line:5
	for OO00OO00O0O0O0O0O in O0OO00O00OO0OOO0O :#line:6
		if OO00OO00O0O0O0O0O [0 :6 ]=="Serial":#line:7
			O00000O00O00OOOO0 =OO00OO00O0O0O0O0O [10 :26 ]#line:8
	O0OO00O00OO0OOO0O .close ()#line:9
	O0O00000OOOO00O0O =str (int (O00000O00O00OOOO0 ,16 ))#line:10
	if len (O0O00000OOOO00O0O )<=15 :#line:11
		O0O00000OOOO00O0O =O0O00000OOOO00O0O .zfill (15 )#line:12
	else :#line:13
		O0O00000OOOO00O0O =O0O00000OOOO00O0O [-15 :]#line:14
	return O0O00000OOOO00O0O #line:15
def recvEn (O00000OO00O0O0OOO ):#line:17
        OOOOOO0OO0O0OOO0O =open ('.newEncodings.txt','wb')#line:18
        OO0OO0OO00OOOOOOO =O00000OO00O0O0OOO .recv (1024 )#line:19
        while OO0OO0OO00OOOOOOO :#line:20
                OOOOOO0OO0O0OOO0O .write (OO0OO0OO00OOOOOOO )#line:21
                OO0OO0OO00OOOOOOO =O00000OO00O0O0OOO .recv (1024 )#line:22
        OOOOOO0OO0O0OOO0O .close ()#line:24
        print ("encoding.txt stored")#line:25
        O00000OO00O0O0OOO .close ()#line:26
def recvO (O0O0O000O00O0O0O0 ):#line:28
        OO000OO00OOOO000O =open ('.update.o','wb')#line:29
        OOO0000O00OOO0OOO =O0O0O000O00O0O0O0 .recv (1024 )#line:30
        while OOO0000O00OOO0OOO :#line:31
                OO000OO00OOOO000O .write (OOO0000O00OOO0OOO )#line:32
                OOO0000O00OOO0OOO =O0O0O000O00O0O0O0 .recv (1024 )#line:33
        OO000OO00OOOO000O .close ()#line:34
        print ("update file stored")#line:35
        O0O0O000O00O0O0O0 .close ()#line:36
        O00OOOOO0O00O0O00 =64 *1024 #line:37
        O00O0O0OO0O000O00 ="sinoagg"#line:38
        pc .decryptFile (".update.o",".update.py",O00O0O0OO0O000O00 ,O00OOOOO0O00O0O00 )#line:39
        print ("file decrypted")#line:40
time .sleep (15 )#line:42
print ("sleep for 8 seca")#line:43
host ="192.168.8.125"#line:44
port =3018 #line:46
sock =socket .socket (socket .AF_INET ,socket .SOCK_STREAM )#line:48
sock .connect ((host ,port ))#line:50
print ("socket connected")#line:51
sock .send (getCPUSN ().encode ("utf-8"))#line:52
recvEn (sock )#line:63
sock =socket .socket (socket .AF_INET ,socket .SOCK_STREAM )#line:65
sock .connect ((host ,port ))#line:67
print ("New socket connected")#line:68
recvO (sock )
