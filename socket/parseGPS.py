import struct
#20 bytes string to presenting latitude or longitude
def convertGPS(value):
	plist = []
	for i in range(4):
		plist.append(int(value[6-i*2:8-i*2],16))
	result = str(struct.unpack('<f', struct.pack('4B', *plist))[0]).zfill(20)
	print("function ",latitude)
	return result

ll = "42203BC342E85605"
hexLo = ll[0:8]
hexLa = ll[8:16]
convertGPS(hexLo)
convertGPS(hexLa)
#x = [66,32,59,195]
x = [192,59,32,66]
latitude = struct.unpack('<f', struct.pack('4B', *x))[0]
#y = [66,232,86,5]
y = [5,86,232,66]
longitude = struct.unpack('<f', struct.pack('4B', *y))[0]
print("len of latitude: ",len(str(latitude)))
print(latitude)

print("len of latitude: ",len(str(longitude)))
print(longitude)
