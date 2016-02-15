import time
import re
import serial 
ser = serial.Serial('/dev/ttyACM0',115200)

try:
	while 1:
		s = ''
		ser.write(b'm')
		line = ser.readline()
		#s = repr(timestamp)
		line = re.sub(' +',',',line)
		line = re.sub('\r\n','',line)
		pre = line.split(",")
		for _i in range(1,17,1):
			s=s+' '+pre[_i]
		print s

except KeyboardInterrupt:
	ser.close()
except Exception:
	ser.close()
