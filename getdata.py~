import time
import re
import serial 
#ser = serial.Serial('/dev/ttyACM0',115200)

def collectData():
	with serial.Serial('/dev/ttyACM0',115200) as ser:
		with open('data.txt', 'a') as myfile:
			try:
				while True:
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
					myfile.writelines(s + '\n')
			except KeyboardInterrupt:
				1+1
			except Exception:
				1+1
