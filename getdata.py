import time
import re
import serial 
ser = serial.Serial('/dev/ttyACM0',115200)

for i in range(1,100,1):
	s = ''
	#limb.move_to_joint_positions(angles,timeout=1.0)
	#print(angles['left_w2'])
	ser.write(b'm')
	start = time.clock()
	# read sensor strip
	line = ser.readline()
	stop = time.clock()
	elasped = (stop - start)/2
	timestamp = elasped + start
	s = repr(timestamp)
	line = re.sub(' +',',',line)
	line = re.sub('\r\n','',line)
	pre = line.split(",")
	for _i in range(1,17,1):
		s=s+' '+pre[_i]
	print s
