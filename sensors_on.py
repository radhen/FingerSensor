import serial

ser = serial.Serial('/dev/ttyACM0')

while 1:
 ser.write(b'm')
 line = ser.readline()

