import serial
ser = serial.Serial('/dev/ttyACM0', 9600)

while 1:
    serial_line = ser.read(17)
    print(serial_line)
