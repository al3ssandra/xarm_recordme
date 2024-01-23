import serial

serialHandle = serial.Serial('/dev/ttyACM0', 9600)  # Initialize the serial port, baud rate is 9600


def move_servo(id, time, position):
    buf = bytearray(b'\x55\x55')
    buf.extend([(0xff & 8), 0x03, (0xff & 1)])  # length = 8, CMD_SERVO_MOVE = 0x03, number of servos to control = 1

    buf1 = bytearray(b'')
    if time is not None:
        buf1.extend([(0xff & time), (0xff & (time >> 8))])  # Split into low 8 bits and high 8 bits, then put into the buffer
    buf.extend(buf1)  # Append parameters

    buf.append((0xff & id))

    buf2 = bytearray(b'')
    if position is not None:
        buf2.extend([(0xff & position), (0xff & (position >> 8))])  # Split into low 8 bits and high 8 bits, then put into the buffer
    buf.extend(buf2)  # Append parameters

    # buf = bytearray(b'\x55\x55\x08\x03\x01\xE8\x03\x06\x00\x00\n')  # ID = 6 pos = 0 t = 1000ms
    serialHandle.write(buf)  # Send
