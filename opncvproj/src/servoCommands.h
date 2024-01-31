#include <serial.h>

// Instructions for the transmission part
#define FRAME_HEADER            0x55   // Frame header
#define CMD_SERVO_MOVE          0x03   // Servo movement instruction
#define CMD_SERVO_READ          0x15   // Servo read instruction

// Macro function to obtain the low byte of A
#define GET_LOW_BYTE(A) (uint8_t)((A))

// Macro function to obtain the high byte of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)


struct RobotServo {  // Structure for servo ID and position
  uint8_t  ID;       // Servo ID
  uint16_t Position; // Servo data
};

void move_servo(SerialWrapper& serialHandle ,uint8_t servoID, uint16_t Position, uint16_t Time) {
	uint8_t buf[11];
	if (servoID > 31 || !(Time > 0)) { // Servo ID cannot be greater than 31, modify according to the corresponding control board
		std::cerr << "Servo ID cannot be greater than 31, modify according to the corresponding control board" << std::endl;  // iostream
        return;
	}
	buf[0] = FRAME_HEADER;                   // Fill in the frame header
	buf[1] = FRAME_HEADER;
	buf[2] = 8;                              // Data length = number of servos to control * 3 + 5, here = 1 * 3 + 5
	buf[3] = CMD_SERVO_MOVE;                 // Fill in the servo move command
	buf[4] = 1;                              // Number of servos to control
	buf[5] = GET_LOW_BYTE(Time);             // Fill in the low byte of time
	buf[6] = GET_HIGH_BYTE(Time);            // Fill in the high byte of time
	buf[7] = servoID;                        // Servo ID
	buf[8] = GET_LOW_BYTE(Position);         // Fill in the low byte of the target position
	buf[9] = GET_HIGH_BYTE(Position);        // Fill in the high byte of the target position

	// int bytesWritten = serialHandle.send_command(buf, sizeof buf);  // unistd.hsizeof
    int bytesWritten = serialHandle.send_command(buf, 10);  // unistd.h
    if (bytesWritten < 0) {
        std::cerr << "Error writing to serial port" << std::endl;  // iostream
    }
}

void read_servos(SerialWrapper& serialHandle, uint8_t numservos, RobotServo servos[]) {
    uint8_t buf[10];
    buf[0] = FRAME_HEADER;                   // Fill in the frame header
	buf[1] = FRAME_HEADER;
	buf[2] = numservos + 3;                  // Data length = number of servos to read + 3, here = 4 + 3
	buf[3] = CMD_SERVO_READ;                 // Fill in the servo read command
    buf[4] = numservos;                      // Number of servos to read
    for (uint8_t i = 0; i < numservos; i++) {
        buf[5 + i] = servos[i].ID;           // ID of servo to read
    }

    int bytesWritten = serialHandle.send_command(buf, sizeof buf);  // unistd.h
    if (bytesWritten < 0) {
        std::cerr << "Error writing to serial port" << std::endl;  // iostream
    }
}

void move_servos(SerialWrapper& serialHandle, RobotServo servos[], uint8_t numservos, uint16_t Time) {
	uint8_t buf[103];    // Establish a buffer
	if (numservos < 1 || numservos > 32 || !(Time > 0)) {
		return; // Number of servos cannot be zero or greater than 32, time cannot be zero
	}
	buf[0] = FRAME_HEADER;    // Fill in the frame header
	buf[1] = FRAME_HEADER;
	buf[2] = numservos * 3 + 5;     // Data length = number of servos to control * 3 + 5
	buf[3] = CMD_SERVO_MOVE;  // Fill in the servo move command
	buf[4] = numservos;             // Number of servos to control
	buf[5] = GET_LOW_BYTE(Time); // Obtain the low byte of time
	buf[6] = GET_HIGH_BYTE(Time); // Obtain the high byte of time
	uint8_t index = 7;
	for (uint8_t i = 0; i < numservos; i++) { // Loop to fill in servo ID and corresponding target position
		buf[index++] = servos[i].ID; // Fill in servo ID
		buf[index++] = GET_LOW_BYTE(servos[i].Position); // Fill in the low byte of the target position
		buf[index++] = GET_HIGH_BYTE(servos[i].Position);// Fill in the high byte of the target position
	}
	int bytesWritten = serialHandle.send_command(buf, buf[2] + 2); // Send the frame, length is data length + two bytes of the frame header
    if (bytesWritten < 0) {
        std::cerr << "Error writing to serial port" << std::endl;  // iostream
    }
}