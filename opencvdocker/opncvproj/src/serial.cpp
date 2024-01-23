#include <iostream>     // Standard C++ input/output stream library
#include <fcntl.h>      // File control options
#include <termios.h>    // POSIX terminal control library
#include <unistd.h>     // UNIX standard function definitions
#include <cstring>
#include <cstdint>

// Instructions for the transmission part
#define FRAME_HEADER            0x55   //帧头 // Frame header
#define CMD_SERVO_MOVE          0x03   //舵机移动指令 // Servo movement instruction

#define GET_LOW_BYTE(A) (uint8_t)((A))
//宏函数 获得A的低八位
// Macro function to obtain the low byte of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//宏函数 获得A的高八位
// Macro function to obtain the high byte of A

int main() {
    const char *portName = "/dev/ttyACM0"; // Change this to your serial port name

    // Open the serial port
    int serialPort = open(portName, O_RDWR | O_NOCTTY);  // fcntl.h

    if (serialPort == -1) {
        std::cerr << "Error opening serial port" << std::endl;  // iostream
        return 1;
    }

    // Configure serial port
    struct termios tty;
    if (tcgetattr(serialPort, &tty) != 0) {
        std::cerr << "Error getting serial port attributes" << std::endl;  // iostream
        return 1;
    }

    tty.c_cflag &= ~PARENB;     // No parity
    tty.c_cflag &= ~CSTOPB;     // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         // 8 data bits
    tty.c_cflag &= ~CRTSCTS;    // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    cfsetospeed(&tty, B9600);   // Set baud rate to 9600   // termios.h
    cfsetispeed(&tty, B9600);   // termios.h

    // Apply configuration
    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial port attributes" << std::endl;  // iostream
        return 1;
    }

    // Write data to the serial port
    // const char *data = "Hello";
    // int bytesWritten = write(serialPort, data, strlen(data));  // unistd.h
    // if (bytesWritten < 0) {
    //     std::cerr << "Error writing to serial port" << std::endl;  // iostream
    //     return 1;
    // }

	// uint8_t buf[11];
    // uint8_t servoID = 3;
    // uint16_t Position = 850;
    // uint16_t Time = 1000;
	// if (servoID > 31 || !(Time > 0)) { //舵机ID不能打于31,可根据对应控制板修改 // Servo ID cannot be greater than 31, modify according to the corresponding control board
	// 	return 1;
	// }
	// buf[0] = FRAME_HEADER;                   //填充帧头 // Fill in the frame header
	// buf[1] = FRAME_HEADER;
	// buf[2] = 8;                              //数据长度=要控制舵机数*3+5，此处=1*3+5 // Data length = number of servos to control * 3 + 5, here = 1 * 3 + 5
	// buf[3] = CMD_SERVO_MOVE;                 //填充舵机移动指令 // Fill in the servo move command
	// buf[4] = 1;                              //要控制的舵机个数 // Number of servos to control
	// buf[5] = GET_LOW_BYTE(Time);             //填充时间的低八位 // Fill in the low byte of time
	// buf[6] = GET_HIGH_BYTE(Time);            //填充时间的高八位 // Fill in the high byte of time
	// buf[7] = servoID;                        //舵机ID // Servo ID
	// buf[8] = GET_LOW_BYTE(Position);         //填充目标位置的低八位 // Fill in the low byte of the target position
	// buf[9] = GET_HIGH_BYTE(Position);        //填充目标位置的高八位 // Fill in the high byte of the target position

	// int bytesWritten = write(serialPort, buf, 10);  // unistd.h
    // if (bytesWritten < 0) {
    //     std::cerr << "Error writing to serial port" << std::endl;  // iostream
    //     return 1;
    // }

    // read servo 6 position
    uint8_t buf2[10];
    buf2[0] = FRAME_HEADER;                   //填充帧头 // Fill in the frame header
	buf2[1] = FRAME_HEADER;
	buf2[2] = 7;                              //数据长度=要控制舵机数*3+5，此处=1*3+5 // Data length = number of servos to read + 3, here = 4 + 3
	buf2[3] = 0x15;                           //填充舵机移动指令 // Fill in the servo read command
    buf2[4] = 4;                              //要控制的舵机个数 // Number of servos to read
    buf2[5] = 3;                              // ID of servo to read
    buf2[6] = 4;                              // ID of servo to read
    buf2[7] = 5;                              // ID of servo to read
    buf2[8] = 6;                              // ID of servo to read

    int bytesWritten = write(serialPort, buf2, 9);  // unistd.h
    if (bytesWritten < 0) {
        std::cerr << "Error writing to serial port" << std::endl;  // iostream
        return 1;
    }
    
    // char read_buf [256];
    // int num_bytes = read(serialPort, &read_buf, sizeof(read_buf));

    // // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    // if (num_bytes < 0) {
    //     std::cerr << "Error reading serial port" << std::endl;  // iostream
    //     return 1;
    // }

    // Close the serial port
    close(serialPort);  // unistd.h

    return 0;
}