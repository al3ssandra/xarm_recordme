#include <iostream>     // Standard C++ input/output stream library
#include <fcntl.h>      // File control options
#include <termios.h>    // POSIX terminal control library
#include <unistd.h>     // UNIX standard function definitions
#include <cstring>
#include <cstdint>

class SerialWrapper {
    public:
        SerialWrapper(const char * portName, speed_t baudRate);
        ~SerialWrapper();
        int send_command(uint8_t * buf, size_t nbyte);
    
    private:
        const char *portName;
        speed_t baudRate;
        int serialPort;
};

SerialWrapper::SerialWrapper(const char * portName, speed_t baudRate) : 
portName(portName),
baudRate(baudRate)    
{
    // Open the serial port
    serialPort = open(portName, O_RDWR | O_NOCTTY);  // fcntl.h

    if (serialPort == -1) {
        std::cerr << "Error opening serial port" << std::endl;  // iostream
    }

    // Configure serial port
    struct termios tty;
    if (tcgetattr(serialPort, &tty) != 0) {
        std::cerr << "Error getting serial port attributes" << std::endl;  // iostream
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

    cfsetospeed(&tty, baudRate);   // Set baud rate to 9600   // termios.h
    cfsetispeed(&tty, baudRate);   // termios.h

    // Apply configuration
    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial port attributes" << std::endl;  // iostream
    }
}

SerialWrapper::~SerialWrapper() {
    // Close the serial port
    close(serialPort);  // unistd.h
}

int SerialWrapper::send_command(uint8_t * buf, size_t nbyte) {
	int bytesWritten = write(serialPort, buf, nbyte);  // unistd.h
    return bytesWritten;
}
