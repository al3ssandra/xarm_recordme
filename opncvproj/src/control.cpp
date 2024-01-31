#include <servoCommands.h>


int main() {
    SerialWrapper serialHandle("/dev/ttyACM0", B9600);
    uint8_t Num = 4;
    RobotServo servos[Num];
    servos[0].ID = 3;
    servos[0].Position = 500 + 350;
    servos[1].ID = 4;
    servos[1].Position = 500 + 175;
    servos[2].ID = 5;
    servos[2].Position = 500 + 175;
    servos[3].ID = 6;
    servos[3].Position = 500;
    uint16_t Time = 1000;
    move_servos(serialHandle, servos, Num, Time);
    return 0;
}