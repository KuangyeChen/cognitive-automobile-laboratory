#ifndef SERVO_H
#define SERVO_H

#include "IOWarriorIIC.h"

namespace motor_interface {

class Servo {
public:
    inline Servo(){};

    void setSteer(unsigned char b) {
        steer = b > 0 ? b : 1;
        iic.send3Bytes(0xC2, 0x01, steer); // 0xC2=address of SD20, 0x00=output address 0, b=value
    }

    void setSteerOff() {
        steer = 0;
        iic.send3Bytes(0xC2, 0x01, steer); // 0xC2=address of SD20, 0x00=output address 0, b=value
    }

    unsigned char getSteer() const {
        return steer;
    }

private:
    IOWarriorIIC iic;
    unsigned char steer;
};

} // namespace

#endif // SERVO_H
