#pragma once

#include <iowkit.h>
#include <stdexcept>

namespace motor_interface {

/** driver class for the IIC-adapter based on the IOWarrior24 */
class IOWarriorIIC {
private:
    unsigned long int pid;
    IOWKIT_HANDLE iowarrior;

public:
    /** create a driver, throws an exception if iowarrior is not connected */
    IOWarriorIIC() throw(std::invalid_argument);
    ~IOWarriorIIC() throw();
    /** send three bytes */
    void send3Bytes(unsigned char b1, unsigned char b2, unsigned char b3) throw();
};
}
