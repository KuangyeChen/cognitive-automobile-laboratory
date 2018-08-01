
#ifndef _DerWeg_EPOS_h_
#define _DerWeg_EPOS_h_

#include <string>

namespace motor_interface {

class EPOS {
    int fd; ///< file descriptor
public:
    enum State {
        start = 0,
        notReadyToSwitchOn = 1,
        switchOnDisabled = 2,
        readyToSwitchOn = 3,
        switchedOn = 4,
        refresh = 5,
        measureInit = 6,
        operationEnabled = 7,
        quickStopActive = 8,
        faultReactionActiveDisabled = 9,
        faultReactionActiveEnabled = 10,
        fault = 11,
        error = 12 // error means: could not determine state
    };

    EPOS(const char* device);
    ~EPOS();

    std::string getVersion();              ///< determine EPOS hardware and firmware version
    State getState();                      ///< read the state of the drive
    bool getErrorRegister(unsigned char&); ///< read the error register
    static const std::string& getStateDescription(
        State);                  ///< return a string that describes the state
    bool emergencyStop();        ///< execute stop command, return true on success
    bool setVelocity(long int);  ///< set velocity in rpm, return true on success
    bool getVelocity(long int&); ///< get present velocity in rpm, return true on success
};
}

#endif
