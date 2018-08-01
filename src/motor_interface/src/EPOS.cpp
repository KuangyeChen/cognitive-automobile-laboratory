#include "EPOS.h"

#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

using namespace motor_interface;
using namespace std;

#define eout(x) // std::cerr << x

namespace {
typedef unsigned short WORD; // 16 bit
typedef unsigned char BYTE;  // 8-bit

std::string opmodestring[] = {std::string("start"),
                              std::string("notReadyToSwitchOn"),
                              std::string("switchOnDisabled"),
                              std::string("readyToSwitchOn"),
                              std::string("switchedOn"),
                              std::string("refresh"),
                              std::string("measureInit"),
                              std::string("operationEnable"),
                              std::string("quickStopActive"),
                              std::string("faultReactionActiveDisabled"),
                              std::string("faultReactionActiveEnabled"),
                              std::string("fault"),
                              std::string("error")};
float baudratevals[] = {9.6e3, 14.4e3, 19.2e3, 38.4e3, 57.6e3, 115.2e3};

/** encode buf as hex number, for debugging */
// char hex[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
//  std::string toHex (const WORD* buf, int n) {
//    std::string res;
//    res.resize (5*n);
//    for (int j=0; j<n; ++j) {
//      res[5*j]=hex[(buf[j]&0xF000)>>12];
//      res[5*j+1]=hex[(buf[j]&0x0F00)>>8];
//      res[5*j+2]=hex[(buf[j]&0x00F0)>>4];
//      res[5*j+3]=hex[(buf[j]&0x000F)];
//      res[5*j+4]=' ';
//    }
//    return res;
//  }

const BYTE ACK = 0x4f;  ///< Acknowldegment byte = 'O'
const BYTE FAIL = 0x47; ///< Fail byte = 'F'
const BYTE ANSW = 0x00; ///< Answer byte
WORD buffer[260];       ///< a large buffer to read data from fd

/** calculate the checksum of buffer frame with length framelen (in WORDS) */
WORD calcCRC(WORD* frame, unsigned int framelen) {
    // algorithm from Communication Guide, p. 8
    WORD shifter, c;
    WORD carry;
    WORD CRC = 0;
    while (framelen--) {
        shifter = 0x8000; // Initialize BitX to Bit15
        c = *frame++;     // Copy next DataWord to c
        do {
            carry = CRC & 0x8000; // Check if Bit15 of CRC is set
            CRC <<= 1;            // CRC = CRC * 2
            if (c & shifter)
                CRC++; // CRC = CRC + 1, if BitX is set in c
            if (carry)
                CRC ^= 0x1021; // CRC = CRC XOR G(x), if carry is true
            shifter >>= 1;     // Set BitX to next lower Bit, shifter = shifter/2
        } while (shifter);
    }
    return CRC;
}

/** write bufsz byte from buf to file fd and return true on success */
bool writeBytes(int fd, const void* buf, int bufsz) {
    return (write(fd, buf, bufsz) > 0);
}

/** read at most bufsz bytes from file fd into buf and return number of
  bytes read (in case of success) or a negative number.
  buf must be allocated before calling readBytes! */
int readBytes(int fd, void* buf, int bufsz) {
    int br = 0;
    for (int i = 0; i < 5 && br < bufsz; ++i) {
        int answ = read(fd, reinterpret_cast<BYTE*>(buf) + br, bufsz - br);
        if (answ > 0) {
            br += answ;
        }
        if (br < bufsz)
            usleep(1000);
    }
    return br;
}

/** send the command frame to file fd and return error code */
bool sendCommand(int fd, WORD* frame) {
    int answ;
    BYTE abyte;
    unsigned int framelen = (frame[0] & 0x00FF) + 3;
    frame[framelen - 1] = calcCRC(frame, framelen);
    eout(">> " << toHex(frame, framelen) << '\n');
    BYTE opcode = (frame[0] & 0xFF00) >> 8;
    BYTE len = frame[0] & 0x00FF;

    if (!writeBytes(fd, &opcode, 1))
        return false; // send opcode
    if ((answ = readBytes(fd, &abyte, 1)) != 1 || abyte != ACK)
        return false; // lese ACK
    if (!writeBytes(fd, &len, 1))
        return false; // send length;
    if (!writeBytes(fd, frame + 1, 2 * (framelen - 1)))
        return false; // send data and crc
    if ((answ = readBytes(fd, &abyte, 1)) != 1 || abyte != ACK)
        return false; // lese ACK
    return true;
}

/** receive an answer frame from fd and return
    a) the length of the frame (in WORDS) and the frame, if communication was successful
    b) a negative number, otherwise
    the buffer frame is owned by this function a must not be deleted outside! */
int receiveAnswer(int fd, WORD*& frame) {
    frame = buffer;
    int answ;
    BYTE abyte;
    if ((answ = readBytes(fd, &abyte, 1)) != 1)
        return -1; // read opcode
    if (abyte != ANSW)
        return -2; // not an answer code
    frame[0] = static_cast<WORD>(abyte) << 8;
    if (!writeBytes(fd, &ACK, 1))
        return -3; // send ACK
    if ((answ = readBytes(fd, &abyte, 1)) != 1)
        return -4; // read len-1
    frame[0] |= (0x00FF & static_cast<WORD>(abyte));
    int framelen = static_cast<unsigned int>(abyte) + 3;
    if ((answ = readBytes(fd, frame + 1, 2 * (framelen - 1))) != 2 * (framelen - 1))
        return -5; // read data and crc
    WORD crc = frame[framelen - 1];
    eout("<< " << toHex(frame, framelen) << '\n');
    frame[framelen - 1] = 0x0000;
    if (calcCRC(frame, framelen) == crc) {
        // crc stimmt
        writeBytes(fd, &ACK, 1);
        return framelen;
    } else {
        writeBytes(fd, &FAIL, 1);
        return -6;
    }
}

/** write object to file fd */
bool writeObject(int fd, WORD index, BYTE subindex, WORD dataLow, WORD dataHigh = 0x00) {
    WORD frame[6];
    frame[0] = 0x1103; // write object, len-1==3
    frame[1] = index;
    frame[2] = (0x0000 | subindex);
    frame[3] = dataLow;
    frame[4] = dataHigh;
    frame[5] = 0x00; // reserved for checksum
    if (!sendCommand(fd, frame))
        return false;

    WORD* answer;
    return receiveAnswer(fd, answer) >= 0;
}

/** read object from file fd. Return
  a) size of data buffer (in WORDS) in case of success
  b) a negative number, otherwise
  The buffer is returned in data. Don't delete data! */
int readObject(int fd, WORD index, BYTE subindex, WORD*& data) {
    WORD comframe[4];
    comframe[0] = 0x1001; // command and len-1
    comframe[1] = index;
    comframe[2] = (0x0000 | subindex);
    comframe[3] = 0x0000; // reserved for crc
    if (!sendCommand(fd, comframe))
        return -1;

    WORD* receiveFrame;
    int answ = receiveAnswer(fd, receiveFrame);
    data = receiveFrame + 3;
    return answ - 2;
}
}

EPOS::EPOS(const char* device) : fd(0) {
    // try max. 5 times to open file descriptor
    for (unsigned int trials = 0; trials < 5; ++trials) {
        if ((fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY)) >= 0)
            break;
        sleep(1);
    }

    if (fd < 0)
        throw std::invalid_argument(std::string("EPOS::EPOS: unable to open device ") + device);

    // configure serial connection:
    struct termios options;
    if (tcgetattr(fd, &options) < 0)
        throw std::invalid_argument(
            std::string("EPOS::EPOS: could not obtain termios for device ") + device);
    memset(&options, 0, sizeof(options));
    // EPOS transfer format is: 1 start bit, 8 data bits, no parity, 1 stop bit
    // see: EPOS Communication Guide p. 5
    options.c_cflag |= B115200; // B38400;
    options.c_cflag |= CS8;     // 8 bits per byte
    options.c_cflag |= CLOCAL | CREAD;
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) < 0)
        throw std::invalid_argument(std::string("EPOS::EPOS: could not set termios for device ") +
                                    device);
    if (fcntl(fd, F_SETFL, FNDELAY) < 0) // FNDELAY enyspricht grob O_NONBLOCK
        throw std::invalid_argument(
            std::string("EPOS::EPOS: could not set nonblocking mode for device ") + device);

    // at this point the serial connection is configured
    // initialize EPOS:
    State s2, s1;

    s2 = getState();
    //std::cout << "initial mode: " << getStateDescription(s2) << std::endl;
    do {
      s1 = s2;
      switch (s1) { // breaks are NOT missing!
        case error:
          writeObject(fd, 0x6040, 0x00, 0x00); // disable voltage
          usleep(1000000);
        case fault:
          writeObject(fd, 0x6040, 0x00, 0x80); // fault reset
          usleep(1000000);
        default:
          writeObject(fd, 0x6040, 0x00, 0x06); // shutdown
          usleep(1000000);
          writeObject(fd, 0x6040, 0x00, 0x07); // switch on
          usleep(1000000);
        case quickStopActive:
        case switchedOn:
          writeObject(fd, 0x6040, 0x00, 0x0F); // enable operation
          usleep(100000);
          break;
        case start:
          usleep(1000000);
          break;
        case operationEnabled:
          break;
      }
      s2 = getState();
      //std::cout << "mode: " << getStateDescription(s2) << std::endl;
    } while (s2 != operationEnabled && s1 != s2);
    if (s2 != operationEnabled)
        throw std::invalid_argument(
                "EPOS::EPOS: could not switch EPOS to operationEnabled mode. Final mode was: " + getStateDescription(s2) + " Mode before was: " + getStateDescription(s1));

    usleep(100000);
    // set velocity mode:
    if (!writeObject(fd, 0x6060, 0x00, 0xFE))  
        throw std::invalid_argument(
            std::string("EPOS::EPOS: could not switch EPOS to velocity mode"));
}

EPOS::~EPOS() {
    emergencyStop();
    sleep(1);
    writeObject(fd, 0x6040, 0x00, 0x00); // disable voltage
    close(fd);
}

EPOS::State EPOS::getState() {
    State st = error;
    WORD* data;
    int datalen = readObject(fd, 0x6041, 0x00, data);
    if (datalen > 0) {
        WORD w = data[0] & 0x417F;
        switch (w) {
        case 0x0000:
            st = start;
            break;
        case 0x0100:
            st = notReadyToSwitchOn;
            break;
        case 0x0140:
            st = switchOnDisabled;
            break;
        case 0x0121:
            st = readyToSwitchOn;
            break;
        case 0x0123:
            st = switchedOn;
            break;
        case 0x4123:
            st = refresh;
            break;
        case 0x4133:
            st = measureInit;
            break;
        case 0x0137:
            st = operationEnabled;
            break;
        case 0x0117:
            st = quickStopActive;
            break;
        case 0x010F:
            st = faultReactionActiveDisabled;
            break;
        case 0x011F:
            st = faultReactionActiveEnabled;
            break;
        case 0x0108:
            st = fault;
            break;
        default:
            eout("unknown status word 0x" << toHex(&w, 1) << '\n');
            break;
        }
    }
    eout("EPOS state is: " << getStateDescription(st) << '\n');
    return st;
}

const std::string& EPOS::getStateDescription(EPOS::State s) {
    return opmodestring[s];
}

bool EPOS::emergencyStop() {
    return writeObject(fd, 0x6040, 0x00, 0x02);
}

bool EPOS::setVelocity(long int rpm) {
    return writeObject(fd, 0x206B, 0x00, rpm & 0x0000FFFF, rpm >= 0 ? 0x00000000 : 0xFFFFFFFF);
}

bool EPOS::getVelocity(long int& rpm) {
    WORD* data;
    if (readObject(fd, 0x206B, 0x00, data) >= 2) {
        if (sizeof(long int) == 4) {
            rpm = data[0];
            if (data[1]) {
                rpm = (static_cast<unsigned long int>(data[1]) << 16) | data[0];
            }
            return true;
        } else { // sizeof(long int)=8 (64bit-System)
            rpm = data[0];
            if (data[1]) {
                rpm = 0xFFFFFFFF00000000ul | (static_cast<unsigned long int>(data[1]) << 16) |
                      data[0];
            }
            return true;
        }
    }
    return false;
}

bool EPOS::getErrorRegister(unsigned char& err) {
    WORD* data;
    if (readObject(fd, 0x1001, 0x00, data) >= 1) {
        err = data[0];
        return true;
    }
    return false;
}

std::string EPOS::getVersion() {
    std::stringstream sout;
    WORD* data;
    if (readObject(fd, 0x1008, 0x00, data) > 0)
        sout << "Device name = " << static_cast<char>(data[0] & 0x00FF)
             << static_cast<char>((data[0] & 0xFF00) >> 8) << static_cast<char>(data[1] & 0x00FF)
             << static_cast<char>((data[1] & 0xFF00) >> 8) << '\n';
    if (readObject(fd, 0x2003, 0x01, data) > 0)
        sout << "Software version = " << data[0] << '\n';
    if (readObject(fd, 0x2005, 0x00, data) > 0)
        sout << "RS232 timeout = " << data[0] << " msec\n";
    sout << "EPOS state = " << getStateDescription(getState()) << '\n';
    if (readObject(fd, 0x2002, 0x00, data) > 0)
        sout << "Baudrate = " << baudratevals[data[0]] << std::flush;
    return sout.str();
}


/*
#include "Timestamp.h"

unsigned long rpm = 0;
char c='a';

int main(int argc, char** argv)
{
  try {
    EPOS epos ("/dev/ttyUSB0");
    cerr << epos.getVersion() << endl;
    while (c!='q') {
      cout << ">";
      cin >> c;
      switch (c) {
        case 'q' : break;
        case '+' : rpm+=300; break;
        case '-' : rpm-=300; break;
        case '#' : rpm=0; break;
        default : break;
      }
      Timestamp t;
      epos.setVelocity (rpm);
      cerr << "setVelocity: " << t.elapsed_msec() << " msec\n";
      t.update();
    }
  } catch (std::exception& e) {
    cerr << e.what() << std::endl;
    return -1;
  }
  return 0;
}
*/
