#include "IOWarriorIIC.h"

#include <cstring>

using namespace motor_interface;
using namespace std;

IOWarriorIIC::IOWarriorIIC() throw(std::invalid_argument) {
    iowarrior = IowKitOpenDevice();
    // switch on IIC-Mode
    if (iowarrior != NULL) {
        IOWKIT_SPECIAL_REPORT report;
        pid = IowKitGetProductId(iowarrior);
        switch (pid) {
        case IOWKIT_PID_IOW40:
        case IOWKIT_PID_IOW24:
            memset(&report, 0, IOWKIT_SPECIAL_REPORT_SIZE);
            report.ReportID = 0x1;
            report.Bytes[0] = 0x1; // enable IIC
            IowKitWrite(
                iowarrior, IOW_PIPE_SPECIAL_MODE, (char*)&report, IOWKIT_SPECIAL_REPORT_SIZE);
            break;
        default:
            throw std::invalid_argument("IOWarriorIIC: not the right IOWarrior");
        }
    } else {
        throw std::invalid_argument("IOWarriorIIC: no IOWarrior found");
    }
}

IOWarriorIIC::~IOWarriorIIC() throw() {
    // switch off IIC-Mode
    if (iowarrior != NULL) {
        IOWKIT_SPECIAL_REPORT report;
        switch (pid) {
        case IOWKIT_PID_IOW40:
        case IOWKIT_PID_IOW24:
            memset(&report, 0, IOWKIT_SPECIAL_REPORT_SIZE);
            report.ReportID = 0x1;
            report.Bytes[0] = 0x0; // disable IIC
            IowKitWrite(
                iowarrior, IOW_PIPE_SPECIAL_MODE, (char*)&report, IOWKIT_SPECIAL_REPORT_SIZE);
            break;
        }
    }
    IowKitCloseDevice(iowarrior);
}

void IOWarriorIIC::send3Bytes(unsigned char b1, unsigned char b2, unsigned char b3) throw() {
    IOWKIT_SPECIAL_REPORT report;
    memset(&report, 0, sizeof(report));
    report.ReportID = 0x02; // ReportID IIC write request
    report.Bytes[0] = 0xC3; // 3 bytes with IIC Start and Stop (two bytes would be C2)
    report.Bytes[1] = b1;
    report.Bytes[2] = b2;
    report.Bytes[3] = b3;
    IowKitWrite(iowarrior, IOW_PIPE_SPECIAL_MODE, (char*)&report, sizeof(report));
    // swallow ACK report
    IowKitRead(iowarrior, IOW_PIPE_SPECIAL_MODE, (char*)&report, sizeof(report));
}
