//
// IO-Warrior kit library V1.5 include file
//

// IOW Library definitions
#ifndef _IOW_KIT_H_
#define _IOW_KIT_H_

#ifndef _IOWKIT_BUILD_RUN
#ifdef _IOWKIT2_H_
#error "including both iowkit2.h and iowkit.h is not allowed"
#endif // _IOWKIT2_H_
#endif // _IOWKIT_BUILD_RUN

#ifdef _WIN32

#define IOWKIT_API __stdcall

#else

#define IOWKIT_API

// the following legacy types can only be defined once
#ifndef _IOW_WINTYPES_H_
#define _IOW_WINTYPES_H_

/*
 * Windows specific types and defines
 */
typedef unsigned long          ULONG;
typedef long                   LONG;
typedef unsigned short         USHORT;
typedef unsigned short         WORD;
typedef unsigned char          UCHAR;
typedef unsigned char          BYTE;
typedef char *                 PCHAR;
typedef unsigned short *       PWCHAR;
typedef int                    BOOL;
typedef unsigned char          BOOLEAN;
typedef unsigned long          DWORD;
typedef DWORD *                PDWORD;
typedef void *                 PVOID;
typedef DWORD                  HANDLE;
typedef ULONG *                PULONG;
typedef const char *           PCSTR;
typedef const unsigned short * PWCSTR;

#define FALSE 0
#define TRUE 1

#endif // _IOW_WINTYPES_H_

#endif // _WIN32

// IO-Warrior vendor & product IDs
#define IOWKIT_VENDOR_ID         0x07c0
#define IOWKIT_VID               IOWKIT_VENDOR_ID
// IO-Warrior 40
#define IOWKIT_PRODUCT_ID_IOW40  0x1500
#define IOWKIT_PID_IOW40         IOWKIT_PRODUCT_ID_IOW40
// IO-Warrior 24
#define IOWKIT_PRODUCT_ID_IOW24  0x1501
#define IOWKIT_PID_IOW24         IOWKIT_PRODUCT_ID_IOW24
// IO-Warrior PowerVampire
#define IOWKIT_PRODUCT_ID_IOWPV1 0x1511
#define IOWKIT_PID_IOWPV1        IOWKIT_PRODUCT_ID_IOWPV1
#define IOWKIT_PRODUCT_ID_IOWPV2 0x1512
#define IOWKIT_PID_IOWPV2        IOWKIT_PRODUCT_ID_IOWPV2
// IO-Warrior 56
#define IOWKIT_PRODUCT_ID_IOW56  0x1503
#define IOWKIT_PID_IOW56         IOWKIT_PRODUCT_ID_IOW56

// Max number of pipes per IOW device
#define IOWKIT_MAX_PIPES    2

// pipe names
#define IOW_PIPE_IO_PINS      0
#define IOW_PIPE_SPECIAL_MODE 1

// Max number of IOW devices in system
#define IOWKIT_MAX_DEVICES  16
// IOW Legacy devices open modes
#define IOW_OPEN_SIMPLE     1
#define IOW_OPEN_COMPLEX    2

// first IO-Warrior revision with serial numbers
#define IOW_NON_LEGACY_REVISION 0x1010

// Don't forget to pack it!
#pragma pack(push, 1)

typedef struct _IOWKIT_REPORT
 {
  UCHAR ReportID;
  union
   {
    DWORD Value;
    BYTE Bytes[4];
   };
 }
  IOWKIT_REPORT, *PIOWKIT_REPORT;

typedef struct _IOWKIT40_IO_REPORT
 {
  UCHAR ReportID;
  union
   {
    DWORD Value;
    BYTE Bytes[4];
   };
 }
  IOWKIT40_IO_REPORT, *PIOWKIT40_IO_REPORT;

typedef struct _IOWKIT24_IO_REPORT
 {
  UCHAR ReportID;
  union
   {
    WORD Value;
    BYTE Bytes[2];
   };
 }
  IOWKIT24_IO_REPORT, *PIOWKIT24_IO_REPORT;

typedef struct _IOWKIT_SPECIAL_REPORT
 {
  UCHAR ReportID;
  UCHAR Bytes[7];
 }
  IOWKIT_SPECIAL_REPORT, *PIOWKIT_SPECIAL_REPORT;

typedef struct _IOWKIT56_IO_REPORT
 {
  UCHAR ReportID;
  UCHAR Bytes[7];
 }
  IOWKIT56_IO_REPORT, *PIOWKIT56_IO_REPORT;

typedef struct _IOWKIT56_SPECIAL_REPORT
 {
  UCHAR ReportID;
  UCHAR Bytes[63];
 }
  IOWKIT56_SPECIAL_REPORT, *PIOWKIT56_SPECIAL_REPORT;

#define IOWKIT_REPORT_SIZE sizeof(IOWKIT_REPORT)
#define IOWKIT40_IO_REPORT_SIZE sizeof(IOWKIT40_IO_REPORT)
#define IOWKIT24_IO_REPORT_SIZE sizeof(IOWKIT24_IO_REPORT)
#define IOWKIT_SPECIAL_REPORT_SIZE sizeof(IOWKIT_SPECIAL_REPORT)
#define IOWKIT56_IO_REPORT_SIZE sizeof(IOWKIT56_IO_REPORT)
#define IOWKIT56_SPECIAL_REPORT_SIZE sizeof(IOWKIT56_SPECIAL_REPORT)

#pragma pack(pop)

// Opaque IO-Warrior handle
typedef PVOID IOWKIT_HANDLE;

// Function prototypes

#ifdef  __cplusplus
extern "C" {
#endif // __cplusplus

IOWKIT_HANDLE IOWKIT_API IowKitOpenDevice(void);
void IOWKIT_API IowKitCloseDevice(IOWKIT_HANDLE devHandle);
ULONG IOWKIT_API IowKitWrite(IOWKIT_HANDLE devHandle, ULONG numPipe,
  PCHAR buffer, ULONG length);
ULONG IOWKIT_API IowKitRead(IOWKIT_HANDLE devHandle, ULONG numPipe,
  PCHAR buffer, ULONG length);
ULONG IOWKIT_API IowKitReadNonBlocking(IOWKIT_HANDLE devHandle, ULONG numPipe,
  PCHAR buffer, ULONG length);
BOOL IOWKIT_API IowKitReadImmediate(IOWKIT_HANDLE devHandle, PDWORD value);
ULONG IOWKIT_API IowKitGetNumDevs(void);
IOWKIT_HANDLE IOWKIT_API IowKitGetDeviceHandle(ULONG numDevice);
BOOL IOWKIT_API IowKitSetLegacyOpenMode(ULONG legacyOpenMode);
ULONG IOWKIT_API IowKitGetProductId(IOWKIT_HANDLE devHandle);
ULONG IOWKIT_API IowKitGetRevision(IOWKIT_HANDLE devHandle);
HANDLE IOWKIT_API IowKitGetThreadHandle(IOWKIT_HANDLE devHandle);
BOOL IOWKIT_API IowKitGetSerialNumber(IOWKIT_HANDLE devHandle, PWCHAR serialNumber);
BOOL IOWKIT_API IowKitSetTimeout(IOWKIT_HANDLE devHandle, ULONG timeout);
BOOL IOWKIT_API IowKitSetWriteTimeout(IOWKIT_HANDLE devHandle, ULONG timeout);
BOOL IOWKIT_API IowKitCancelIo(IOWKIT_HANDLE devHandle, ULONG numPipe);
PCSTR IOWKIT_API IowKitVersion(void);

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // _IOW_KIT_H_
