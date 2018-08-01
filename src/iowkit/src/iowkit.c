/*
 * This one's for linux
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
//#include <asm/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
//#include <linux/input.h>
//#include <linux/ioctl.h>
#include <pthread.h>

#include "iowkit.h"
#include "iowarrior.h"

#define TRUE 1
#define FALSE 0

// Name of the device on our filesystem
static const char dev_name[] =  "/dev/usb/iowarrior";
static char KitVersion[] = "IO-Warrior Kit V1.5";

// A mutex to protect the open call
static pthread_mutex_t device_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef
  struct _IowDevice_t
   {
    int dev_num[IOWKIT_MAX_PIPES];                   // numeric part of device name for the pipe
    struct iowarrior_info info[IOWKIT_MAX_PIPES];  // All the information available for this pipe
    int fd[IOWKIT_MAX_PIPES];                        // the filedescriptor for the pipe
    struct timeval readTimeout;                    // timeout for reading from the device
    struct timeval writeTimeout;                   // timeout for writing to the device, not used on linux
    ULONG readTimeoutVal;                            // read timeout from IowKitSetTimeout
    ULONG writeTimeoutVal;                           // write timeout from IowKitSetWriteTimeout
    ULONG lastValue;
   }
  IowDevice_t;

static IowDevice_t IoWarriors[IOWKIT_MAX_DEVICES];
static ULONG numIoWarriors = 0;

static void IowiClear(void)
 {
  int i, j;

  for (i = 0; i < numIoWarriors; i++)
    for (j = 0; j < IOWKIT_MAX_PIPES; j++)
      if (IoWarriors[i].fd[j] != -1)
        close(IoWarriors[i].fd[j]);
  memset(IoWarriors, 0, sizeof(IoWarriors));
  numIoWarriors = 0;
 }

static IowDevice_t *IowiGetDeviceByHandle(IOWKIT_HANDLE iowHandle)
 {
  int i;

  for (i = 0; i < numIoWarriors; i++)
    if ((IowDevice_t *) iowHandle == &IoWarriors[i])
      return &IoWarriors[i];
  return NULL;
 }

IOWKIT_HANDLE IOWKIT_API IowKitOpenDevice(void)
 {
  char ifname[80];              //The filename for the io interface
  struct iowarrior_info info;   //the info for the device
  int i;
  int fd;
  int ioctlret;

  pthread_mutex_lock(&device_mutex);
  IowiClear();
  for(i = 0; i < IOWKIT_MAX_DEVICES*2; i++)
   {
    sprintf(ifname, "%s%d", dev_name, i);
    fd = open(ifname, O_RDWR);
    if(fd != -1)
     {
      //now get the info for this device
      ioctlret = ioctl(fd, IOW_GETINFO, &info);
      if(ioctlret != -1)
       {
        IoWarriors[numIoWarriors].dev_num[info.if_num] = i;
        IoWarriors[numIoWarriors].info[info.if_num] = info;
        IoWarriors[numIoWarriors].fd[info.if_num] = fd;
        IoWarriors[numIoWarriors].lastValue = 0xFFFFFFFF;
        IoWarriors[numIoWarriors].readTimeoutVal = 0xFFFFFFFF;
        IoWarriors[numIoWarriors].writeTimeoutVal = 0xFFFFFFFF;
        if (info.if_num == 1)
          numIoWarriors++;
       }
      else
        close(fd);
     }
   }

  pthread_mutex_unlock(&device_mutex);
  return IowKitGetDeviceHandle(1);
 }


ULONG IOWKIT_API IowKitGetNumDevs(void)
 {
  return numIoWarriors;
 }

IOWKIT_HANDLE IOWKIT_API IowKitGetDeviceHandle(ULONG numDevice)
 {
  if (numDevice >= 1 && numDevice <= numIoWarriors)
    return (IOWKIT_HANDLE) &IoWarriors[numDevice - 1];
  else
    return NULL;
 }

BOOL IOWKIT_API IowKitGetSerialNumber(IOWKIT_HANDLE iowHandle, PWCHAR serialNumber)
 {
  IowDevice_t *dev;

  if (serialNumber == NULL)
    return FALSE;
  dev = IowiGetDeviceByHandle(iowHandle);
  if(dev != NULL)
   {
    int i;

    for(i = 0; i < 9; i++)
      serialNumber[i] = (unsigned short) dev->info[IOW_PIPE_IO_PINS].serial[i];
   }
  else
    serialNumber[0] = 0;
  return serialNumber[0] != 0;
 }

ULONG IOWKIT_API IowKitGetProductId(IOWKIT_HANDLE iowHandle)
 {
  IowDevice_t *dev;

  dev = IowiGetDeviceByHandle(iowHandle);
  if(dev != NULL)
    return (ULONG) dev->info[IOW_PIPE_IO_PINS].product;
  else
    return 0;
 }

ULONG IOWKIT_API IowKitGetRevision(IOWKIT_HANDLE iowHandle)
 {
  IowDevice_t *dev;

  dev = IowiGetDeviceByHandle(iowHandle);
  if(dev != NULL)
    return (ULONG) dev->info[IOW_PIPE_IO_PINS].revision;
  else
    return 0;
 }

void IOWKIT_API IowKitCloseDevice(IOWKIT_HANDLE devHandle)
 { 
  pthread_mutex_lock(&device_mutex);
  IowiClear();
  pthread_mutex_unlock(&device_mutex);
 }


BOOL IOWKIT_API IowKitSetTimeout(IOWKIT_HANDLE devHandle, ULONG timeout)
 {
  IowDevice_t *dev;

  dev = IowiGetDeviceByHandle(devHandle);
  if(dev != NULL)
   {
    dev->readTimeout.tv_sec = (time_t)(timeout / 1000);
    dev->readTimeout.tv_usec = (long) ((timeout % 1000) * 1000);
    dev->readTimeoutVal = timeout;
   }
  return dev != NULL;
 }

BOOL IOWKIT_API IowKitSetWriteTimeout(IOWKIT_HANDLE devHandle, ULONG timeout)
 {
  IowDevice_t *dev;

  dev = IowiGetDeviceByHandle(devHandle);
  if(dev != NULL)
   {
    dev->writeTimeout.tv_sec = (time_t)(timeout / 1000);
    dev->writeTimeout.tv_usec = (long) ((timeout % 1000) * 1000);
    dev->writeTimeoutVal = timeout;
   }
  return dev != NULL;
 }

BOOL IOWKIT_API IowKitSetLegacyOpenMode(ULONG legacyOpenMode)
 {
  return ((legacyOpenMode == IOW_OPEN_SIMPLE) || (legacyOpenMode == IOW_OPEN_COMPLEX));
 }

ULONG IOWKIT_API IowKitRead(IOWKIT_HANDLE devHandle, ULONG numPipe, PCHAR buffer, ULONG length)
 {
  IowDevice_t *dev;
  struct timeval tv;
  fd_set rfds;
  ULONG rbc;
  int result, siz, extra;

  if (buffer == NULL)
    return 0;
  dev = IowiGetDeviceByHandle(devHandle);
  if(dev != NULL)
   {
    siz = dev->info[numPipe].packet_size;
    switch(numPipe)
     {
      case IOW_PIPE_IO_PINS:
        extra = 1;
        break;
      case IOW_PIPE_SPECIAL_MODE:
        extra = 0;
        break;
      default:
        return 0;
     }
    for(rbc = 0; rbc + siz + extra <= length; )
     {
      tv = dev->readTimeout;
      FD_ZERO(&rfds);                    // clear the read-descriptor for select
      FD_SET(dev->fd[numPipe], &rfds);   // add the filedescriptor to read-descriptor
      if (dev->readTimeoutVal != 0xFFFFFFFF)
        result = select(dev->fd[numPipe] + 1, &rfds, NULL, NULL, &tv);
      else
        result = select(dev->fd[numPipe] + 1, &rfds, NULL, NULL, NULL);
      if(result > 0 && FD_ISSET(dev->fd[numPipe], &rfds))
       {
        // put in eventual ReportID 0 to be a Windows lookalike
        *buffer = '\0';
        buffer += extra;
        // there should be some data ready for reading now
        if (read(dev->fd[numPipe], buffer, siz) != siz)
          break;
        if (numPipe == IOW_PIPE_IO_PINS)
          memcpy(&dev->lastValue, buffer, sizeof(DWORD));
        buffer += siz;
        rbc += siz + extra;
       }
      else
      // if we encountered a timeout then do not try to read several reports
      if(result == 0)
        break;
     }
    return rbc;
   }
  return 0;
 }

ULONG IOWKIT_API IowKitReadNonBlocking(IOWKIT_HANDLE devHandle, ULONG numPipe, PCHAR buffer, ULONG length)
 {
  IowDevice_t *dev;
  struct timeval tv;
  fd_set rfds;
  ULONG rbc;
  int result, siz, extra;

  if (buffer == NULL)
    return 0;
  dev = IowiGetDeviceByHandle(devHandle);
  if(dev != NULL)
   {
    siz = dev->info[numPipe].packet_size;
    switch(numPipe)
     {
      case IOW_PIPE_IO_PINS:
        extra = 1;
        break;
      case IOW_PIPE_SPECIAL_MODE:
        extra = 0;
        break;
      default:
        return 0;
     }
    for(rbc = 0; rbc + siz + extra <= length; )
     {
      tv.tv_sec = 0;
      tv.tv_usec = 0;
      FD_ZERO(&rfds);     // clear the read-descriptor for select
      FD_SET(dev->fd[numPipe], &rfds);  // add the filedescriptor to read-descriptor
      result = select(dev->fd[numPipe] + 1, &rfds, NULL, NULL, &tv);
      if(result > 0 && FD_ISSET(dev->fd[numPipe], &rfds))
       {
        // put in eventual ReportID 0 to be a Windows lookalike
        *buffer = '\0';
        buffer += extra;
        // there should be some data ready for reading now
        if (read(dev->fd[numPipe], buffer, siz) != siz)
          break;
        if (numPipe == IOW_PIPE_IO_PINS)
          memcpy(&dev->lastValue, buffer, sizeof(DWORD));
        buffer += siz;
        rbc += siz + extra;
       }
      else
      if(result == 0)
        break;
     }
    return rbc;
   }
  return 0;
 }

BOOL IOWKIT_API IowKitReadImmediate(IOWKIT_HANDLE devHandle, PDWORD value)
 {
  IowDevice_t *dev;
  IOWKIT_SPECIAL_REPORT report;

  dev = IowiGetDeviceByHandle(devHandle);
  if (value == NULL || dev == NULL || dev->info[IOW_PIPE_IO_PINS].product == IOWKIT_PID_IOW56)
    return FALSE;
  memset(&report, 0, sizeof(report));
  report.ReportID = 0xff;
  if (IowKitWrite(devHandle, IOW_PIPE_SPECIAL_MODE, (PCHAR) &report, IOWKIT_SPECIAL_REPORT_SIZE))
   {
    if (IowKitRead(devHandle, IOW_PIPE_SPECIAL_MODE, (PCHAR) &report, IOWKIT_SPECIAL_REPORT_SIZE) == IOWKIT_SPECIAL_REPORT_SIZE &&
        report.ReportID == 0xff)
     {
      memcpy(value, &report.Bytes[0], sizeof(DWORD));
      dev->lastValue = *value;
      return TRUE;
     }
    else
      return FALSE;
   }
  else
    return FALSE;
 }

ULONG IOWKIT_API IowKitWrite(IOWKIT_HANDLE devHandle, ULONG numPipe, PCHAR buffer, ULONG length)
 {
  IowDevice_t *dev;
  ULONG rbc;
  int siz, extra;

  dev = IowiGetDeviceByHandle(devHandle);
  if(dev != NULL)
   {
    siz = dev->info[numPipe].packet_size;
    switch(numPipe)
     {
      case IOW_PIPE_IO_PINS:
        extra = 1;
        break;
      case IOW_PIPE_SPECIAL_MODE:
        extra = 0;
        break;
      default:
        return 0;
     }
    if (buffer == NULL)
      return 0;
    for(rbc = 0; rbc + siz + extra <= length; rbc += siz + extra)
     {
      buffer += extra;
      if (write(dev->fd[numPipe], buffer, siz) != siz)
        break;
      buffer += siz;
     }
    return rbc;
   }
  return 0;
 }

BOOL IOWKIT_API IowKitCancelIo(IOWKIT_HANDLE devHandle, ULONG numPipe)
 {
  return FALSE;
 }

HANDLE IOWKIT_API IowKitGetThreadHandle(IOWKIT_HANDLE iowHandle)
 {
  // no threads used for IowKitReadImmediate
  return 0;
 }

PCSTR IOWKIT_API IowKitVersion(void)
 {
  return KitVersion;
 }
