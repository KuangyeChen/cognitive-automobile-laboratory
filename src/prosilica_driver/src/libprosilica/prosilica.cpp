/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "prosilica/prosilica.h"
#include <prosilica_gige_sdk/PvRegIo.h>
#include <cassert>
#include <cstdio>
#include <ctime>
#include <cstring>
#include <arpa/inet.h>

#include <ros/console.h>

#define CHECK_ERR(fnc, amsg)                               \
do {                                                       \
  tPvErr err = fnc;                                        \
  if (err != ePvErrSuccess) {                              \
    char msg[256];                                         \
    snprintf(msg, 256, "%s: %s", amsg, errorStrings[err]); \
    throw ProsilicaException(err, msg);                    \
  }                                                        \
} while (false)

namespace prosilica {

static const unsigned int MAX_CAMERA_LIST = 10;
static const char* autoValues[] = {"Manual", "Auto", "AutoOnce"};
static const char* triggerModes[] = {"Freerun", "SyncIn1", "SyncIn2", "FixedRate", "Software"};
static const char* acquisitionModes[] = {"Continuous","SingleFrame","MultiFrame","Recorder"};
static const char* errorStrings[] = {"No error",
                                     "Unexpected camera fault",
                                     "Unexpected fault in PvApi or driver",
                                     "Camera handle is invalid",
                                     "Bad parameter to API call",
                                     "Sequence of API calls is incorrect",
                                     "Camera or attribute not found",
                                     "Camera cannot be opened in the specified mode",
                                     "Camera was unplugged",
                                     "Setup is invalid (an attribute is invalid)",
                                     "System/network resources or memory not available",
                                     "1394 bandwidth not available",
                                     "Too many frames on queue",
                                     "Frame buffer is too small",
                                     "Frame cancelled by user",
                                     "The data for the frame was lost",
                                     "Some data in the frame is missing",
                                     "Timeout during wait",
                                     "Attribute value is out of the expected range",
                                     "Attribute is not this type (wrong access function)",
                                     "Attribute write forbidden at this time",
                                     "Attribute is not available at this time",
                                     "A firewall is blocking the traffic"};

static tPvCameraInfo cameraList[MAX_CAMERA_LIST];
static unsigned long cameraNum = 0;

void init()
{
  CHECK_ERR( PvInitialize(), "Failed to initialize Prosilica API" );
}

void fini()
{
  PvUnInitialize();
}

size_t numCameras()
{
  cameraNum = PvCameraList(cameraList, MAX_CAMERA_LIST, NULL);
  return cameraNum;
}

uint64_t getGuid(size_t i)
{
  assert(i < MAX_CAMERA_LIST);
  if (i >= cameraNum)
    throw ProsilicaException(ePvErrBadParameter, "No camera at index i");
  return cameraList[i].UniqueId;
}

std::vector<CameraInfo> listCameras()
{
    std::vector<CameraInfo> cameras;
    tPvCameraInfo cameraList[MAX_CAMERA_LIST];
    unsigned long cameraNum = 0;
    //! get list of all cameras
    cameraNum = PvCameraList(cameraList, MAX_CAMERA_LIST, NULL);
    //! append list of unreachable cameras
    if (cameraNum < MAX_CAMERA_LIST)
    {
        cameraNum += PvCameraListUnreachable(&cameraList[cameraNum], MAX_CAMERA_LIST-cameraNum, NULL);
    }
    if(cameraNum)
    {
        struct in_addr addr;
        tPvIpSettings Conf;

        //! get info
        for(unsigned long i=0; i < cameraNum; i++)
        {

            CameraInfo camInfo;
            camInfo.serial     = to_string(cameraList[i].SerialString);
            camInfo.name       = to_string(cameraList[i].DisplayName);
            camInfo.guid       = to_string(cameraList[i].UniqueId);
            PvCameraIpSettingsGet(cameraList[i].UniqueId,&Conf);
            addr.s_addr = Conf.CurrentIpAddress;
            camInfo.ip_address = to_string(inet_ntoa(addr));
            camInfo.access     = cameraList[i].PermittedAccess & ePvAccessMaster ? true : false;

            cameras.push_back(camInfo);
        }
    }
    return cameras;
}

std::string getIPAddress(uint64_t guid)
{
    struct in_addr addr;
    tPvIpSettings Conf;
    CHECK_ERR(PvCameraIpSettingsGet(guid, &Conf), "Unable to retrieve IP address");
    addr.s_addr = Conf.CurrentIpAddress;
    std::stringstream ip;
    ip<<inet_ntoa(addr);
    return ip.str();
}

/// @todo support opening as monitor?
static void openCamera(boost::function<tPvErr (tPvCameraInfo*)> info_fn,
                       boost::function<tPvErr (tPvAccessFlags)> open_fn)
{
  cameraNum = PvCameraList(cameraList, MAX_CAMERA_LIST, NULL);
  tPvCameraInfo info;
  CHECK_ERR( info_fn(&info), "Unable to find requested camera" );

  if (!(info.PermittedAccess & ePvAccessMaster))
    throw ProsilicaException(ePvErrAccessDenied,
                             "Unable to open camera as master. "
                             "Another process is already using it.");
  
  CHECK_ERR( open_fn(ePvAccessMaster), "Unable to open requested camera" );
}

Camera::Camera(unsigned long guid, size_t bufferSize)
  : bufferSize_(bufferSize), FSTmode_(None)
{
  openCamera(boost::bind(PvCameraInfo, guid, _1),
             boost::bind(PvCameraOpen, guid, _1, &handle_));
  
  setup();
}

Camera::Camera(const char* ip_address, size_t bufferSize)
  : bufferSize_(bufferSize), FSTmode_(None)
{
  unsigned long addr = inet_addr(ip_address);
  tPvIpSettings settings;
  openCamera(boost::bind(PvCameraInfoByAddr, addr, _1, &settings),
             boost::bind(PvCameraOpenByAddr, addr, _1, &handle_));
  
  setup();
}

void Camera::setup()
{
  // adjust packet size according to the current network capacity
  tPvUint32 maxPacketSize = 9000;
  PvCaptureAdjustPacketSize(handle_, maxPacketSize);

  // set data rate to the max
  unsigned long max_data_rate = getMaxDataRate();
  if (max_data_rate < GIGE_MAX_DATA_RATE) {
    ROS_WARN("Detected max data rate is %lu bytes/s, typical maximum data rate for a "
             "GigE port is %lu bytes/s. Are you using a GigE network card and cable?\n",
             max_data_rate, GIGE_MAX_DATA_RATE);
  }
  setAttribute("StreamBytesPerSecond", max_data_rate);

  // capture whole frame by default
  setBinning();
  setRoiToWholeFrame();
  
  // query for attributes (TODO: more)
  CHECK_ERR( PvAttrUint32Get(handle_, "TotalBytesPerFrame", &frameSize_),
             "Unable to retrieve frame size" );
  
  // allocate frame buffers
  frames_ = new tPvFrame[bufferSize_];
  memset(frames_, 0, sizeof(tPvFrame) * bufferSize_);
  for (unsigned int i = 0; i < bufferSize_; ++i)
  {
    frames_[i].ImageBuffer = new char[frameSize_];
    frames_[i].ImageBufferSize = frameSize_;
    frames_[i].Context[0] = (void*)this; // for frameDone callback
  }

  PvLinkCallbackRegister(Camera::kill, ePvLinkRemove, this);
}

Camera::~Camera()
{
  PvLinkCallbackUnRegister(Camera::kill, ePvLinkRemove);
  stop();
  
  PvCameraClose(handle_);

  if (frames_)
  {
    for (unsigned int i = 0; i < bufferSize_; ++i)
      delete[] (char*)frames_[i].ImageBuffer;
    delete[] frames_;
  }
}

void Camera::setFrameCallback(boost::function<void (tPvFrame*)> callback)
{
  userCallback_ = callback;
}
void Camera::setFrameRate(tPvFloat32 frame_rate){
  CHECK_ERR( PvAttrFloat32Set(handle_, "FrameRate", frame_rate),
	     "Could not set frame rate");
}

void Camera::setKillCallback(boost::function<void (unsigned long UniqueId)> callback)
{
    killCallback_ = callback;
}

void Camera::start(FrameStartTriggerMode fmode, tPvFloat32 frame_rate, AcquisitionMode amode)
{
    assert( FSTmode_ == None && fmode != None );
    ///@todo verify this assert again
    assert( fmode == SyncIn1 || fmode == SyncIn2 || fmode == Software || fmode == FixedRate || !userCallback_.empty() );

    // set camera in acquisition mode
    CHECK_ERR( PvCaptureStart(handle_), "Could not start capture");

    if (fmode == Freerun || fmode == FixedRate || fmode == SyncIn1 || fmode == SyncIn2)
    {
        for (unsigned int i = 0; i < bufferSize_; ++i)
            PvCaptureQueueFrame(handle_, frames_ + i, Camera::frameDone);

    }
    else
    {
        bufferIndex_ = 0;
        CHECK_ERR( PvCaptureQueueFrame(handle_, &frames_[bufferIndex_], NULL), "Could not queue frame");
    }

    // start capture after setting acquisition and trigger modes
    try {
        ///@todo take this one also as an argument
        CHECK_ERR( PvAttrEnumSet(handle_, "AcquisitionMode", acquisitionModes[amode]),
                   "Could not set acquisition mode" );
        CHECK_ERR( PvAttrEnumSet(handle_, "FrameStartTriggerMode", triggerModes[fmode]),
                   "Could not set trigger mode" );
        CHECK_ERR( PvCommandRun(handle_, "AcquisitionStart"),
                   "Could not start acquisition" );
    }
    catch (ProsilicaException& e) {
        stop();
        throw; // rethrow
    }
    FSTmode_ = fmode;
    Amode_ = amode;
    
    CHECK_ERR( PvAttrFloat32Set(handle_, "FrameRate", frame_rate),
	           "Could not set frame rate");
}

void Camera::stop()
{
  if (FSTmode_ == None)
    return;
  
  PvCommandRun(handle_, "AcquisitionStop");
  PvCaptureQueueClear(handle_);
  PvCaptureEnd(handle_);
  FSTmode_ = None;
}

void Camera::removeEvents()
{
    // clear all events
    PvAttrUint32Set(handle_, "EventsEnable1", 0);
}

tPvFrame* Camera::grab(unsigned long timeout_ms)
{
    assert( FSTmode_ == Software );
    tPvFrame* frame = &frames_[0];

    CHECK_ERR( PvCommandRun(handle_, "FrameStartTriggerSoftware"), "Couldn't trigger capture" );
    CHECK_ERR( PvCaptureWaitForFrameDone(handle_, frame, timeout_ms), "couldn't capture frame");
    // don't requeue if capture has stopped
    if (frame->Status == ePvErrUnplugged || frame->Status == ePvErrCancelled )
    {
      return NULL;
    }
    CHECK_ERR( PvCaptureQueueFrame(handle_, frame, NULL), "Couldn't queue frame");

    if (frame->Status == ePvErrSuccess)
        return frame;
    if (frame->Status == ePvErrDataMissing || frame->Status == ePvErrTimeout)
    {
        //! recommanding after an error seems to cause a sequence error if next command is too fast
        boost::this_thread::sleep(boost::posix_time::millisec(50));
        return NULL;
    }
    else
        throw std::runtime_error("Unknown error grabbing frame");

    return frame;
}

void Camera::setExposure(unsigned int val, AutoSetting isauto)
{
  CHECK_ERR( PvAttrEnumSet(handle_, "ExposureMode", autoValues[isauto]),
             "Couldn't set exposure mode" );

  if (isauto == Manual)
    CHECK_ERR( PvAttrUint32Set(handle_, "ExposureValue", val),
               "Couldn't set exposure value" );
}

void Camera::setGain(unsigned int val, AutoSetting isauto)
{
  /// @todo Here and in setWhiteBalance, would be better to split off setGainMode etc.
  /// I didn't take into account there are cameras that don't support auto gain, auto white balance.
  if (PvAttrIsAvailable(handle_, "GainMode") == ePvErrSuccess)
  {
    CHECK_ERR( PvAttrEnumSet(handle_, "GainMode", autoValues[isauto]),
               "Couldn't set gain mode" );
  }

  if (isauto == Manual)
    CHECK_ERR( PvAttrUint32Set(handle_, "GainValue", val),
               "Couldn't set gain value" );
}

void Camera::setWhiteBalance(unsigned int blue, unsigned int red, AutoSetting isauto)
{
  if (PvAttrIsAvailable(handle_, "WhitebalMode") == ePvErrSuccess)
  {
    CHECK_ERR( PvAttrEnumSet(handle_, "WhitebalMode", autoValues[isauto]),
               "Couldn't set white balance mode" );
  }

  if (isauto == Manual)
  {
      if(hasAttribute("WhitebalValueBlue"))
      {
        CHECK_ERR( PvAttrUint32Set(handle_, "WhitebalValueBlue", blue),
                   "Couldn't set white balance blue value" );
      }
      if(hasAttribute("WhitebalValueRed"))
      {
        CHECK_ERR( PvAttrUint32Set(handle_, "WhitebalValueRed", red),
                   "Couldn't set white balance red value" );
      }
  }
}

void Camera::setRoi(unsigned int x, unsigned int y,
                    unsigned int width, unsigned int height)
{
  CHECK_ERR( PvAttrUint32Set(handle_, "RegionX", x),
             "Couldn't set region x (left edge)" );
  CHECK_ERR( PvAttrUint32Set(handle_, "RegionY", y),
             "Couldn't set region y (top edge)" );
  CHECK_ERR( PvAttrUint32Set(handle_, "Width", width),
             "Couldn't set region width" );
  CHECK_ERR( PvAttrUint32Set(handle_, "Height", height),
             "Couldn't set region height" );
}

void Camera::setRoiToWholeFrame()
{
  tPvUint32 min_val, max_val;
  CHECK_ERR( PvAttrUint32Set(handle_, "RegionX", 0),
             "Couldn't set region x (left edge)" );
  CHECK_ERR( PvAttrUint32Set(handle_, "RegionY", 0),
             "Couldn't set region y (top edge)" );
  CHECK_ERR( PvAttrRangeUint32(handle_, "Width", &min_val, &max_val),
             "Couldn't get range of Width attribute" );
  CHECK_ERR( PvAttrUint32Set(handle_, "Width", max_val),
             "Couldn't set region width" );
  CHECK_ERR( PvAttrRangeUint32(handle_, "Height", &min_val, &max_val),
             "Couldn't get range of Height attribute" );
  CHECK_ERR( PvAttrUint32Set(handle_, "Height", max_val),
             "Couldn't set region height" );
}

void Camera::setBinning(unsigned int binning_x, unsigned int binning_y)
{
  // Permit setting to "no binning" on cameras without binning support
  if (!hasAttribute("BinningX") && binning_x == 1 && binning_y == 1)
    return;
  
  CHECK_ERR( PvAttrUint32Set(handle_, "BinningX", binning_x),
             "Couldn't set horizontal binning" );
  CHECK_ERR( PvAttrUint32Set(handle_, "BinningY", binning_y),
             "Couldn't set vertical binning" );
}

bool Camera::hasAttribute(const std::string &name)
{
  return (PvAttrIsAvailable(handle_, name.c_str()) == ePvErrSuccess);
}

static void getStringValuedAttribute(std::string &value,
  boost::function<tPvErr (char*, unsigned long, unsigned long*)> get_fn)
{
  if (value.size() == 0)
    value.resize(32);

  unsigned long actual_size;
  CHECK_ERR( get_fn(&value[0], value.size(), &actual_size),
             "Couldn't get attribute" );

  if (actual_size >= value.size()) {
    value.resize(actual_size + 1);
    CHECK_ERR( get_fn(&value[0], value.size(), &actual_size),
               "Couldn't get attribute" );
  }
}

void Camera::getAttributeEnum(const std::string &name, std::string &value)
{
  getStringValuedAttribute(value,
    boost::bind(PvAttrEnumGet, handle_, name.c_str(), _1, _2, _3));
}

void Camera::getAttribute(const std::string &name, tPvUint32 &value)
{
  std::string err_msg = "Couldn't get attribute " + name;
  CHECK_ERR( PvAttrUint32Get(handle_, name.c_str(), &value),
	     err_msg.c_str());
             
}

void Camera::getAttribute(const std::string &name, tPvFloat32 &value)
{
std::string err_msg = "Couldn't get attribute " + name;
  CHECK_ERR( PvAttrFloat32Get(handle_, name.c_str(), &value),
             err_msg.c_str());
}

void Camera::getAttribute(const std::string &name, std::string &value)
{
  getStringValuedAttribute(value,
    boost::bind(PvAttrStringGet, handle_, name.c_str(), _1, _2, _3));
}

void Camera::setAttributeEnum(const std::string &name, const std::string &value)
{
  std::string err_msg = "Couldn't get attribute " + name;
  CHECK_ERR( PvAttrEnumSet(handle_, name.c_str(), value.c_str()),
             err_msg.c_str());
}

void Camera::setAttribute(const std::string &name, tPvUint32 value)
{
  std::string err_msg = "Couldn't set attribute " + name;
  CHECK_ERR( PvAttrUint32Set(handle_, name.c_str(), value),
             err_msg.c_str());
}

void Camera::setAttribute(const std::string &name, tPvFloat32 value)
{
  std::string err_msg = "Couldn't set attribute " + name;
  CHECK_ERR( PvAttrFloat32Set(handle_, name.c_str(), value),
             err_msg.c_str());
}

void Camera::setAttribute(const std::string &name, const std::string &value)
{
  std::string err_msg = "Couldn't set attribute " + name;
  CHECK_ERR( PvAttrStringSet(handle_, name.c_str(), value.c_str()),
             err_msg.c_str());
}

void Camera::runCommand(const std::string& name)
{
  std::string err_msg = "Couldn't run command " + name;
  CHECK_ERR( PvCommandRun(handle_, name.c_str()), err_msg.c_str());
}

unsigned long Camera::guid()
{
  unsigned long id;
  CHECK_ERR( PvAttrUint32Get(handle_, "UniqueId", &id),
             "Couldn't retrieve unique id" );
  return id;
}

unsigned long Camera::getMaxDataRate()
{
  tPvUint32 min_data_rate, max_data_rate;
  CHECK_ERR( PvAttrRangeUint32(handle_, "StreamBytesPerSecond", &min_data_rate, &max_data_rate),
             "Couldn't get range of attribute StreamBytesPerSecond" );
  return max_data_rate;
}

static const unsigned long USER_ADDRESS = 0x17200;

void Camera::writeUserMemory(const char* data, size_t size)
{
  assert(size <= USER_MEMORY_SIZE);

  unsigned char buffer[USER_MEMORY_SIZE] = {0};
  memcpy(buffer, data, size);

  unsigned long written;
  CHECK_ERR( PvMemoryWrite(handle_, USER_ADDRESS, USER_MEMORY_SIZE, buffer, &written),
             "Couldn't write to user memory" );
}

void Camera::readUserMemory(char* data, size_t size)
{
  assert(size <= USER_MEMORY_SIZE);

  unsigned char buffer[USER_MEMORY_SIZE];
  
  CHECK_ERR( PvMemoryRead(handle_, USER_ADDRESS, USER_MEMORY_SIZE, buffer),
             "Couldn't read from user memory" );

  memcpy(data, buffer, size);
}

void Camera::frameDone(tPvFrame* frame)
{

  Camera* camPtr = (Camera*) frame->Context[0];
  if (camPtr && !camPtr->userCallback_.empty()) {
    // TODO: thread safety OK here?
    boost::lock_guard<boost::mutex> guard(camPtr->frameMutex_);
    camPtr->userCallback_(frame);
  }

  // don't requeue if capture has stopped
  if (frame->Status == ePvErrUnplugged || frame->Status == ePvErrCancelled)
  {
    return;
  }
  PvCaptureQueueFrame(camPtr->handle_, frame, Camera::frameDone);
}


void Camera::kill(void* Context,
                   tPvInterface Interface,
                   tPvLinkEvent Event,
                   unsigned long UniqueId)
{
    Camera* camPtr = (Camera*) Context;
    if (camPtr && !camPtr->killCallback_.empty())
    {
        //boost::lock_guard<boost::mutex> guard(camPtr->aliveMutex_);
        camPtr->killCallback_(UniqueId);
    }
}

tPvHandle Camera::handle()
{
  return handle_;
}

} // namespace prosilica

