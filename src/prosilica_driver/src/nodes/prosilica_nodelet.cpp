/*********************************************************************
* Software License Agreement (BSD License)
*
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Willow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <polled_camera/publication_server.h>
#include <sensor_msgs/CameraInfo.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <boost/thread.hpp>

#include <prosilica_driver/ProsilicaCameraConfig.h>
#include "prosilica/prosilica.h"
#include "prosilica/rolling_sum.h"

bool prosilica_inited = false;//for if the nodelet is loaded multiple times in the same manager
int num_cameras = 0;

namespace prosilica_driver
{

class ProsilicaNodelet : public nodelet::Nodelet
{

public:

    virtual ~ProsilicaNodelet()
    {
        //! Make sure we interrupt initialization (if it happened to still execute).
        init_thread_.interrupt();
        init_thread_.join();

        if(camera_)
        {
            camera_->stop();
            camera_.reset(); // must destroy Camera before calling prosilica::fini
        }

        trigger_sub_.shutdown();
        poll_srv_.shutdown();
        image_publisher_.shutdown();

        --num_cameras;
        if(num_cameras<=0)
        {
            prosilica::fini();
            prosilica_inited = false;
            num_cameras = 0;
        }

        NODELET_WARN("Unloaded prosilica camera with guid %s", hw_id_.c_str());
    }

    ProsilicaNodelet()
      : auto_adjust_stream_bytes_per_second_(true),
        count_(0),
        frames_dropped_total_(0), frames_completed_total_(0),
        frames_dropped_acc_(WINDOW_SIZE),
        frames_completed_acc_(WINDOW_SIZE),
        packets_missed_total_(0), packets_received_total_(0),
        packets_missed_acc_(WINDOW_SIZE),
        packets_received_acc_(WINDOW_SIZE)
    {
        ++num_cameras;
    }

private:

    boost::shared_ptr<prosilica::Camera> camera_;
    boost::thread init_thread_;
    ros::Timer update_timer_;

    image_transport::CameraPublisher                           image_publisher_;
    polled_camera::PublicationServer                           poll_srv_;
    ros::ServiceServer                                         set_camera_info_srv_;
    ros::Subscriber                                            trigger_sub_;

    sensor_msgs::Image img_;
    sensor_msgs::CameraInfo cam_info_;

    std::string   frame_id_;
    unsigned long guid_;
    std::string   hw_id_;
    std::string   ip_address_;
    double        open_camera_retry_period_;
    std::string   trig_timestamp_topic_;
    ros::Time     trig_time_;

    // Dynamic reconfigure parameters
    double        update_rate_;
    int           trigger_mode_;
    bool          auto_adjust_stream_bytes_per_second_;

    tPvUint32 sensor_width_, sensor_height_;
    tPvUint32 max_binning_x, max_binning_y, dummy;
    int count_;

    // Dynamic Reconfigure
    prosilica_driver::ProsilicaCameraConfig last_config_;
    boost::recursive_mutex config_mutex_;
    typedef dynamic_reconfigure::Server<prosilica_driver::ProsilicaCameraConfig> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;

    // State updater
    enum CameraState
    {
        OPENING,
        CAMERA_NOT_FOUND,
        FORMAT_ERROR,
        ERROR,
        OK
    }camera_state_;
    std::string state_info_;
    std::string intrinsics_;
    static const int WINDOW_SIZE = 100; // remember previous 5s
    unsigned long frames_dropped_total_, frames_completed_total_;
    RollingSum<unsigned long> frames_dropped_acc_, frames_completed_acc_;
    unsigned long packets_missed_total_, packets_received_total_;
    RollingSum<unsigned long> packets_missed_acc_, packets_received_acc_;

    diagnostic_updater::Updater updater;


    virtual void onInit()
    {
        //! We will be retrying to open camera until it is open, which may block the
        //! thread. Nodelet::onInit() should not block, hence spawning a new thread
        //! to do initialization.
        init_thread_ = boost::thread(boost::bind(&ProsilicaNodelet::onInitImpl, this));

    }

    void onInitImpl()
    {
        ros::NodeHandle& nh = getNodeHandle();
        ros::NodeHandle& pn = getPrivateNodeHandle();

        //! initialize prosilica if necessary
        if(!prosilica_inited)
        {
            NODELET_INFO("Initializing prosilica GIGE API");
            prosilica::init();
            prosilica_inited = true;
        }

        //! Retrieve parameters from server
        count_ = 0;
        update_rate_=30;
        NODELET_INFO("namespace: %s", pn.getNamespace().c_str());
        pn.param<std::string>("frame_id", frame_id_, "/camera_optical_frame");
        NODELET_INFO("Loaded param frame_id: %s", frame_id_.c_str());

        pn.param<std::string>("guid", hw_id_, "");
        if(hw_id_ == "")
        {
            guid_ = 0;
        }
        else
        {
            guid_ = boost::lexical_cast<int>(hw_id_);
            NODELET_INFO("Loaded param guid: %lu", guid_);
        }

        pn.param<std::string>("ip_address", ip_address_, "");
        NODELET_INFO("Loaded ip address: %s", ip_address_.c_str());

        pn.param<double>("open_camera_retry_period", open_camera_retry_period_, 3.);
        NODELET_INFO("Retry period: %f", open_camera_retry_period_);

        // Setup updater
        updater.add(getName().c_str(), this, &ProsilicaNodelet::getCurrentState);
        NODELET_INFO("updated state");
        // Setup periodic callback to get new data from the camera
        update_timer_ = nh.createTimer(ros::Rate(update_rate_).expectedCycleTime(), &ProsilicaNodelet::updateCallback, this, false ,false);
        update_timer_.stop();
        NODELET_INFO("created update timer");
        // Open camera
        openCamera();

        // Advertise topics
        ros::NodeHandle image_nh(nh);
        image_transport::ImageTransport image_it(image_nh);
        image_publisher_ = image_it.advertiseCamera("image_raw", 1);
        poll_srv_ = polled_camera::advertise(nh, "request_image", &ProsilicaNodelet::pollCallback, this);
        set_camera_info_srv_ = pn.advertiseService("set_camera_info", &ProsilicaNodelet::setCameraInfo, this);
        trigger_sub_ = pn.subscribe(trig_timestamp_topic_, 1, &ProsilicaNodelet::syncInCallback, this);

        // Setup dynamic reconfigure server
        reconfigure_server_.reset(new ReconfigureServer(config_mutex_, pn));
        ReconfigureServer::CallbackType f = boost::bind(&ProsilicaNodelet::reconfigureCallback, this, _1, _2);
        reconfigure_server_->setCallback(f);
    }

    void openCamera()
    {
        while (!camera_)
        {
            boost::lock_guard<boost::recursive_mutex> scoped_lock(config_mutex_);
            camera_state_ = OPENING;
            try
            {
                if(guid_ != 0)
                {
                    state_info_ = "Trying to load camera with guid " + hw_id_;
                    NODELET_INFO("%s", state_info_.c_str());
                    camera_ = boost::make_shared<prosilica::Camera>((unsigned long)guid_);
                    updater.setHardwareIDf("%d", guid_);
                    NODELET_INFO("Started Prosilica camera with guid \"%lu\"", guid_);

                }
                else if(!ip_address_.empty())
                {
                    state_info_ = "Trying to load camera with ipaddress: " + ip_address_;
                    NODELET_INFO("%s", state_info_.c_str());
                    camera_ = boost::make_shared<prosilica::Camera>(ip_address_.c_str());
                    guid_ = camera_->guid();
                    hw_id_ = boost::lexical_cast<std::string>(guid_);
                    updater.setHardwareIDf("%d", guid_);

                    NODELET_INFO("Started Prosilica camera with guid \"%d\"", (int)camera_->guid());
                }
                else
                {
                    updater.setHardwareID("unknown");
                    if(prosilica::numCameras()>0)
                    {
                        state_info_ = "Trying to load first camera found";
                        NODELET_INFO("%s", state_info_.c_str());
                        guid_ = prosilica::getGuid(0);
                        camera_ = boost::make_shared<prosilica::Camera>((unsigned long)guid_);
                        hw_id_ = boost::lexical_cast<std::string>(guid_);
                        updater.setHardwareIDf("%d", guid_);
                        NODELET_INFO("Started Prosilica camera with guid \"%d\"", (int)guid_);
                    }
                    else
                    {
                        throw std::runtime_error("Found no cameras on local subnet");
                    }
                }

            }
            catch (std::exception& e)
            {
                camera_state_ = CAMERA_NOT_FOUND;
                std::stringstream err;
                if (prosilica::numCameras() == 0)
                {
                    err << "Found no cameras on local subnet";
                }
                else if (guid_ != 0)
                {
                    err << "Unable to open prosilica camera with guid " << guid_ <<": "<<e.what();
                }
                else if (ip_address_ != "")
                {
                    err << "Unable to open prosilica camera with ip address " << ip_address_ <<": "<<e.what();
                }

                state_info_ = err.str();
                NODELET_WARN("%s", state_info_.c_str());
                if(prosilica::numCameras() > 0)
                {
                    NODELET_INFO("Available cameras:\n%s", getAvailableCameras().c_str());
                }
                camera_.reset();

            }
            updater.update();
            boost::this_thread::sleep(boost::posix_time::seconds(open_camera_retry_period_));
        }
        loadIntrinsics();
        start();
    }

    std::string getAvailableCameras()
    {
        std::vector<prosilica::CameraInfo> cameras = prosilica::listCameras();
        std::stringstream list;
        for (unsigned int i = 0; i < cameras.size(); ++i)
        {
            list << cameras[i].serial << " - " <<cameras[i].name<< " - Unique ID = "<<cameras[i].guid<<" IP = "<<cameras[i].ip_address<<std::endl;
        }
        return list.str();
    }

    void loadIntrinsics()
    {
        try
        {
            camera_->setKillCallback(boost::bind(&ProsilicaNodelet::kill, this, _1));

            if(auto_adjust_stream_bytes_per_second_ && camera_->hasAttribute("StreamBytesPerSecond"))
                camera_->setAttribute("StreamBytesPerSecond", (tPvUint32)(115000000/num_cameras));

            // Retrieve contents of user memory
            std::string buffer(prosilica::Camera::USER_MEMORY_SIZE, '\0');
            camera_->readUserMemory(&buffer[0], prosilica::Camera::USER_MEMORY_SIZE);

            PvAttrRangeUint32(camera_->handle(), "BinningX", &dummy, &max_binning_x);
            PvAttrRangeUint32(camera_->handle(), "BinningY", &dummy, &max_binning_y);
            PvAttrRangeUint32(camera_->handle(), "Width",    &dummy, &sensor_width_);
            PvAttrRangeUint32(camera_->handle(), "Height",   &dummy, &sensor_height_);


            // Parse calibration file
            std::string camera_name;
            if (camera_calibration_parsers::parseCalibrationIni(buffer, camera_name, cam_info_))
            {
                intrinsics_ = "Loaded calibration";
                NODELET_INFO("Loaded calibration for camera '%s'", camera_name.c_str());
            }
            else
            {
                intrinsics_ = "Failed to load intrinsics from camera";
                NODELET_WARN("Failed to load intrinsics from camera");
            }
        }
        catch(std::exception &e)
        {
            camera_state_ = CAMERA_NOT_FOUND;
            state_info_ = e.what();
        }
    }

    void start()
    {
        try
        {
            switch(trigger_mode_)
            {
                case prosilica::Software:
                    NODELET_INFO("starting camera %s in software trigger mode", hw_id_.c_str());
                    camera_->start(prosilica::Software, 1., prosilica::Continuous);
                    if(update_rate_ > 0)
                    {
                        update_timer_.setPeriod(ros::Rate(update_rate_).expectedCycleTime());
                        update_timer_.start();
                    }
                    break;
                case prosilica::Freerun:
                    NODELET_INFO("starting camera %s in freerun trigger mode", hw_id_.c_str());
                    camera_->setFrameCallback(boost::bind(&ProsilicaNodelet::publishImage, this, _1));
                    camera_->start(prosilica::Freerun, 1., prosilica::Continuous);
                    break;
                case prosilica::FixedRate:
                    NODELET_INFO("starting camera %s in fixedrate trigger mode", hw_id_.c_str());
                    camera_->setFrameCallback(boost::bind(&ProsilicaNodelet::publishImage, this, _1));
                    camera_->start(prosilica::FixedRate, update_rate_, prosilica::Continuous);
                    break;
                case prosilica::SyncIn1:
                    NODELET_INFO("starting camera %s in sync1 trigger mode", hw_id_.c_str());
                    camera_->setFrameCallback(boost::bind(&ProsilicaNodelet::publishImage, this, _1));
                    camera_->start(prosilica::SyncIn1, update_rate_, prosilica::Continuous);
                    break;
                case prosilica::SyncIn2:
                    NODELET_INFO("starting camera %s in sync2 trigger mode", hw_id_.c_str());
                    camera_->setFrameCallback(boost::bind(&ProsilicaNodelet::publishImage, this, _1));
                    camera_->start(prosilica::SyncIn2, update_rate_, prosilica::Continuous);
                    break;
                default:
                    break;
            }
        }
        catch(std::exception &e)
        {
            camera_state_ = CAMERA_NOT_FOUND;
            state_info_ = e.what();
        }
    }

    void stop()
    {
        update_timer_.stop();
        if(!camera_)
            return;
        camera_->removeEvents();
        camera_->stop();

    }

    void kill(unsigned long guid)
    {
        if(guid == guid_)
        {
            NODELET_WARN("[%s] got kill request for prosilica camera %lu",getName().c_str(), guid);
            //! Make sure we interrupt initialization (if it happened to still execute).
            init_thread_.interrupt();
            init_thread_.join();

            camera_state_ = CAMERA_NOT_FOUND;
            state_info_ = "Prosilica camera " + hw_id_ + " disconnected";
            NODELET_ERROR("%s", state_info_.c_str());
            updater.update();
            boost::lock_guard<boost::recursive_mutex> scoped_lock(config_mutex_);
            stop();
            camera_.reset();
            init_thread_ = boost::thread(boost::bind(&ProsilicaNodelet::openCamera, this));
            return;
        }
    }

    void publishImage(tPvFrame* frame)
    {
        publishImage(frame, ros::Time::now());
    }

    void publishImage(tPvFrame* frame, ros::Time time)
    {
        camera_state_ = OK;
        state_info_ = "Camera operating normally";
        if (image_publisher_.getNumSubscribers() > 0)
        {

            if (processFrame(frame, img_, cam_info_))
            {
                image_publisher_.publish(img_, cam_info_, time);
                frames_dropped_acc_.add(0);
            }
            else
            {
                camera_state_ = FORMAT_ERROR;
                state_info_ = "Unable to process frame";
                ++frames_dropped_total_;
                frames_dropped_acc_.add(1);
            }
            ++frames_completed_total_;
            frames_completed_acc_.add(1);
        }
        updater.update();
    }

    void updateCallback(const ros::TimerEvent &event)
    {
        // Download the most recent data from the device
        camera_state_ = OK;
        state_info_ = "Camera operating normally";
        if(image_publisher_.getNumSubscribers() > 0)
        {
            boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
            try
            {
                tPvFrame* frame = NULL;
                frame = camera_->grab(1000);
                publishImage(frame, event.current_real);
            }
            catch(std::exception &e)
            {
                camera_state_ = ERROR;
                state_info_ = e.what();
                NODELET_ERROR("Unable to read from camera: %s", e.what());
                ++frames_dropped_total_;
                frames_dropped_acc_.add(1);
                updater.update();
                return;
            }
        }
    }

    void pollCallback(polled_camera::GetPolledImage::Request& req,
                      polled_camera::GetPolledImage::Response& rsp,
                      sensor_msgs::Image& image, sensor_msgs::CameraInfo& info)
    {
        if (trigger_mode_ != prosilica::Software)
        {
            rsp.success = false;
            rsp.status_message = "Camera is not in software triggered mode";
            return;
        }

        last_config_.binning_x = req.binning_x;
        last_config_.binning_y = req.binning_y;
        last_config_.x_offset = req.roi.x_offset;
        last_config_.y_offset = req.roi.y_offset;
        last_config_.height   = req.roi.height;
        last_config_.width    = req.roi.width;

        reconfigureCallback(last_config_, driver_base::SensorLevels::RECONFIGURE_RUNNING);

        try
        {
            tPvFrame* frame = NULL;
            frame = camera_->grab(req.timeout.toSec()*100);
            if (processFrame(frame, image, info))
            {
                image.header.stamp = info.header.stamp =rsp.stamp = ros::Time::now();
                rsp.status_message = "Success";
                rsp.success = true;
            }
            else
            {
                rsp.success = false;
                rsp.status_message = "Failed to process image";
                return;
            }
        }
        catch(std::exception &e)
        {
            rsp.success = false;
            std::stringstream err;
            err<< "Failed to grab frame: "<<e.what();
            rsp.status_message = err.str();
            return;
        }
    }

    void syncInCallback (const std_msgs::HeaderConstPtr& msg)
    {
        if (trigger_mode_ != prosilica::Software)
        {
            camera_state_ = ERROR;
            state_info_ = "Can not sync from topic trigger unless in Software Trigger mode";
            NODELET_ERROR_ONCE("%s", state_info_.c_str());
            return;
        }
        ros::TimerEvent e;
        e.current_real = msg->stamp;
        updateCallback(e);
    }

    bool processFrame(tPvFrame* frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info)
    {
        /// @todo Match time stamp from frame to ROS time?
        if (frame==NULL || frame->Status != ePvErrSuccess)
            return false;

        try
        {
            /// @todo Binning values retrieved here may differ from the ones used to actually
            /// capture the frame! Maybe need to clear queue when changing binning and/or
            /// stuff binning values into context?
            tPvUint32 binning_x = 1, binning_y = 1;
            if (camera_->hasAttribute("BinningX"))
            {
                camera_->getAttribute("BinningX", binning_x);
                camera_->getAttribute("BinningY", binning_y);
            }

            // Binning averages bayer samples, so just call it mono8 in that case
            if (frame->Format == ePvFmtBayer8 && (binning_x > 1 || binning_y > 1))
                frame->Format = ePvFmtMono8;

            if (!frameToImage(frame, img))
                return false;

            // Set the operational parameters in CameraInfo (binning, ROI)
            cam_info.binning_x = binning_x;
            cam_info.binning_y = binning_y;
            // ROI in CameraInfo is in unbinned coordinates, need to scale up
            cam_info.roi.x_offset = frame->RegionX * binning_x;
            cam_info.roi.y_offset = frame->RegionY * binning_y;
            cam_info.roi.height = frame->Height * binning_y;
            cam_info.roi.width = frame->Width * binning_x;
            cam_info.roi.do_rectify = (frame->Height != sensor_height_ / binning_y) ||
                                       (frame->Width  != sensor_width_  / binning_x);

            if(auto_adjust_stream_bytes_per_second_ && camera_->hasAttribute("StreamBytesPerSecond"))
                camera_->setAttribute("StreamBytesPerSecond", (tPvUint32)(115000000/num_cameras));
        }
        catch(std::exception &e)
        {
            return false;
        }

        count_++;
        return true;
    }

    bool frameToImage(tPvFrame* frame, sensor_msgs::Image &image)
    {
        // NOTE: 16-bit and Yuv formats not supported
        static const char* BAYER_ENCODINGS[] = { "bayer_rggb8", "bayer_gbrg8", "bayer_grbg8", "bayer_bggr8" };

        std::string encoding;
        if (frame->Format == ePvFmtMono8)       encoding = sensor_msgs::image_encodings::MONO8;
        else if (frame->Format == ePvFmtBayer8) encoding = BAYER_ENCODINGS[frame->BayerPattern];
        else if (frame->Format == ePvFmtRgb24)  encoding = sensor_msgs::image_encodings::RGB8;
        else if (frame->Format == ePvFmtBgr24)  encoding = sensor_msgs::image_encodings::BGR8;
        else if (frame->Format == ePvFmtRgba32) encoding = sensor_msgs::image_encodings::RGBA8;
        else if (frame->Format == ePvFmtBgra32) encoding = sensor_msgs::image_encodings::BGRA8;
        else {
        NODELET_WARN("Received frame with unsupported pixel format %d", frame->Format);
        return false;
      }

      if(frame->ImageSize == 0 || frame->Height == 0)
          return false;
      uint32_t step = frame->ImageSize / frame->Height;
      return sensor_msgs::fillImage(image, encoding, frame->Height, frame->Width, step, frame->ImageBuffer);
    }

    bool setCameraInfo(sensor_msgs::SetCameraInfoRequest &req, sensor_msgs::SetCameraInfoResponse &rsp)
    {
        NODELET_INFO("New camera info received");
        sensor_msgs::CameraInfo &info = req.camera_info;

        // Sanity check: the image dimensions should match the max resolution of the sensor.
        if (info.width != sensor_width_ || info.height != sensor_height_)
        {
            rsp.success = false;
            rsp.status_message = (boost::format("Camera_info resolution %ix%i does not match current video "
                                                "setting, camera running at resolution %ix%i.")
                                                 % info.width % info.height % sensor_width_ % sensor_height_).str();
            NODELET_ERROR("%s", rsp.status_message.c_str());
            return true;
        }

        stop();

        std::string cam_name = "prosilica";
        cam_name += hw_id_;
        std::stringstream ini_stream;
        if (!camera_calibration_parsers::writeCalibrationIni(ini_stream, cam_name, info))
        {
            rsp.status_message = "Error formatting camera_info for storage.";
            rsp.success = false;
        }
        else
        {
            std::string ini = ini_stream.str();
            if (ini.size() > prosilica::Camera::USER_MEMORY_SIZE)
            {
                rsp.success = false;
                rsp.status_message = "Unable to write camera_info to camera memory, exceeded storage capacity.";
            }
            else
            {
                try
                {
                    camera_->writeUserMemory(ini.c_str(), ini.size());
                    cam_info_ = info;
                    rsp.success = true;
                }
                catch (prosilica::ProsilicaException &e)
                {
                    rsp.success = false;
                    rsp.status_message = e.what();
                }
            }
        }
        if (!rsp.success)
        NODELET_ERROR("%s", rsp.status_message.c_str());

        start();

        return true;
    }

    void reconfigureCallback(prosilica_driver::ProsilicaCameraConfig &config, uint32_t level)
    {
        NODELET_DEBUG("Reconfigure request received");

        if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP)
            stop();

        //! Trigger mode
        if (config.trigger_mode == "streaming")
        {
            trigger_mode_ = prosilica::Freerun;
            update_rate_ = 1.; // make sure we get _something_
        }
        else if (config.trigger_mode == "syncin1")
        {
            trigger_mode_ = prosilica::SyncIn1;
            update_rate_ = config.trig_rate;
        }
        else if (config.trigger_mode == "syncin2")
        {
            trigger_mode_ = prosilica::SyncIn2;
            update_rate_ = config.trig_rate;
        }
        else if (config.trigger_mode == "fixedrate")
        {
            trigger_mode_ = prosilica::FixedRate;
            update_rate_ = config.trig_rate;
        }
        else if (config.trigger_mode == "software")
        {
            trigger_mode_ = prosilica::Software;
            update_rate_ = config.trig_rate;
        }

        else if (config.trigger_mode == "polled")
        {
            trigger_mode_ = prosilica::Software;
            update_rate_ = 0;
        }
        else if (config.trigger_mode == "triggered")
        {
            trigger_mode_ = prosilica::Software;
            update_rate_ = 0;
        }
        else
        {
            NODELET_ERROR("Invalid trigger mode '%s' in reconfigure request", config.trigger_mode.c_str());
        }

        if(config.trig_timestamp_topic != last_config_.trig_timestamp_topic)
        {
            trigger_sub_.shutdown();
            trig_timestamp_topic_ = config.trig_timestamp_topic;
        }

        if(!trigger_sub_ && config.trigger_mode == "triggered")
        {
            trigger_sub_ = ros::NodeHandle().subscribe(trig_timestamp_topic_, 1, &ProsilicaNodelet::syncInCallback, this);
        }


        // Exposure
        if (config.auto_exposure)
        {
            camera_->setExposure(0, prosilica::Auto);
            if (camera_->hasAttribute("ExposureAutoMax"))
            {
                tPvUint32 us = config.exposure_auto_max*1000000. + 0.5;
                camera_->setAttribute("ExposureAutoMax", us);
            }
            if (camera_->hasAttribute("ExposureAutoTarget"))
                camera_->setAttribute("ExposureAutoTarget", (tPvUint32)config.exposure_auto_target);
        }
        else
        {
            unsigned us = config.exposure*1000000. + 0.5;
            camera_->setExposure(us, prosilica::Manual);
            camera_->setAttribute("ExposureValue", (tPvUint32)us);
        }

        // Gain
        if (config.auto_gain)
        {
            if (camera_->hasAttribute("GainAutoMax"))
            {
                camera_->setGain(0, prosilica::Auto);
                camera_->setAttribute("GainAutoMax", (tPvUint32)config.gain_auto_max);
                camera_->setAttribute("GainAutoTarget", (tPvUint32)config.gain_auto_target);
            }
            else
            {
                tPvUint32 major, minor;
                camera_->getAttribute("FirmwareVerMajor", major);
                camera_->getAttribute("FirmwareVerMinor", minor);
                NODELET_WARN("Auto gain not available for this camera. Auto gain is available "
                "on firmware versions 1.36 and above. You are running version %u.%u.",
                (unsigned)major, (unsigned)minor);
                config.auto_gain = false;
            }
        }
        else
        {
            camera_->setGain(config.gain, prosilica::Manual);
            camera_->setAttribute("GainValue", (tPvUint32)config.gain);
        }

        // White balance
        if (config.auto_whitebalance)
        {
            if (camera_->hasAttribute("WhitebalMode"))
                camera_->setWhiteBalance(0, 0, prosilica::Auto);
            else
            {
                NODELET_WARN("Auto white balance not available for this camera.");
                config.auto_whitebalance = false;
            }
        }
        else
        {
            camera_->setWhiteBalance(config.whitebalance_blue, config.whitebalance_red, prosilica::Manual);
            if (camera_->hasAttribute("WhitebalValueRed"))
                camera_->setAttribute("WhitebalValueRed", (tPvUint32)config.whitebalance_red);
            if (camera_->hasAttribute("WhitebalValueBlue"))
                camera_->setAttribute("WhitebalValueBlue", (tPvUint32)config.whitebalance_blue);
        }

        // Binning configuration
        if (camera_->hasAttribute("BinningX"))
        {
            config.binning_x = std::min(config.binning_x, (int)max_binning_x);
            config.binning_y = std::min(config.binning_y, (int)max_binning_y);

            camera_->setBinning(config.binning_x, config.binning_y);
        }
        else if (config.binning_x > 1 || config.binning_y > 1)
        {
            NODELET_WARN("Binning not available for this camera.");
            config.binning_x = config.binning_y = 1;
        }

        // Region of interest configuration
        // Make sure ROI fits in image
        config.x_offset = std::min(config.x_offset, (int)sensor_width_ - 1);
        config.y_offset = std::min(config.y_offset, (int)sensor_height_ - 1);
        config.width  = std::min(config.width, (int)sensor_width_ - config.x_offset);
        config.height = std::min(config.height, (int)sensor_height_ - config.y_offset);
        // If width or height is 0, set it as large as possible
        int width  = config.width  ? config.width  : sensor_width_  - config.x_offset;
        int height = config.height ? config.height : sensor_height_ - config.y_offset;

        // Adjust full-res ROI to binning ROI
        /// @todo Replicating logic from polledCallback
        int x_offset = config.x_offset / config.binning_x;
        int y_offset = config.y_offset / config.binning_y;
        unsigned int right_x  = (config.x_offset + width  + config.binning_x - 1) / config.binning_x;
        unsigned int bottom_y = (config.y_offset + height + config.binning_y - 1) / config.binning_y;
        // Rounding up is bad when at max resolution which is not divisible by the amount of binning
        right_x = std::min(right_x, (unsigned)(sensor_width_ / config.binning_x));
        bottom_y = std::min(bottom_y, (unsigned)(sensor_height_ / config.binning_y));
        width = right_x - x_offset;
        height = bottom_y - y_offset;

        camera_->setRoi(x_offset, y_offset, width, height);

      // TF frame
      img_.header.frame_id = cam_info_.header.frame_id = config.frame_id;

        // Normally the node adjusts the bandwidth used by the camera during diagnostics, to use as
        // much as possible without dropping packets. But this can create interference if two
        // cameras are on the same switch, e.g. for stereo. So we allow the user to set the bandwidth
        // directly.
        auto_adjust_stream_bytes_per_second_ = config.auto_adjust_stream_bytes_per_second;
        if (!auto_adjust_stream_bytes_per_second_)
            camera_->setAttribute("StreamBytesPerSecond", (tPvUint32)config.stream_bytes_per_second);
        else
            camera_->setAttribute("StreamBytesPerSecond", (tPvUint32)(115000000/num_cameras));

        //! If exception thrown due to bad settings, it will fail to start camera
        //! Reload last good config
        if (level >= (uint32_t)driver_base::SensorLevels::RECONFIGURE_STOP)
        {
            try
            {
                start();
            }
            catch(std::exception &e)
            {
                NODELET_ERROR("Invalid settings: %s", e.what());
                config = last_config_;
            }
        }

        last_config_ = config;
    }

    void getCurrentState(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        stat.add("Serial", guid_);
        stat.add("Info", state_info_);
        stat.add("Intrinsics", intrinsics_);
        stat.add("Total frames dropped", frames_dropped_total_);
        stat.add("Total frames", frames_completed_total_);

        if(frames_completed_total_>0)
        {
            stat.add("Total % frames dropped", 100.*(double)frames_dropped_total_/frames_completed_total_);
        }
        if(frames_completed_acc_.sum()>0)
        {
            stat.add("Recent % frames dropped", 100.*frames_dropped_acc_.sum()/frames_completed_acc_.sum());
        }

        switch (camera_state_)
        {
            case OPENING:
                stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Opening camera");
                break;
            case OK:
                stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Camera operating normally");
                break;
            case CAMERA_NOT_FOUND:
                stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Can not find camera %d", guid_ );
                stat.add("Available Cameras", getAvailableCameras());
                break;
            case FORMAT_ERROR:
                stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Problem retrieving frame");
                break;
            case ERROR:
                stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Camera has encountered an error");
                break;
            default:
                break;
        }
    }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(prosilica_driver::ProsilicaNodelet, nodelet::Nodelet);

