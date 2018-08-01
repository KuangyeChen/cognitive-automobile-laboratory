#include "label_dilation.hpp"

#include <cv_bridge/cv_bridge.h>

namespace image_preproc_ros_tool {

LabelDilation::LabelDilation(ros::NodeHandle nh_public, ros::NodeHandle nh_private)
        : interface_{nh_private}, reconfigureServer_{nh_private} {

    /**
     * Initialization
     */
    interface_.fromParamServer();


    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/LabelDilation.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&LabelDilation::reconfigureRequest, this, _1, _2));
    interface_.subscriber->registerCallback(&LabelDilation::callbackSubscriber, this);

    rosinterface_handler::showNodeInfo();
}

namespace {
std::set<int> getLabels() {
    std::set<int> outlier_labels{0, 1, 2, 3, 5, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, -1};
    return outlier_labels;
}
}

void LabelDilation::callbackSubscriber(const Msg::ConstPtr& msg) {
    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

    // publish faulty image
    if (img->image.rows == 0 || img->image.cols == 0) {
        ROS_WARN_STREAM("In LabelDilation: image invalid, republish input data");
        try {
            interface_.publisher.publish(img->toImageMsg());
            return;
        } catch (const cv_bridge::Exception& e) {
            ROS_WARN_STREAM("In LabelDilation: " << e.what());
        }
    }

    for (const auto& label : getLabels()) {
        // threshold image
        cv::Mat mask = (img->image == label);

        // create kernel
        cv::Mat element = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(2 * interface_.half_kernel_size + 1, 2 * interface_.half_kernel_size + 1),
            cv::Point(interface_.half_kernel_size, interface_.half_kernel_size));

        // do erosion or dilation
        if (interface_.erode) {
            cv::erode(mask, mask, element);
            // cv::morphologyEx( mask, mask, 3, element );
        } else {
            cv::dilate(mask, mask, element);
            // cv::morphologyEx( mask, mask, 2, element );
        }

        // apply mask with labels to img
        cv::Mat new_labels = mask * label;
        new_labels.copyTo(img->image, mask);
    }

    interface_.publisher.publish(img->toImageMsg());
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void LabelDilation::reconfigureRequest(const ReconfigureConfig& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace image_preproc_ros_tool
