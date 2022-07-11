#include "mineral_detect/mineral_detect.h"
namespace mineral_detect
{
    Detector::Detector() = default;

    void Detector::receiveFromCam(const sensor_msgs::ImageConstPtr &image) {
        cv_image_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
        cv::Mat origin_img = cv_image_->image.clone();
        publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", origin_img).toImageMsg());
    }

    void Detector::onInit() {
        ros::NodeHandle nh = getMTPrivateNodeHandle();
        subscriber_ = nh.subscribe("/usb_cam/image_raw", 1, &Detector::receiveFromCam, this);
        publisher_ = nh.advertise<sensor_msgs::Image>("image_publisher", 1);
    }

    Detector::~Detector()
    {
    }
}
PLUGINLIB_EXPORT_CLASS(mineral_detect::Detector, nodelet::Nodelet)
