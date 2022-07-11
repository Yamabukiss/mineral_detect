#include "mineral_detect/mineral_detect.h"
namespace mineral_detect
{
    Detector::Detector() = default;

    void Detector::receiveFromCam(const sensor_msgs::ImageConstPtr &image) {
        cv_image_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
        cv::Mat origin_img = cv_image_->image.clone();
        cv::cvtColor(origin_img,gray_img_,CV_BGR2GRAY);
        cv::threshold(gray_img_,thresh_img_,120,255,CV_THRESH_OTSU);
        publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", origin_img).toImageMsg());
    }

    void Detector::onInit() {
        ros::NodeHandle nh = getMTPrivateNodeHandle();
        subscriber_ = nh.subscribe("/usb_cam/image_raw", 1, &Detector::receiveFromCam, this);
        publisher_ = nh.advertise<sensor_msgs::Image>("image_publisher", 1);
        dynamic_reconfigure::Server<mineral_detect::dynamicConfig> server;
        dynamic_reconfigure::Server<mineral_detect::dynamicConfig>::CallbackType callback;
//        callback = boost::bind(&Detector::dynamicCallback, this, _1);
//        server.setCallback(callback);
    }

    Detector::~Detector()
    {
    }
}
PLUGINLIB_EXPORT_CLASS(mineral_detect::Detector, nodelet::Nodelet)
