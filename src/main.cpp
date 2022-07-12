#include "mineral_detect/mineral_detect.h"
namespace mineral_detect {
    Detector::Detector() = default;

    void Detector::onInit() {
        nh_ = getMTPrivateNodeHandle();
        subscriber_ = nh_.subscribe("/usb_cam/image_raw", 1, &Detector::receiveFromCam, this);
        publisher_ = nh_.advertise<sensor_msgs::Image>("image_publisher", 1);
        callback_ = boost::bind(&Detector::dynamicCallback, this, _1);
        server_.setCallback(callback_);
    }

    void Detector::receiveFromCam(const sensor_msgs::ImageConstPtr &image) {
        cv_image_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
        cv::Mat origin_img = cv_image_->image.clone();
        cv::Mat gray_img;
        cv::cvtColor(origin_img, gray_img, CV_BGR2GRAY);
        cv::Mat thresh_img;
        cv::threshold(gray_img,thresh_img,thresh_,255,thresh_type_);
        cv::Mat gauss_img;
        cv::GaussianBlur(thresh_img,gauss_img,cv::Size (3,3),0,0);
        cv::Mat mor_img;
        cv::morphologyEx(gauss_img,mor_img,morph_type_,3,cv::Point(-1,-1),morph_iterations_);
        std::vector< std::vector< cv::Point> > contours;
        cv::findContours(mor_img,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
        mor_img=cv::Scalar::all(0);
//        cv::drawContours(mor_img, contours, -1, cv::Scalar::all(255));
        cv::Point2f min_area_rect_points[4];
        for (int i = 0; i < contours.size(); ++i)
        {
            cv::RotatedRect min_area_rect = cv::minAreaRect(cv::Mat(contours[i]));
            min_area_rect.points(min_area_rect_points);
            for( int i = 0; i < 4; i++ ){
                cv::line(mor_img, min_area_rect_points[i], min_area_rect_points[(i+1)%4],cv::Scalar(100, 200, 211), 2, cv::LINE_AA);
            }
        }
        publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", mor_img).toImageMsg());
        }

        void Detector::dynamicCallback(mineral_detect::dynamicConfig &config) {
            thresh_ = config.thresh;
            thresh_type_ = config.thresh_type;
            morph_type_ = config.morph_type;
            morph_iterations_ = config.morph_iterations;
        }

        Detector::~Detector()=default;
    }
PLUGINLIB_EXPORT_CLASS(mineral_detect::Detector, nodelet::Nodelet)
