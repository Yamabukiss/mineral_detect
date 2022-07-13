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
        cv::Mat gauss_img;
        cv::GaussianBlur(gray_img,gauss_img,cv::Size (3,3),0,0);
        cv::Mat canny_img;
        cv::Canny(gauss_img,canny_img,thresh1_,thresh2_,3,l2_gradient_);
        cv::Mat mor_img;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
        cv::morphologyEx(canny_img,mor_img,morph_type_,kernel,cv::Point(-1,-1),1);
        std::vector<cv::Vec4f> lines;
        cv::HoughLinesP(mor_img,lines,hough_rho_,hough_theta_,hough_thresh_,min_line_length_,max_line_gap_);
        mor_img=cv::Scalar::all(0);
        for(int i=0;i<lines.size();i++)
        {
            cv::Vec4f line=lines[i];
            cv::line(mor_img,cv::Point2f (line[0],line[1]),cv::Point2f (line[2],line[3]),cv::Scalar(255),1);
        }

        publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", mor_img).toImageMsg());
        }

        void Detector::dynamicCallback(mineral_detect::dynamicConfig &config) {
            thresh1_ = config.thresh1;
            thresh2_ = config.thresh2;
            hough_rho_ = config.rho;
            hough_theta_ = config.theta;
            l2_gradient_ = config.l2_gradient;
            morph_type_ = config.morph_type;
            hough_thresh_=config.hough_thresh;
            min_line_length_=config.min_line_length;
            max_line_gap_=config.max_line_gap;
        }

        Detector::~Detector()=default;
    }
PLUGINLIB_EXPORT_CLASS(mineral_detect::Detector, nodelet::Nodelet)
