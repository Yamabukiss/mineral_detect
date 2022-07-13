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
        cv::Canny(gauss_img,canny_img,thresh1_,thresh2_,3, true);
        cv::Mat mor_img;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
        cv::morphologyEx(canny_img,mor_img,morph_type_,kernel,cv::Point(-1,-1),morph_iterations_);
        std::vector< std::vector< cv::Point> > contours;
        cv::findContours(mor_img,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
        mor_img=cv::Scalar::all(0);
        cv::Point2f min_area_rect_points[4];
        for (int i = 0; i < contours.size(); ++i)
        {
            cv::RotatedRect min_area_rect = cv::minAreaRect(cv::Mat(contours[i]));
            min_area_rect.points(min_area_rect_points);
            for( int  j= 0; j < 4; j++ ){
                if(chooseRect(min_area_rect_points[0],min_area_rect_points[2]))
                {
                    cv::line(mor_img, min_area_rect_points[j], min_area_rect_points[(j + 1) % 4], cv::Scalar(255), 2,cv::LINE_AA);
                }
                else break;
            }
        }
        publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", mor_img).toImageMsg());
        }

        bool Detector::chooseRect(const cv::Point2f &point1, const cv::Point2f &point2)
        {
            double subtract_x=abs(point1.x-point2.x);
            double subtract_y= abs(point1.y-point2.y);
            double k= abs(subtract_y/subtract_x);
            if(k<=1+k_bias_&&k>=1-k_bias_) {
                if (subtract_x <= subtract_y + length_bias_ && subtract_x >= subtract_y - length_bias_) return true;
            }
                else return false;
        }

        void Detector::dynamicCallback(mineral_detect::dynamicConfig &config) {
            morph_type_ = config.morph_type;
            morph_iterations_ = config.morph_iterations;
            thresh1_ = config.canny_thresh1;
            thresh2_ = config.canny_thresh2;
            k_bias_=config.k_bias;
            length_bias_=config.length_bias;
        }

        Detector::~Detector()=default;
    }
PLUGINLIB_EXPORT_CLASS(mineral_detect::Detector, nodelet::Nodelet)
