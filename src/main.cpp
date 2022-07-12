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
//        cv::Mat thresh_img;
//        cv::threshold(gray_img,thresh_img,thresh_,255,thresh_type_);
        // Harris Corner
        cv::Mat harris_dst, norm_harris_dst, norm_abs_harris_dst;
        harris_dst = cv::Mat::zeros(gray_img.size(), CV_32FC1);
        cv::cornerHarris(gray_img, harris_dst, block_size_, 3, r_alpha_, cv::BORDER_DEFAULT);
        cv::normalize(harris_dst, norm_harris_dst, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
        cv::convertScaleAbs(norm_harris_dst, norm_abs_harris_dst);

        for (int i = 0; i < norm_harris_dst.rows; i++)
        {
            for (int j = 0; j < norm_harris_dst.cols; j++)
            {
                if ((int)norm_harris_dst.at<float>(i, j) > harris_thresh_)
                {
                    circle(norm_abs_harris_dst, cv::Point(j, i), 10, cv::Scalar(255, 255, 255), 2, 8, 0);
                }
            }
        }

        publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", norm_abs_harris_dst).toImageMsg());
        }

        void Detector::dynamicCallback(mineral_detect::dynamicConfig &config) {
            thresh_ = config.thresh;
            thresh_type_ = config.thresh_type;
            harris_thresh_=config.harris_thresh;
            block_size_ = config.block_size;
            r_alpha_ = config.r_alpha;
        }

        Detector::~Detector()=default;
    }
PLUGINLIB_EXPORT_CLASS(mineral_detect::Detector, nodelet::Nodelet)
