#pragma once

#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"
#include <algorithm>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>
#include "dynamic_reconfigure/server.h"
#include "mineral_detect/dynamicConfig.h"
#include "std_msgs/Float32MultiArray.h"
namespace mineral_detect
{
    class Detector : public nodelet::Nodelet {
    public:
        Detector();

        virtual ~Detector();

        void onInit() override;

        void receiveFromCam(const sensor_msgs::ImageConstPtr &image);

        void dynamicCallback(mineral_detect::dynamicConfig& config);

        cv::Point2i targetPointSelect(std::vector<cv::Point2i> &points_vec,std::vector<cv::Rect> &rect_vec,std::vector<cv::Point2i> &text_vec);

        bool chooseRect(const cv::Rect &rect);

        bool rectColorChoose(const cv::Rect &rect);

        cv_bridge::CvImagePtr cv_image_;
        int morph_type_;
        int morph_iterations_;
        int lower_hsv_h_;
        int lower_hsv_s_;
        int lower_hsv_v_;
        int upper_hsv_h_;
        int upper_hsv_s_;
        int upper_hsv_v_;
        double min_perimeter_area_ratio_;
        double max_perimeter_area_ratio_;
        double min_area_thresh_;
        double roi_nonzero_percent_;
        double shape_bias_;
        double x_scale_;
        double y_scale_;
        double text_size_;
        double y_bias_;

        cv::Mat camera_matrix_;
        cv::Mat distortion_coefficients_;
        cv::Mat rvec_;
        cv::Mat tvec_;
        dynamic_reconfigure::Server<mineral_detect::dynamicConfig> server_;
        dynamic_reconfigure::Server<mineral_detect::dynamicConfig>::CallbackType callback_;
        ros::NodeHandle nh_;
        ros::Publisher test_publisher_;
        ros::Publisher hsv_publisher_;
        ros::Publisher point_publisher_;
        ros::Publisher gray_publisher_;
        ros::Subscriber subscriber_;
    };
}
