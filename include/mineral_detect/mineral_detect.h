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
namespace mineral_detect
{
    class Detector : public nodelet::Nodelet {
    public:
        Detector();

        virtual ~Detector();

        void onInit() override;

        void receiveFromCam(const sensor_msgs::ImageConstPtr &image);

        void dynamicCallback(mineral_detect::dynamicConfig& config);

        bool chooseRect(const cv::Point2f &point1, const cv::Point2f &point2,const cv::Point2f &point3);

        bool rectColorChoose(const cv::Rect &rect);

        cv::Point2f getMiddlePoint(const std::vector<float> &point_x_vector,const std::vector<float> &point_y_vector,std::vector<cv::Point2f> &coordinate_vec2d);

        void getDirection(const std::vector<cv::Point2f> &rect_middle_points_vector,const std::vector<cv::Rect> &rect_vector);

        cv_bridge::CvImagePtr cv_image_;
        int morph_type_;
        int morph_iterations_;
        int thresh1_;
        int thresh2_;
        int lower_hsv_h_;
        int lower_hsv_s_;
        int lower_hsv_v_;
        int upper_hsv_h_;
        int upper_hsv_s_;
        int upper_hsv_v_;
        double contours_ratio_;
        double k_bias_;
        double length_bias_;
        double min_area_thresh_;
        double max_area_thresh_;
        double roi_nonzero_percent_;
        float biggest_center_point_x_;
        float biggest_center_point_y_;
        float smallest_center_point_x_;
        float smallest_center_point_y_;
        cv::Mat camera_matrix_;
        cv::Mat distortion_coefficients_;
        cv::Mat rvec_;
        cv::Mat tvec_;
        dynamic_reconfigure::Server<mineral_detect::dynamicConfig> server_;
        dynamic_reconfigure::Server<mineral_detect::dynamicConfig>::CallbackType callback_;

        ros::NodeHandle nh_;
        ros::Publisher binary_publisher_;
        ros::Publisher hsv_publisher_;
        ros::Publisher direction_publisher_;
        ros::Subscriber subscriber_;
    };
}
