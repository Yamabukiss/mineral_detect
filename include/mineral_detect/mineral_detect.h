#pragma once

#include <sensor_msgs/Image.h>
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

        bool chooseRect(const cv::Point2f &point1,const cv::Point2f &point2);

        cv_bridge::CvImagePtr cv_image_;

        int morph_type_;
        int morph_iterations_;
        int thresh1_;
        int thresh2_;
        double k_bias_;
        double length_bias_;
        double min_area_thresh_;
        double max_area_thresh_;
        dynamic_reconfigure::Server<mineral_detect::dynamicConfig> server_;
        dynamic_reconfigure::Server<mineral_detect::dynamicConfig>::CallbackType callback_;

        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        ros::Subscriber subscriber_;
    };
}
