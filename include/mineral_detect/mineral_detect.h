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


        cv_bridge::CvImagePtr cv_image_;

        int thresh1_;
        int thresh2_;
        bool l2_gradient_;
        int hough_thresh_;
        int hough_rho_;
        int hough_theta_;
        double min_line_length_;
        double max_line_gap_;
        int morph_type_;

        dynamic_reconfigure::Server<mineral_detect::dynamicConfig> server_;
        dynamic_reconfigure::Server<mineral_detect::dynamicConfig>::CallbackType callback_;

        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        ros::Subscriber subscriber_;
    };
}
