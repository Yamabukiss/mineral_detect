#include "mineral_detect/mineral_detect.h"
/// the topic name is "/mineral_detect_node/point_publisher" ///
int num=1;
namespace mineral_detect {
    Detector::Detector() = default;

    void Detector::onInit() {
        nh_ = getMTPrivateNodeHandle();

//        XmlRpc::XmlRpcValue distortion_param_list;
//        std::vector<double> distortion_vec;
//        if(ros::param::get("/distortion_coefficients/data",distortion_param_list))
//        {
//            for (int i = 0; i < distortion_param_list.size(); ++i)
//            {
//                XmlRpc::XmlRpcValue tmp_value = distortion_param_list[i];
//                if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
//                    distortion_vec.push_back(double(tmp_value));
//            }
//
//            distortion_coefficients_ =(cv::Mat_<double>(1,5)<<distortion_vec[0],distortion_vec[1],distortion_vec[2],distortion_vec[3],distortion_vec[4]);
//
//        }
//
//        XmlRpc::XmlRpcValue camera_param_list;
//        std::vector<double> camera_vec;
//        if(ros::param::get("/camera_matrix/data",camera_param_list))
//        {
//            for (int i = 0; i < camera_param_list.size(); ++i)
//            {
//                XmlRpc::XmlRpcValue tmp_value = camera_param_list[i];
//                if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
//                    camera_vec.push_back(double(tmp_value));
//            }
//
//            camera_matrix_ =(cv::Mat_<double>(3,3)<<camera_vec[0],camera_vec[1],camera_vec[2],camera_vec[3],camera_vec[4],camera_vec[5],camera_vec[6],camera_vec[7],camera_vec[8]);
//
//        }

        subscriber_ = nh_.subscribe("/usb_cam/image_raw", 1, &Detector::receiveFromCam, this);
        test_publisher_ = nh_.advertise<sensor_msgs::Image>("test_publisher", 1);
        hsv_publisher_ = nh_.advertise<sensor_msgs::Image>("hsv_publisher", 1);
        mask_publisher_ = nh_.advertise<sensor_msgs::Image>("mask_publisher",1);
        point_publisher_=nh_.advertise<std_msgs::Float32MultiArray>("point_publisher",1);
        callback_ = boost::bind(&Detector::dynamicCallback, this, _1);
        server_.setCallback(callback_);

        cv::Mat r_img=cv::imread("/home/yamabuki/Downloads/R.jpg");
        cv::cvtColor(r_img,r_img,CV_BGR2GRAY);
        cv::threshold(r_img,r_img,255,255,CV_THRESH_OTSU);
        std::vector< std::vector< cv::Point> > contours;
        cv::findContours(r_img,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
        std::vector<cv::Point2i> hull_back;
        cv::convexHull( contours[0],hull_back, true);
        auto moment = cv::moments(hull_back);
        double hu_moment[7];
        cv::HuMoments(moment,hu_moment);
        std::cout<<hu_moment[0]<<std::endl;
        back_humoment_=hu_moment[0];
    }

    void Detector::receiveFromCam(const sensor_msgs::ImageConstPtr &image) {
        cv_image_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
        cv::Mat origin_img = cv_image_->image.clone();
        cv::Mat gray_img;
        cv::cvtColor(origin_img,gray_img,CV_BGR2GRAY);
        cv::Mat hsv_img;
        cv::cvtColor(origin_img,hsv_img,cv::COLOR_BGR2HSV);
        cv::Mat bin_img;
        cv::inRange(hsv_img,cv::Scalar(lower_hsv_h_,lower_hsv_s_,lower_hsv_v_),cv::Scalar(upper_hsv_h_,upper_hsv_s_,upper_hsv_v_),bin_img);
        cv::Mat mor_img;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
        cv::morphologyEx(bin_img,mor_img,morph_type_,kernel,cv::Point(-1,-1),morph_iterations_);
        hsv_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", mor_img).toImageMsg());
        std::vector< std::vector< cv::Point> > contours;
        std::vector< std::vector< cv::Point> > mask_contours;
        cv::findContours(mor_img,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
        std_msgs::Float32MultiArray middle_point_array;
        std::vector<cv::Point2i> middle_points_vec;
        for (int i = 0; i < contours.size(); ++i)
        {
            std::vector<cv::Point2i> hull;
            cv::convexHull( contours[i],hull, true);
                    if(cv::contourArea(hull)>min_area_thresh_)
                    {
                        cv::Mat mask=cv::Mat::zeros(cv_image_->image.rows,cv_image_->image.cols,CV_8UC1);
                        cv::fillConvexPoly(mask,hull,cv::Scalar(255));
                        mask=mask-mor_img;
//                        cv::bitwise_xor(mor_img,mask,mask);
                        mask_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg());
                        cv::findContours(mask,mask_contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
                        for (int j = 0; j < mask_contours.size(); ++j)
                        {
                            std::vector<cv::Point2i> hull2;
                            cv::convexHull( mask_contours[j],hull2, true);
                            if(cv::contourArea(hull2)>=min_area_r_thresh_ && cv::contourArea(hull2)<=max_area_r_thresh_)
                            {
                                auto moment = cv::moments(hull2);
                                double hu_moment[7];
                                cv::HuMoments(moment,hu_moment);
                                if(hu_moment[0]<=back_humoment_*hu_moment_max_ && hu_moment[0]>=back_humoment_*hu_moment_min_)
                                {
                                    int cx = int(moment.m10 / moment.m00);
                                    int cy = int(moment.m01/  moment.m00);
                                    cv::Point2i middle_point (cx,cy);
                                    cv::polylines(cv_image_->image,hull2, true,cv::Scalar(0,199,140),7);
                                    cv::circle(cv_image_->image, middle_point, 3, cv::Scalar(0,199,140), 7);
                                    middle_points_vec.push_back(middle_point);
                                }
                            }
                        }
                } else continue;
        }
        if(!middle_points_vec.empty())
        {
            cv::Point2i target_point= targetPointSelect(middle_points_vec);
            middle_point_array.data.push_back((float)target_point.x);
            middle_point_array.data.push_back((float)target_point.y);
            point_publisher_.publish(middle_point_array);
        }

        test_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), cv_image_->encoding, cv_image_->image).toImageMsg());
        }
    cv::Point2i Detector::targetPointSelect(std::vector<cv::Point2i> &points_vec)
    {
        cv::Point2i img_center (int(cv_image_->image.cols/2)-1,int(cv_image_->image.rows/2)-1);
        int min_x=cv_image_->image.cols;
        cv::Point2i min_bias_point;
        for (auto i=points_vec.begin();i<points_vec.end();i++)
        {
            int bias= abs(i->x-img_center.x);
            if(bias<min_x)
            {
                min_x=bias;
                min_bias_point=*i;
            }
        }
        auto min_point_iter=std::find(points_vec.begin(), points_vec.end(),min_bias_point);
        int min_point_index=std::distance(points_vec.begin(),min_point_iter);
        cv::Point2i target_point=points_vec[min_point_index];
        cv::circle(cv_image_->image, target_point, 2, cv::Scalar(153,51,250), 7);
        return target_point;
    }

    void Detector::dynamicCallback(mineral_detect::dynamicConfig &config) {
        morph_type_ = config.morph_type;
        morph_iterations_ = config.morph_iterations;
        min_area_thresh_=config.min_area_thresh;
        min_area_r_thresh_=config.min_area_r_thresh;
        max_area_r_thresh_=config.max_area_r_thresh;
        lower_hsv_h_=config.lower_hsv_h;
        lower_hsv_s_=config.lower_hsv_s;
        lower_hsv_v_=config.lower_hsv_v;
        upper_hsv_h_=config.upper_hsv_h;
        upper_hsv_s_=config.upper_hsv_s;
        upper_hsv_v_=config.upper_hsv_v;
        hu_moment_min_=config.hu_moment_min;
        hu_moment_max_=config.hu_moment_max;
        }

        Detector::~Detector()=default;
    }
PLUGINLIB_EXPORT_CLASS(mineral_detect::Detector, nodelet::Nodelet)
