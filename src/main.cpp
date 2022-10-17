#include "mineral_detect/mineral_detect.h"
/// the topic name is "/mineral_detect_node/point_publisher" ///
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
        gray_publisher_ = nh_.advertise<sensor_msgs::Image>("gray_publisher",1);
        point_publisher_=nh_.advertise<std_msgs::Float32MultiArray>("point_publisher",1);
        callback_ = boost::bind(&Detector::dynamicCallback, this, _1);
        server_.setCallback(callback_);
        flash_counter_=0;
    }

    void Detector::receiveFromCam(const sensor_msgs::ImageConstPtr &image) {
        cv_image_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
        cv::Mat origin_img = cv_image_->image.clone();
        cv::Mat gray_img;
        cv::cvtColor(origin_img,gray_img,CV_BGR2GRAY);
        gray_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_img).toImageMsg());
        cv::Mat hsv_img;
        cv::cvtColor(origin_img,hsv_img,cv::COLOR_BGR2HSV);
        cv::Mat bin_img;
        cv::inRange(hsv_img,cv::Scalar(lower_hsv_h_,lower_hsv_s_,lower_hsv_v_),cv::Scalar(upper_hsv_h_,upper_hsv_s_,upper_hsv_v_),bin_img);
        cv::Mat mor_img;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
        cv::morphologyEx(bin_img,mor_img,morph_type_,kernel,cv::Point(-1,-1),morph_iterations_);
        hsv_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", mor_img).toImageMsg());
        std::vector< std::vector< cv::Point> > contours;
        cv::findContours(mor_img,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
//        mor_img=cv::Scalar::all(0);
        cv::Point2f min_area_rect_points[4];
//        std::vector<float> point_x_vector;
//        std::vector<float> point_y_vector;
//        std::vector<cv::Rect> rect_vector;
        for (int i = 0; i < contours.size(); ++i)
        {
            cv::RotatedRect min_area_rect = cv::minAreaRect(cv::Mat(contours[i]));
            min_area_rect.points(min_area_rect_points);
            cv::Rect rect_to_color_select(min_area_rect_points[0],min_area_rect_points[2]);
            if(chooseRect(rect_to_color_select))
            {
                    if (rect_to_color_select.tl().x<0) continue;
                    if (rect_to_color_select.br().x>cv_image_->image.cols-1) continue;
                    if (rect_to_color_select.tl().y<0) continue;
                    if (rect_to_color_select.br().y>cv_image_->image.rows-1) continue;
                    if(rectColorChoose(rect_to_color_select))
                    {
                        if(cv::arcLength(contours[i], true)/cv::contourArea(contours[i])>=min_perimeter_area_ratio_&&cv::arcLength(contours[i], true)/cv::contourArea(contours[i])<=max_perimeter_area_ratio_)
                        {
                                cv::Rect compentioned_rect=middlePointCompention(rect_to_color_select);
                                cv::Point2i middle_point(compentioned_rect.x+compentioned_rect.width/2,compentioned_rect.y+compentioned_rect.height/2);
                                if(flashProcess(middle_point,compentioned_rect,gray_img))
                                {
                                    std_msgs::Float32MultiArray middle_point_array;
                                    middle_point_array.data.push_back((float)middle_point.x);
                                    middle_point_array.data.push_back((float)middle_point.y);
                                    point_publisher_.publish(middle_point_array);
                                }
                                std::cout<<flash_counter_<<std::endl;
                        } else continue;
                } else continue;
            } else continue;
        }

        test_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), cv_image_->encoding, cv_image_->image).toImageMsg());
        }

    bool Detector::flashProcess(cv::Point2i &mineral_point,cv::Rect &mineral,cv::Mat &gray_img)
    {
        int light_rect_width=int(light_rect_width_ratio_*float(mineral.width));
        int light_rect_height=int(light_rect_height_ratio_*float(mineral.height));
        int light_x=mineral.x+mineral.width/2;
        int light_y=mineral_point.y-int(box_light_ratio_*float((mineral_point.y-mineral.y)));
        if (light_y<0) light_y=0;
        cv::Point2i  light_middle_point=cv::Point2i (light_x,light_y);
//        cv::Rect light_rect;
//        light_rect.width=light_rect_width;
//        light_rect.height=light_rect_height;
//        light_rect.x=int(light_middle_point.x-light_rect_width/2);
//        light_rect.y=int(light_middle_point.y-light_rect_height/2);
//        cv::rectangle(cv_image_->image,light_rect,cv::Scalar(255,0,255),3);
        cv::circle(cv_image_->image, light_middle_point, 2, cv::Scalar(255,0,255), 5);
        cv::rectangle(cv_image_->image,mineral,cv::Scalar(255,0,255),3);
        cv::circle(cv_image_->image, mineral_point, 2, cv::Scalar(255,0,255), 5);

        cv::Point2i arrow_base (mineral_point.x,mineral_point.y-int(arrow_base_ratio_*(mineral_point.y-light_middle_point.y)));
        cv::line(cv_image_->image,light_middle_point,arrow_base,cv::Scalar(255,0,255),6);

        cv::Point2i arrow_branch_base(mineral_point.x,int(arrow_branch_ratio_base_*arrow_base.y));
        cv::Point2i arrow_left(mineral_point.x-int(arrow_branch_ratio_base_x_*arrow_base.y),arrow_branch_base.y);
        cv::Point2i arrow_right(mineral_point.x+int(arrow_branch_ratio_base_x_*arrow_base.y),arrow_branch_base.y);

        cv::line(cv_image_->image,arrow_left,arrow_base,cv::Scalar(255,0,255),6);
        cv::line(cv_image_->image,arrow_right,arrow_base,cv::Scalar(255,0,255),6);

        int avg_pixel;
//        avg_pixel=calculateGrayValue(light_rect,gray_img);
        avg_pixel=gray_img.at<uchar>(light_middle_point);
        std::cout<<"the light average gray pixel is :"<<abs(avg_pixel)<<std::endl; // 9 ~ 13
        if (abs(avg_pixel)<dark_thresh_)
        {
            flash_counter_+=1.2;
//            cv::rectangle(cv_image_->image,light_rect,cv::Scalar(0,0,255),3);
            cv::circle(cv_image_->image, light_middle_point, 2, cv::Scalar(0,0,255), 5);
            cv::rectangle(cv_image_->image,mineral,cv::Scalar(0,0,255),3);
            cv::circle(cv_image_->image, mineral_point, 2, cv::Scalar(0,0,255), 5);
            cv::line(cv_image_->image,light_middle_point,arrow_base,cv::Scalar(0,0,255),6);
            cv::line(cv_image_->image,arrow_left,arrow_base,cv::Scalar(0,0,255),6);
            cv::line(cv_image_->image,arrow_right,arrow_base,cv::Scalar(0,0,255),6);

            return true;
        }
        else
        {
            if(flash_counter_>0)
            {
                flash_counter_ -= 1;
//                    cv::rectangle(cv_image_->image,light_rect,cv::Scalar(0,0,255),3);
                cv::circle(cv_image_->image, light_middle_point, 10, cv::Scalar(0, 0, 255), 5);
                cv::rectangle(cv_image_->image, mineral, cv::Scalar(0, 0, 255), 3);
                cv::circle(cv_image_->image, mineral_point, 10, cv::Scalar(0, 0, 255), 5);
                cv::line(cv_image_->image, light_middle_point, arrow_base, cv::Scalar(0, 0, 255), 6);
                cv::line(cv_image_->image, arrow_left, arrow_base, cv::Scalar(0, 0, 255), 6);
                cv::line(cv_image_->image, arrow_right, arrow_base, cv::Scalar(0, 0, 255), 6);
                return true;
            }
                if (flash_counter_<0)
                {
                    flash_counter_=0;
                    return false;
                }
            }

    }


    int Detector::calculateGrayValue(cv::Rect & rect,cv::Mat &gray_img)
    {
        int sum_pixel=0;
        int max_pixel=0;
        for (int i=rect.x;i<rect.x+rect.width;i++)
        {
            for (int j = rect.y; j < rect.y + rect.height; j++)
            {
                if(max_pixel<gray_img.at<uchar>(i,j)) max_pixel=gray_img.at<uchar>(i,j);
            }
        }

//        int avg_pixel=int(sum_pixel/rect.area());

        return max_pixel;
    }

    bool Detector::chooseRect(const cv::Rect &rect)
        {
            if(rect.area()>=min_area_thresh_)
            {
                        return true;
            }
            return false;
        }

//        cv::Point2f Detector::getMiddlePoint(const std::vector<float> &point_x_vector,const std::vector<float> &point_y_vector,std::vector<cv::Point2f> &coordinate_vec2d)
//        {
//            float biggest_x = *std::max_element(std::begin(point_x_vector), std::end(point_x_vector));
//            float biggest_y=*std::max_element(std::begin(point_y_vector),std::end(point_y_vector));
//            float smallest_x=*std::min_element(std::begin(point_x_vector), std::end(point_x_vector));
//            float smallest_y=*std::min_element(std::begin(point_y_vector),std::end(point_y_vector));
//
//            std::vector<cv::Point2f> corner_point2d_vec;
//            corner_point2d_vec.emplace_back(smallest_x,smallest_y);
//            corner_point2d_vec.emplace_back(biggest_x,smallest_y);
//            corner_point2d_vec.emplace_back(biggest_x,biggest_y);
//            corner_point2d_vec.emplace_back(smallest_x,biggest_y);
//
//            std::vector<cv::Point3f> corner_point3d_vec;
//            corner_point3d_vec.emplace_back(0,0,150);
//            corner_point3d_vec.emplace_back(0,150,150);
//            corner_point3d_vec.emplace_back(0,150,0);
//            corner_point3d_vec.emplace_back(0,0,0);
//            cv::solvePnP(corner_point3d_vec,corner_point2d_vec,camera_matrix_,distortion_coefficients_,rvec_,tvec_);
//
//            std::vector<cv::Point3f> coordinate_vec3d;
//            coordinate_vec3d.emplace_back(0, 75, 75);
//            coordinate_vec3d.emplace_back(50, 75, 75);
//            coordinate_vec3d.emplace_back(0, 125, 75);
//            coordinate_vec3d.emplace_back(0, 75, 125);
//            cv::projectPoints(coordinate_vec3d,rvec_,tvec_,camera_matrix_,distortion_coefficients_,coordinate_vec2d);
//
//            float middle_x=smallest_x+(biggest_x-smallest_x)/2;
//            float middle_y=smallest_y+(biggest_y-smallest_y)/2;
//
//            return {middle_x,middle_y};
//        }

    bool Detector::rectColorChoose(const cv::Rect &rect)
    {
        cv::Mat roi_rect = cv_image_->image(rect);
        cv::Mat roi_hsv_rect;
        if(!roi_rect.empty())
        {
            cv::cvtColor(roi_rect,roi_hsv_rect,cv::COLOR_BGR2HSV);
            cv::Mat roi_binary_rect;
            cv::inRange(roi_hsv_rect,cv::Scalar(lower_hsv_h_,lower_hsv_s_,lower_hsv_v_),cv::Scalar(upper_hsv_h_,upper_hsv_s_,upper_hsv_v_),roi_binary_rect);
            int num_non_zero_pixel=cv::countNonZero(roi_binary_rect);
            if(num_non_zero_pixel>0)
            {
                double roi_percent=(double)num_non_zero_pixel/rect.area();
                if(roi_percent>roi_nonzero_percent_) return true;
                else return false;
            }
            else return false;
        }
        else return false;
    }

    cv::Rect Detector::middlePointCompention(const cv::Rect &rect)
    {
        int bias=rect.height-rect.width;
        cv::Rect new_rect(rect.x,rect.y,rect.width,rect.height-bias);
        if(bias>=shape_bias_)   return new_rect;
        else return rect;
    }

    void Detector::dynamicCallback(mineral_detect::dynamicConfig &config) {
            morph_type_ = config.morph_type;
            morph_iterations_ = config.morph_iterations;
            min_area_thresh_=config.min_area_thresh;
            lower_hsv_h_=config.lower_hsv_h;
            lower_hsv_s_=config.lower_hsv_s;
            lower_hsv_v_=config.lower_hsv_v;
            upper_hsv_h_=config.upper_hsv_h;
            upper_hsv_s_=config.upper_hsv_s;
            upper_hsv_v_=config.upper_hsv_v;
            roi_nonzero_percent_=config.roi_nonzero_percent;
            min_perimeter_area_ratio_=config.min_perimeter_area_ratio;
            max_perimeter_area_ratio_=config.max_perimeter_area_ratio;
            shape_bias_=config.shape_bias;
            box_light_ratio_=config.box_light_ratio;
            light_rect_width_ratio_=config.light_rect_width_ratio;
            light_rect_height_ratio_=config.light_rect_height_ratio;
            dark_thresh_=config.dark_thresh;
//            flash_counter_thresh_=config.flash_counter_thresh;
            arrow_base_ratio_=config.arrow_base_ratio;
            arrow_branch_ratio_base_=config.arrow_branch_ratio_base;
            arrow_branch_ratio_base_x_=config.arrow_branch_ratio_base_x;
        }

        Detector::~Detector()=default;
    }
PLUGINLIB_EXPORT_CLASS(mineral_detect::Detector, nodelet::Nodelet)
