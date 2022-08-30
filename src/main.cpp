#include "mineral_detect/mineral_detect.h"
namespace mineral_detect {
    Detector::Detector() = default;

    void Detector::onInit() {
        nh_ = getMTPrivateNodeHandle();

        XmlRpc::XmlRpcValue distortion_param_list;
        std::vector<double> distortion_vec;
        if(ros::param::get("/distortion_coefficients/data",distortion_param_list))
        {
            for (int i = 0; i < distortion_param_list.size(); ++i)
            {
                XmlRpc::XmlRpcValue tmp_value = distortion_param_list[i];
                if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    distortion_vec.push_back(double(tmp_value));
            }

            distortion_coefficients_ =(cv::Mat_<double>(1,5)<<distortion_vec[0],distortion_vec[1],distortion_vec[2],distortion_vec[3],distortion_vec[4]);

        }

        XmlRpc::XmlRpcValue camera_param_list;
        std::vector<double> camera_vec;
        if(ros::param::get("/camera_matrix/data",camera_param_list))
        {
            for (int i = 0; i < camera_param_list.size(); ++i)
            {
                XmlRpc::XmlRpcValue tmp_value = camera_param_list[i];
                if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    camera_vec.push_back(double(tmp_value));
            }

            camera_matrix_ =(cv::Mat_<double>(3,3)<<camera_vec[0],camera_vec[1],camera_vec[2],camera_vec[3],camera_vec[4],camera_vec[5],camera_vec[6],camera_vec[7],camera_vec[8]);

        }

        subscriber_ = nh_.subscribe("/usb_cam/image_raw", 1, &Detector::receiveFromCam, this);
        binary_publisher_ = nh_.advertise<sensor_msgs::Image>("binary_publisher", 1);
        hsv_publisher_ = nh_.advertise<sensor_msgs::Image>("hsv_publisher", 1);
        direction_publisher_=nh_.advertise<std_msgs::String>("direction_publisher",1);
        callback_ = boost::bind(&Detector::dynamicCallback, this, _1);
        server_.setCallback(callback_);
    }

    void Detector::receiveFromCam(const sensor_msgs::ImageConstPtr &image) {
        cv_image_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
        cv::Mat origin_img = cv_image_->image.clone();
        cv::Mat hsv_img;
        cv::cvtColor(origin_img,hsv_img,cv::COLOR_BGR2HSV);
        cv::Mat bin_img;
        cv::inRange(hsv_img,cv::Scalar(lower_hsv_h_,lower_hsv_s_,lower_hsv_v_),cv::Scalar(upper_hsv_h_,upper_hsv_s_,upper_hsv_v_),bin_img);
        hsv_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", bin_img).toImageMsg());
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
        std::vector<float> point_x_vector;
        std::vector<float> point_y_vector;
        std::vector<cv::Point2f> rect_middle_points_vector;
        std::vector<cv::Rect> rect_vector;
        for (int i = 0; i < contours.size(); ++i)
        {
            cv::RotatedRect min_area_rect = cv::minAreaRect(cv::Mat(contours[i]));
            min_area_rect.points(min_area_rect_points);

                if(chooseRect(min_area_rect_points[0],min_area_rect_points[1],min_area_rect_points[3]))
                {
                    cv::Rect rect_to_color_select(min_area_rect_points[0],min_area_rect_points[2]);
                    if (rect_to_color_select.tl().x<0) rect_to_color_select.x=0;
                    else if (rect_to_color_select.br().x>cv_image_->image.cols-1) rect_to_color_select.x=cv_image_->image.cols-1-rect_to_color_select.width;
                    if (rect_to_color_select.tl().y<0) rect_to_color_select.y=0;
                    else if (rect_to_color_select.br().y>cv_image_->image.rows-1) rect_to_color_select.y=cv_image_->image.rows-1-rect_to_color_select.height;

                    if(rectColorChoose(rect_to_color_select))
                    {
                        if(cv::contourArea(contours[i])/rect_to_color_select.area()>=contours_ratio_)
                        {
                            rect_middle_points_vector.emplace_back(cv::Point2f ((float)rect_to_color_select.tl().x+((float)rect_to_color_select.width/2),(float)rect_to_color_select.tl().y+((float)rect_to_color_select.height/2)));
                            rect_vector.emplace_back(rect_to_color_select);
                            for( int  j= 0; j < 4; j++ )
                            {
                                cv::line(mor_img, min_area_rect_points[j], min_area_rect_points[(j + 1) % 4], cv::Scalar(255), 2,cv::LINE_AA);
                                point_x_vector.push_back(min_area_rect_points[j].x);
                                point_y_vector.push_back(min_area_rect_points[j].y);
                            }
                        } else continue;
                } else continue;
            } else continue;
        }
        if(!point_x_vector.empty()&&rect_middle_points_vector.size()==3)
        {
            std::vector<cv::Point2f> coordinate_vec2d;
            cv::Point2f center_point = getMiddlePoint(point_x_vector, point_y_vector,coordinate_vec2d);
            getDirection(rect_middle_points_vector,rect_vector);
            cv::line(mor_img,coordinate_vec2d[0],coordinate_vec2d[1],cv::Scalar(255));
            cv::line(mor_img,coordinate_vec2d[0],coordinate_vec2d[2],cv::Scalar(255));
            cv::line(mor_img,coordinate_vec2d[0],coordinate_vec2d[3],cv::Scalar(255));
            cv::circle(mor_img, center_point, 10, cv::Scalar(255), 2);
        }

        binary_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", mor_img).toImageMsg());
        }

        bool Detector::chooseRect(const cv::Point2f &point1, const cv::Point2f &point2,const cv::Point2f &point3)
        {
            double subtract_x = abs(sqrt(pow(abs(point1.x-point3.x),2)+pow(abs(point1.y-point3.y),2)));
            double subtract_y = abs(sqrt(pow(abs(point1.x-point2.x),2)+pow(abs(point1.y-point2.y),2)));
            double k= abs(subtract_y/subtract_x);
            double area=subtract_y*subtract_x;
            if(k<=1+k_bias_&&k>=1-k_bias_&&subtract_x <= subtract_y + length_bias_ && subtract_x >= subtract_y - length_bias_&&area>=min_area_thresh_&&area<=max_area_thresh_)
            {
                        return true;
            }
                else return false;
        }

        cv::Point2f Detector::getMiddlePoint(const std::vector<float> &point_x_vector,const std::vector<float> &point_y_vector,std::vector<cv::Point2f> &coordinate_vec2d)
        {
            float biggest_x = *std::max_element(std::begin(point_x_vector), std::end(point_x_vector));
            float biggest_y=*std::max_element(std::begin(point_y_vector),std::end(point_y_vector));
            float smallest_x=*std::min_element(std::begin(point_x_vector), std::end(point_x_vector));
            float smallest_y=*std::min_element(std::begin(point_y_vector),std::end(point_y_vector));

            std::vector<cv::Point2f> corner_point2d_vec;
            corner_point2d_vec.emplace_back(smallest_x,smallest_y);
            corner_point2d_vec.emplace_back(biggest_x,smallest_y);
            corner_point2d_vec.emplace_back(biggest_x,biggest_y);
            corner_point2d_vec.emplace_back(smallest_x,biggest_y);

            std::vector<cv::Point3f> corner_point3d_vec;
            corner_point3d_vec.emplace_back(0,0,150);
            corner_point3d_vec.emplace_back(0,150,150);
            corner_point3d_vec.emplace_back(0,150,0);
            corner_point3d_vec.emplace_back(0,0,0);
            cv::solvePnP(corner_point3d_vec,corner_point2d_vec,camera_matrix_,distortion_coefficients_,rvec_,tvec_);

            std::vector<cv::Point3f> coordinate_vec3d;
            coordinate_vec3d.emplace_back(0, 75, 75);
            coordinate_vec3d.emplace_back(50, 75, 75);
            coordinate_vec3d.emplace_back(0, 125, 75);
            coordinate_vec3d.emplace_back(0, 75, 125);
            cv::projectPoints(coordinate_vec3d,rvec_,tvec_,camera_matrix_,distortion_coefficients_,coordinate_vec2d);

            float middle_x=smallest_x+(biggest_x-smallest_x)/2;
            float middle_y=smallest_y+(biggest_y-smallest_y)/2;

            return {middle_x,middle_y};
        }

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

    void Detector::getDirection(const std::vector<cv::Point2f> &rect_middle_points_vector,const std::vector<cv::Rect> &rect_vector)
    {
        std::vector<float> center_x_vector;
        std::vector<float> center_y_vector;
        for(int i=0;i<rect_middle_points_vector.size();i++)
        {
            center_x_vector.emplace_back(rect_middle_points_vector[i].x);
            center_y_vector.emplace_back(rect_middle_points_vector[i].y);
        }
        std::sort(center_x_vector.begin(),center_x_vector.end());
        std::sort(center_y_vector.begin(),center_y_vector.end());
        std_msgs::String msg;
        std::stringstream ss;
        int counter=0;
        for(int i=0;i<rect_vector.size();i++)
        {
            if (!rect_vector[i].contains(cv::Point2f(center_x_vector[2], center_y_vector[2]))) {
                counter++;
                if (counter == 3) {
                    ss << "br";
                    msg.data = ss.str();
                    direction_publisher_.publish(msg);
                    return;
                }
            }
        }
        counter=0;
        for(int i=0;i<rect_vector.size();i++)
        {
            if (!rect_vector[i].contains(cv::Point2f(center_x_vector[2], center_y_vector[0]))) {
                counter++;
                if (counter == 3) {
                    ss << "tr";
                    msg.data = ss.str();
                    direction_publisher_.publish(msg);
                    return;
                }
            }
        }
        counter=0;
        for(int i=0;i<rect_vector.size();i++)
        {
            if (!rect_vector[i].contains(cv::Point2f(center_x_vector[0], center_y_vector[0]))) {
                counter++;
                if (counter == 3) {
                    ss << "tl";
                    msg.data = ss.str();
                    direction_publisher_.publish(msg);
                    return;
                }
            }
        }
        counter=0;
        for(int i=0;i<rect_vector.size();i++)
        {
            if (!rect_vector[i].contains(cv::Point2f(center_x_vector[0], center_y_vector[2]))) {
                counter++;
                if (counter == 3) {
                    ss << "bl";
                    msg.data = ss.str();
                    direction_publisher_.publish(msg);
                    return;
                }
            }
        }

    }


    void Detector::dynamicCallback(mineral_detect::dynamicConfig &config) {
            morph_type_ = config.morph_type;
            morph_iterations_ = config.morph_iterations;
            thresh1_ = config.canny_thresh1;
            thresh2_ = config.canny_thresh2;
            k_bias_=config.k_bias;
            length_bias_=config.length_bias;
            min_area_thresh_=config.min_area_thresh;
            max_area_thresh_=config.max_area_thresh;
            lower_hsv_h_=config.lower_hsv_h;
            lower_hsv_s_=config.lower_hsv_s;
            lower_hsv_v_=config.lower_hsv_v;
            upper_hsv_h_=config.upper_hsv_h;
            upper_hsv_s_=config.upper_hsv_s;
            upper_hsv_v_=config.upper_hsv_v;
            roi_nonzero_percent_=config.roi_nonzero_percent;
            contours_ratio_=config.contours_ratio;
        }

        Detector::~Detector()=default;
    }
PLUGINLIB_EXPORT_CLASS(mineral_detect::Detector, nodelet::Nodelet)
