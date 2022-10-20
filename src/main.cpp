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
        gray_publisher_ = nh_.advertise<sensor_msgs::Image>("gray_publisher",1);
        point_publisher_=nh_.advertise<std_msgs::Float32MultiArray>("point_publisher",1);
        callback_ = boost::bind(&Detector::dynamicCallback, this, _1);
        server_.setCallback(callback_);
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
        cv::findContours(mor_img,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
        std_msgs::Float32MultiArray middle_point_array;
        std::vector<cv::Point2i> middle_points_vec;
        std::vector<cv::Point2i> text_points_vec;
        std::vector<cv::Rect> rect_vec;
        for (int i = 0; i < contours.size(); ++i)
        {
            std::vector<cv::Point2i> hull;
            cv::convexHull( contours[i],hull, true);
                    if(cv::contourArea(hull)>min_area_thresh_)
                    {
                        auto moment = cv::moments(hull);
                        auto cx = int(moment.m10 / moment.m00);
                        auto cy = int(moment.m01/  moment.m00);
                        cv::Point2i origin_middle_point(cx,cy);
                        int direct=2;
                        if(cx<cv_image_->image.cols/2-1) direct=0;
                        double hu_moment[7];
                        cv::HuMoments(moment,hu_moment);
//                        cx= int( 9.79558722e-01 *cx+( -1.46154603e-02*cy)+4.15699319e+00 *100*hu_moment[0]+(direct*-8.52015082e+01)+42.95259902) ;
//                        cy= int((-6.02135396e-03*cx)+1.03597975e+00*cy+(-4.00755478e+01*100*hu_moment[0])+(direct*-2.97492906e+00)+510.66173278);
                        int trans_cx= int( 9.79558722e-01 *cx+( -1.46154603e-02*cy)+4.15699319e+00 *100*hu_moment[0]+(direct*-8.52015082e+01)+42.95259902) ;
//                        int trans_cy= int((-6.02135396e-03*cx)+1.03597975e+00*cy+(-4.00755478e+01*100*hu_moment[0])+(direct*-2.97492906e+00)+510.66173278);
                        if (direct==0 && trans_cx<origin_middle_point.x)
                        {
                            direct = 2;
                            cx = int(
                                    9.79558722e-01 * cx + (-1.46154603e-02 * cy) + 4.15699319e+00 * 100 * hu_moment[0] +
                                    (direct * -8.52015082e+01) + 42.95259902);
                            cy = int((-6.02135396e-03 * cx) + 1.03597975e+00 * cy +
                                     (-4.00755478e+01 * 100 * hu_moment[0]) + (direct * -2.97492906e+00) +
                                     510.66173278);
                        }
                        else if (direct==2 && trans_cx>origin_middle_point.x)
                        {
                            direct = 0;
                            cx = int(
                                    9.79558722e-01 * cx + (-1.46154603e-02 * cy) + 4.15699319e+00 * 100 * hu_moment[0] +
                                    (direct * -8.52015082e+01) + 42.95259902);
                            cy = int((-6.02135396e-03 * cx) + 1.03597975e+00 * cy +
                                     (-4.00755478e+01 * 100 * hu_moment[0]) + (direct * -2.97492906e+00) +
                                     510.66173278);
                        }
                        else
                        {
                            direct = 1;
                            cx = int(
                                    9.79558722e-01 * cx + (-1.46154603e-02 * cy) + 4.15699319e+00 * 100 * hu_moment[0] +
                                    (direct * -8.52015082e+01) + 42.95259902);
                            cy = int((-6.02135396e-03 * cx) + 1.03597975e+00 * cy +
                                     (-4.00755478e+01 * 100 * hu_moment[0]) + (direct * -2.97492906e+00) +
                                     510.66173278);

                        }
                        cv::Point2i middle_point(cx,cy);
                        cv::circle(cv_image_->image, middle_point, 3, cv::Scalar(0,199,140), 7);
//                        cv::Point2i text_point (int(middle_point.x*x_scale_),int((middle_point.y+int(rect_to_color_select.height/2))*y_scale_));
                        cv::polylines(cv_image_->image,hull, true,cv::Scalar(106,90,205),6);
                        if(save_on_)
                        {
                            std::ofstream ofs;
                            ofs.open("/home/yamabuki/detect_ws/recorder.txt",std::ios::out|std::ios::app );
//                            ofs << middle_point << "\t" << hu_moment[0] << "\t"  << theta << "\t" << std::endl;
                            ofs.close();
                            cv::imwrite("/home/yamabuki/detect_ws/image/"+ std::to_string(num)+".jpg",mor_img);
                            std::cout<<"save"<<num<<std::endl;
                            num++;
                        }
                        if(save_on_) save_on_= false;
                        middle_points_vec.emplace_back(middle_point);
//                        rect_vec.emplace_back(rect_to_color_select);
//                        text_points_vec.emplace_back(text_point);
                } else continue;
//            } else continue;
        }
        if(!middle_points_vec.empty())
        {
            cv::Point2i target_point= targetPointSelect(middle_points_vec,rect_vec,text_points_vec);
            middle_point_array.data.push_back((float)target_point.x);
            middle_point_array.data.push_back((float)target_point.y);
            point_publisher_.publish(middle_point_array);
        }

        test_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), cv_image_->encoding, cv_image_->image).toImageMsg());
        }
    cv::Point2i Detector::targetPointSelect(std::vector<cv::Point2i> &points_vec,std::vector<cv::Rect> &rect_vec,std::vector<cv::Point2i> &text_vec)
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
//        cv::Point2i text_point=text_vec[min_point_index];
//        cv::Rect target_rect=rect_vec[min_point_index];
//        cv::rectangle(cv_image_->image,target_rect,cv::Scalar(0,199,140),5);
        cv::circle(cv_image_->image, target_point, 2, cv::Scalar(0,199,140), 7);
//        cv::putText(cv_image_->image,"Target",text_point,cv::FONT_HERSHEY_SCRIPT_COMPLEX,text_size_,cv::Scalar(0,199,140),8);
        return target_point;
    }

//    cv::Point2i Detector::getTLPoint(const std::vector<cv::Point2i> &hull)
//    {
//        std::vector<int> x_vec;
//        std::vector<int> y_vec;
//        for(const auto & i : hull)
//        {
//            x_vec.push_back(i.x);
//            y_vec.push_back(i.y);
//        }
//
//        int smallest_x=*std::min_element(std::begin(x_vec), std::end(x_vec));
//        int smallest_y=*std::min_element(std::begin(y_vec),std::end(y_vec));
//
//        return cv::Point2i (smallest_x,smallest_y);
//
//    }


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
            x_scale_=config.x_scale;
            y_scale_=config.y_scale;
            text_size_=config.text_size;
            y_bias_=config.y_bias;
            save_on_=config.save_on;
        }

        Detector::~Detector()=default;
    }
PLUGINLIB_EXPORT_CLASS(mineral_detect::Detector, nodelet::Nodelet)
