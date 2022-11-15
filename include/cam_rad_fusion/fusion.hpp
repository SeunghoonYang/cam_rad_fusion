#pragma once
#include "cam_rad_fusion/global_header.hpp"


class FUSION
{
private:
    ros::NodeHandle nh;

    // params.yaml
    std::string image_topic;
    std::string lidar_topic;
    std::string radar_topic;


    // message filter - camera & lidar
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> radar_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    // publisher
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub;


public:
    
    FUSION() : it_(nh)
    {
        // params.yaml 
        nh.param<std::string>("cam_rad_fusion/imageTopic", image_topic, "/camera_array/cam0/image_raw");
        nh.param<std::string>("cam_rad_fusion/lidarTopic", lidar_topic, "/ouster/points");
        nh.param<std::string>("cam_rad_fusion/radarTopic", radar_topic, "/point_cloud");

        // message filter
        image_sub.subscribe(nh, image_topic, 1);
        lidar_sub.subscribe(nh, lidar_topic, 1);
        radar_sub.subscribe(nh, radar_topic, 1);
        sync_.reset(new Sync(MySyncPolicy(10), image_sub, lidar_sub, radar_sub));
        sync_->registerCallback(boost::bind(&FUSION::callback, this, _1, _2, _3));

        image_pub = it_.advertise("/FUSION/image_out", 1);

    }

    void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& lidar, const sensor_msgs::PointCloud2ConstPtr& radar)
    {
        // ROS_INFO("Synchronization Successful!!");
        std::cout << "Your image time : " << image->header.stamp.sec << image->header.stamp.nsec << std::endl;
        std::cout << "Your lidar time : " << lidar->header.stamp.sec << lidar->header.stamp.nsec << std::endl;  
        std::cout << "Your radar time : " << radar->header.stamp.sec << radar->header.stamp.nsec << std::endl;

        // change format  ROS -> OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        cv::Mat image_cv = cv_ptr->image;

        // TODO!!!!!!
        // use opencv function with "image_cv"
        cv::rectangle(image_cv, cv::Rect(50, 50, 100, 50), cv::Scalar(0, 0, 255), 2);

        // change format  OpenCV -> ROS
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_cv).toImageMsg();
        image_pub.publish(image_msg);
    }


};





