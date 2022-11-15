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
    ros::Publisher cloud_pub;

    // save image, lidar, radar
    std::string saveDirectory;
    int count;
    int save_count_duty;



public:
    
    FUSION() : it_(nh)
    {
        // params.yaml 
        nh.param<std::string>("cam_rad_fusion/imageTopic", image_topic, "/camera_array/cam0/image_raw");
        nh.param<std::string>("cam_rad_fusion/lidarTopic", lidar_topic, "/ouster/points");
        nh.param<std::string>("cam_rad_fusion/radarTopic", radar_topic, "/point_cloud");
        nh.param<std::string>("cam_rad_fusion/saveDirectory", saveDirectory, "/home/ysh/catkin_ws/src/cam_rad_fusion/DATA/");
        nh.param<int>("cam_rad_fusion/save_count_duty", save_count_duty, 5);

        // message filter
        image_sub.subscribe(nh, image_topic, 1);
        lidar_sub.subscribe(nh, lidar_topic, 1);
        radar_sub.subscribe(nh, radar_topic, 1);
        sync_.reset(new Sync(MySyncPolicy(10), image_sub, lidar_sub, radar_sub));
        sync_->registerCallback(boost::bind(&FUSION::callback, this, _1, _2, _3));

        image_pub = it_.advertise("/FUSION/image_out", 1);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/FUSION/cloud_out", 1);
        
        count = 0;
    }

    void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& lidar, const sensor_msgs::PointCloud2ConstPtr& radar)
    {
        

        
        // ROS_INFO("Synchronization Successful!!");
        //std::cout << "Your image time : " << image->header.stamp.sec << image->header.stamp.nsec << std::endl;
        //std::cout << "Your lidar time : " << lidar->header.stamp.sec << lidar->header.stamp.nsec << std::endl;  
        //std::cout << "Your radar time : " << radar->header.stamp.sec << radar->header.stamp.nsec << std::endl;

        int NUM_ZEROS = 3;
        std::string count_str = std::to_string(count);
        std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;



        ///////////////////////
        /* HOW TO USE OPENCV */
        ///////////////////////

        // change format  ROS -> OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        cv::Mat image_cv = cv_ptr->image;

        // TODO : use opencv function with "image_cv"
        cv::rectangle(image_cv, cv::Rect(50, 50, 100, 50), cv::Scalar(0, 0, 255), 2);

        // save IMG
        std::string filename_image = saveDirectory + "image" + count_str_padded + ".png";
        if(count % save_count_duty == 0)
        cv::imwrite(filename_image, image_cv);

        // change format  OpenCV -> ROS
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_cv).toImageMsg();
        image_msg->header.stamp = ros::Time::now();
        image_pub.publish(image_msg);



        ///////////////////////////
        /* HOW TO USE PCL(LIDAR) */
        ///////////////////////////

        // change format ROS -> PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;  // pointer declaration
        cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>()); // dynamic allocation
        pcl::fromROSMsg(*lidar, *cloud_ptr);

        // TODO : use pcl function with "cloud_ptr"
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_grid.setInputCloud(cloud_ptr);
        voxel_grid.setLeafSize(0.50f, 0.50f, 0.50f);
        voxel_grid.filter(*cloud_voxel);

        // save PCD
        std::string filename_lidar = saveDirectory + "lidar" + count_str_padded + ".pcd";
        if(count % save_count_duty == 0)
        pcl::io::savePCDFileASCII(filename_lidar, *cloud_ptr);

        // change format PCL -> ROS
        sensor_msgs::PointCloud2::Ptr cloud_msg;
        cloud_msg.reset(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_voxel, *cloud_msg);
        cloud_msg->header.stamp    = ros::Time::now();
        cloud_msg->header.frame_id = "os_sensor";
        cloud_pub.publish(cloud_msg);




        ///////////////////////////
        /* HOW TO USE PCL(RADAR) */
        ///////////////////////////
        
        // change format ROS -> PCL
        pcl::PointCloud<pcl::PointRADAR> radar4d;
        pcl::fromROSMsg(*radar, radar4d);
        // ROS_INFO("x : %f  y: %f  z : %f  intensity : %f", radar4d.back().x, radar4d.back().y, radar4d.back().z, radar4d.back().intensity); 
        // ROS_INFO("normalx : %f  normaly: %f  normalz : %f  curvature : %f", radar4d.back().normal_x, radar4d.back().normal_y, radar4d.back().normal_z, radar4d.back().curvature); 

        // access radar4d
        // for (int i = 0; i < radar4d.size(); i++)
        // {
        //     radar4d.at(i).x;
        //     radar4d.at(i).y;
        //     radar4d.at(i).z;
        // }


        // save PCD
        std::string filename_radar = saveDirectory + "radar4d" + count_str_padded + ".pcd";
        if(count % save_count_duty == 0)
        pcl::io::savePCDFileASCII(filename_radar, radar4d);

        count++;

    }


};





