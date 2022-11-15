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
    int save_count;
    bool save_flag;

    // calibration
    std::vector<double> K_coeff;
    std::vector<double> dist_coeff;
    double calib_xt; double calib_yt; double calib_zt;
    double calib_r;  double calib_p;  double calib_y;

    // radar voxel grid filter
    double radar_voxel_param;


public:
    
    FUSION() : it_(nh)
    {
        // params.yaml 
        nh.param<std::string>("cam_rad_fusion/imageTopic", image_topic, "/camera_array/cam0/image_raw");
        nh.param<std::string>("cam_rad_fusion/lidarTopic", lidar_topic, "/ouster/points");
        nh.param<std::string>("cam_rad_fusion/radarTopic", radar_topic, "/point_cloud");
        nh.param<std::string>("cam_rad_fusion/saveDirectory", saveDirectory, "/home/ysh/catkin_ws/src/cam_rad_fusion/DATA/");
        nh.param<int>("cam_rad_fusion/save_count_duty", save_count_duty, 5);
        nh.param<bool>("cam_rad_fusion/save_flag", save_flag, true);
        nh.param<std::vector<double>>("/cam_rad_fusion/K", K_coeff, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        nh.param<std::vector<double>>("/cam_rad_fusion/D", dist_coeff, {0.0, 0.0, 0.0, 0.0, 0.0});
        nh.param<double>("/cam_rad_fusion/radar_voxel_param", radar_voxel_param, 0.5);


        // message filter
        image_sub.subscribe(nh, image_topic, 1);
        lidar_sub.subscribe(nh, lidar_topic, 1);
        radar_sub.subscribe(nh, radar_topic, 1);
        sync_.reset(new Sync(MySyncPolicy(10), image_sub, lidar_sub, radar_sub));
        sync_->registerCallback(boost::bind(&FUSION::callback, this, _1, _2, _3));

        image_pub = it_.advertise("/FUSION/image_out", 1);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/FUSION/cloud_out", 1);
        
        count = 0;
        save_count = 0;
    }



    void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& lidar, const sensor_msgs::PointCloud2ConstPtr& radar)
    {
        
        // ROS_INFO("Synchronization Successful!!");
        //std::cout << "Your image time : " << image->header.stamp.sec << image->header.stamp.nsec << std::endl;
        //std::cout << "Your lidar time : " << lidar->header.stamp.sec << lidar->header.stamp.nsec << std::endl;  
        //std::cout << "Your radar time : " << radar->header.stamp.sec << radar->header.stamp.nsec << std::endl;

        int NUM_ZEROS = 3;
        std::string count_str = std::to_string(save_count);
        std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;



        ///////////////////////
        /* HOW TO USE OPENCV */
        ///////////////////////

        // change format  ROS -> OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        cv::Mat image_cv = cv_ptr->image;

        // rectify image
        cv::Matx33d K(K_coeff[0], K_coeff[1], K_coeff[2], K_coeff[3], K_coeff[4], K_coeff[5], K_coeff[6], K_coeff[7], K_coeff[8]);
        cv::Mat image_rectify;
        cv::undistort(image_cv, image_rectify, K, dist_coeff);

        // TODO : use opencv function with "image_cv"
        //cv::rectangle(image_rectify, cv::Rect(50, 50, 100, 50), cv::Scalar(0, 0, 255), 2);

        // save IMG
        std::string filename_image = saveDirectory + "image" + count_str_padded + ".png";
        if((count % save_count_duty == 0) && save_flag)
        cv::imwrite(filename_image, image_rectify);

        // change format  OpenCV -> ROS
        // moved to calibration part



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
        if((count % save_count_duty == 0) && save_flag)
        pcl::io::savePCDFileASCII(filename_lidar, *cloud_ptr);

        // change format PCL -> ROS
        // moved to calibration part



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
        if((count % save_count_duty == 0) && save_flag)
        {
            pcl::io::savePCDFileASCII(filename_radar, radar4d);
            save_count++;
        }
        



        //////////////////////////////
        /* CAMERA-RADAR-CALIBRATION */
        //////////////////////////////

        // get calibration data
        nh.getParam("/dynamic_reconfigure/calib_xt", calib_xt);
        nh.getParam("/dynamic_reconfigure/calib_yt", calib_yt);
        nh.getParam("/dynamic_reconfigure/calib_zt", calib_zt);
        nh.getParam("/dynamic_reconfigure/calib_r",  calib_r);
        nh.getParam("/dynamic_reconfigure/calib_p",  calib_p);
        nh.getParam("/dynamic_reconfigure/calib_y",  calib_y);

        std::vector<double> pose;
        pose.push_back(calib_xt);
        pose.push_back(calib_yt);
        pose.push_back(calib_zt);
        pose.push_back(calib_r);
        pose.push_back(calib_p);
        pose.push_back(calib_y);

        // calculate Extrinsic & Intrinsic
        Eigen::Matrix4d Extrinsic = transform3D(pose);
        Eigen::Matrix4d Intrinsic = Eigen::Matrix4d::Identity();
        Intrinsic(0,0) = K_coeff[0];  // fx [m]
        Intrinsic(0,2) = K_coeff[2];  // cx [px]
        Intrinsic(1,1) = K_coeff[4];  // fy [m]
        Intrinsic(1,2) = K_coeff[5];  // cy [px]

        // voxelize radar
        pcl::PointCloud<pcl::PointXYZ>::Ptr radar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        copyRadar(radar4d, radar_cloud);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_radar;
        pcl::PointCloud<pcl::PointXYZ>::Ptr radar_voxel(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_grid_radar.setInputCloud(radar_cloud);
        voxel_grid_radar.setLeafSize(radar_voxel_param, radar_voxel_param, radar_voxel_param);
        voxel_grid_radar.filter(*radar_voxel);

        // change format PCL -> ROS
        sensor_msgs::PointCloud2::Ptr cloud_msg;
        cloud_msg.reset(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*radar_voxel, *cloud_msg);
        cloud_msg->header.stamp    = ros::Time::now();
        cloud_msg->header.frame_id = "os_sensor";
        cloud_pub.publish(cloud_msg);



        // project radar to image
        // for (int i = 0; i < radar4d.size(); i++)
        // {
        //     int color;
        //     if (radar4d.at(i).y > 51) {color = 255;}
        //     else {color = 5 * radar4d.at(i).y;}

        //     Eigen::Vector4d radar3Dpoints(radar4d.at(i).x, radar4d.at(i).y, radar4d.at(i).z, 1);
        //     Eigen::Vector4d radar2image(0.0, 0.0, 0.0, 0.0);
        //     radar2image = Intrinsic * Extrinsic * radar3Dpoints;
        //     int u = radar2image(0)/radar2image(2);  // normalization x / z
        //     int v = radar2image(1)/radar2image(2);  // normalization y / z
        //     if ((0 < u) && (u < 1440) && (0 < v) && (v < 1080))
        //     cv::circle(image_rectify, cv::Point(u,v), 4.0, cv::Scalar(0, 0, color), -1, -1, 0);
        //     //ROS_INFO("u : %d v : %d", u, v);
        //     //ROS_INFO("radar2image : %f, %f, %f, %f", radar2image(0), radar2image(1), radar2image(2),radar2image(3) );
        // }

            for (int i = 0; i < radar_voxel->size(); i++)
        {
            double x_temp = radar_voxel->points.at(i).x;
            double y_temp = radar_voxel->points.at(i).y;
            double z_temp = radar_voxel->points.at(i).z;
            int color;
            if ( y_temp> 51) {color = 255;}
            else {color = 5 * y_temp;}

            Eigen::Vector4d radar3Dpoints(x_temp, y_temp, z_temp, 1);
            Eigen::Vector4d radar2image(0.0, 0.0, 0.0, 0.0);
            radar2image = Intrinsic * Extrinsic * radar3Dpoints;
            int u = radar2image(0)/radar2image(2);  // normalization x / z
            int v = radar2image(1)/radar2image(2);  // normalization y / z
            if ((0 < u) && (u < 1440) && (0 < v) && (v < 1080))
            cv::circle(image_rectify, cv::Point(u,v), 4.0, cv::Scalar(0, 0, color), -1, -1, 0);
            //ROS_INFO("u : %d v : %d", u, v);
            //ROS_INFO("radar2image : %f, %f, %f, %f", radar2image(0), radar2image(1), radar2image(2),radar2image(3) );
        }

        // change format  OpenCV -> ROS
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_rectify).toImageMsg();
        image_msg->header.stamp = ros::Time::now();
        image_pub.publish(image_msg);



   

        count++;
    }

    Eigen::Matrix4d transform3D(std::vector<double> pose)
    {
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

        double xt   = pose[0];          double yt    = pose[1];          double zt   = pose[2];
        double roll = DEG2RAD(pose[3]); double pitch = DEG2RAD(pose[4]); double yaw  = DEG2RAD(pose[5]);

        matrix(0, 0) = cos(yaw) * cos(pitch);
        matrix(1, 0) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
        matrix(2, 0) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
        matrix(0, 1) = sin(yaw) * cos(pitch);
        matrix(1, 1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);       
        matrix(2, 1) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
        matrix(0, 2) = -sin(pitch);
        matrix(1, 2) = cos(pitch) * sin(roll);
        matrix(2, 2) = cos(pitch) * cos(roll);

        Eigen::Matrix3d RT = Eigen::Matrix3d::Identity();
        RT(0,0) = matrix(0,0);
        RT(0,1) = matrix(0,1);
        RT(0,2) = matrix(0,2);
        RT(1,0) = matrix(1,0);
        RT(1,1) = matrix(1,1);
        RT(1,2) = matrix(1,2);
        RT(2,0) = matrix(2,0);
        RT(2,1) = matrix(2,1);
        RT(2,2) = matrix(2,2);

        Eigen::Vector3d t(xt, yt, zt);
        
        Eigen::Vector3d minusRTt(0.0, 0.0, 0.0);
        minusRTt = -1 * RT * t;

        matrix(0, 3) = minusRTt(0);
        matrix(1, 3) = minusRTt(1);
        matrix(2, 3) = minusRTt(2);

        return matrix;
    }


    void copyRadar(const pcl::PointCloud<pcl::PointRADAR> &radar4d,  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        for (int i = 0; i < radar4d.size(); i++)
        {
            cloud->points.emplace_back( pcl::PointXYZ(radar4d.at(i).x, radar4d.at(i).y, radar4d.at(i).z) );
        }
    }


};





