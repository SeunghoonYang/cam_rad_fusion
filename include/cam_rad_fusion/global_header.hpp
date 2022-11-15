#pragma once

// ROS - message filter
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


// ROS - interface(camera, lidar, radar)
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>


// OpenCV


// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// CPP, STL
#include <string>
