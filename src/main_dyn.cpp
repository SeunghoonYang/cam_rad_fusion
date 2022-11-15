#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <cam_rad_fusion/dyn_reconfigConfig.h>

// dynamic reconfigure call back
void callback(cam_rad_fusion::dyn_reconfigConfig &config, uint32_t level)
{
    //ROS_INFO("Dynamic parameter: %d", config.test_parameter);
    ROS_INFO("xt, yt, zt, r, p, y: %f %f %f %f %f %f", config.calib_xt, config.calib_yt, config.calib_zt, 
                                                       config.calib_r,  config.calib_p,  config.calib_y);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "dynamic_reconfigure");

    // dynamic reconfigure
    dynamic_reconfigure::Server<cam_rad_fusion::dyn_reconfigConfig> server;
    dynamic_reconfigure::Server<cam_rad_fusion::dyn_reconfigConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();
    return 0;
}