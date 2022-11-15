#include "cam_rad_fusion/fusion.hpp"

int main (int argc, char** argv)
{
    ros::init (argc, argv, "fusion");
    FUSION fusion;
    ros::spin();
}