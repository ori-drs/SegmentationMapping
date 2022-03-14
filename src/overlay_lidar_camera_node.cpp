#include "overlay_lidar_camera.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "overlay_lidar_camera");
    SegmentationMapping::OverlayLidarCamera colourful_lidar;

    ros::spin();
    return 0;
}