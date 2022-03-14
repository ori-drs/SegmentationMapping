#include "overlay_lidar_multi_camera.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "overlay_lidar_multicam");
    SegmentationMapping::OverlayLidarMultiCamera colourful_lidar;

    ros::spin();
    return 0;
}