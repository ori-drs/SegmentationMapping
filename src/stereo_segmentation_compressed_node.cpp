#include "stereo_segmentation_compressed.hpp"


int main (int argc, char** argv) {
  
  ros::init(argc, argv, "stereo_segmentation_compressed_node");
  ROS_INFO("Start stereo_seg_compr node");
  // SegmentationMapping::StereoSegmentation<14> stereo_seg;
  SegmentationMapping::StereoSegmentationCompressed<21> stereo_seg_compressed;

  ros::spin();

  return 0;
}
