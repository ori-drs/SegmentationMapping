#ifndef OVERLAY_LIDAR_MULTI_CAMERA_H
#define OVERLAY_LIDAR_MULTI_CAMERA_H

#include <ros/ros.h>

// #include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include "pcl_ros/point_cloud.h"

#include <iostream>
#include <mutex>

namespace SegmentationMapping
{

typedef message_filters::sync_policies::ApproximateTime
<sensor_msgs::Image, sensor_msgs::CameraInfo, 
sensor_msgs::Image, sensor_msgs::CameraInfo,
sensor_msgs::Image, sensor_msgs::CameraInfo,
sensor_msgs::PointCloud2> SyncPolicy;

typedef image_transport::SubscriberFilter ImageSubscriber;
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CameraInfoSubscriber;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloudSubscriber;

class OverlayLidarMultiCamera
{
    public:
    OverlayLidarMultiCamera();
    ~OverlayLidarMultiCamera(){};

    void callBack(
        const sensor_msgs::ImageConstPtr& _msg_image_front,
        const sensor_msgs::CameraInfoConstPtr& _msg_camera_info_front,
        const sensor_msgs::ImageConstPtr& _msg_image_left,
        const sensor_msgs::CameraInfoConstPtr& _msg_camera_info_left,
        const sensor_msgs::ImageConstPtr& _msg_image_right,
        const sensor_msgs::CameraInfoConstPtr& _msg_camera_info_right,
        const sensor_msgs::PointCloud2ConstPtr& _msg_point_cloud
    );

    void projectAndColour(
        const std::vector<float> &C_r_CL,
        const std::vector<float> &C_q_CL,
        const sensor_msgs::CameraInfoConstPtr &msg_camera_info,
        const cv::Mat& image,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_coloured);

    std::shared_ptr<ImageSubscriber> getImage(
        int _index, image_transport::TransportHints _hints);

    void imageRos2cv(const sensor_msgs::ImageConstPtr& _msg_image, cv::Mat &_image);

    protected:
    ros::NodeHandle nh_;    // node handle

    std::shared_ptr<image_transport::ImageTransport> it_;

    std::vector<std::string> image_topic_names_;
    std::vector<std::string> camera_info_topic_names_;
    std::vector<std::string> camera_frame_names_;
    std::vector<bool> is_image_compressed_;

    std::string lidar_topic_name_;
    std::string lidar_frame_name_;
    
    // subscriber to images
    std::vector<std::shared_ptr<ImageSubscriber> > sub_images_;
    
    // subscriber to the camera infos
    std::vector<std::shared_ptr<CameraInfoSubscriber> > sub_camera_infos_; 

    // subscriber to Lidar point cloud
    std::shared_ptr<PointCloudSubscriber> sub_point_cloud_;

    // synchronizer
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;

    // publisher of coloured point cloud
    ros::Publisher pub_point_cloud_coloured_;

    // mutex
    std::mutex mutex_input_;

    // transform from lidar to camera
    std::vector<float> Cf_r_CfL;
    std::vector<float> Cf_q_CfL;

    std::vector<float> Cl_r_ClL;
    std::vector<float> Cl_q_ClL;

    std::vector<float> Cr_r_CrL;
    std::vector<float> Cr_q_CrL;

    // counter
    uint debug_count_;
    uint queue_length_;
};

}

#endif