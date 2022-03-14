# include "overlay_lidar_camera.hpp"

namespace SegmentationMapping
{

OverlayLidarCamera::OverlayLidarCamera():
debug_count_(0)
{
    nh_ = ros::NodeHandle("~");
    it_ = std::make_shared<image_transport::ImageTransport>(nh_);
    
    nh_.getParam("image_topic", image_topic_name_);
    nh_.getParam("camera_info_topic", camera_info_topic_name_);
    nh_.getParam("lidar_topic", lidar_topic_name_);
    nh_.param<bool>("image_compressed", is_image_compressed_, false);
    nh_.getParam("C_r_CL", C_r_CL);
    nh_.getParam("C_q_CL", C_q_CL);
    nh_.getParam("camera_frame_name", camera_frame_name_);
    // nh_.getParam("lidar_frame_name", );

    // initialize the subscriber to the image
    image_transport::TransportHints hints(
        "compressed",
        ros::TransportHints());

    uint queue_length = 50;
    if(is_image_compressed_)
    {
        sub_image_ = std::make_shared<ImageSubscriber>(
            *it_, image_topic_name_, queue_length, hints);
    }
    else
    {
        sub_image_ = std::make_shared<ImageSubscriber>(
            *it_, image_topic_name_, queue_length);
    }


    // initialize the subscriber to the intrinsics
    sub_camera_info_ = std::make_shared<CameraInfoSubscriber>(nh_, camera_info_topic_name_, queue_length);

    // initialize the subscriber to the point cloud
    sub_point_cloud_ = std::make_shared<PointCloudSubscriber>(nh_, lidar_topic_name_, queue_length);
    
    // initialize the synchronizer
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(
        SyncPolicy(300), *sub_image_, *sub_camera_info_, *sub_point_cloud_);
    // register callback
    sync_->registerCallback(std::bind(
        &OverlayLidarCamera::callBack, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // advertise publisher
    pub_point_cloud_coloured_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "point_cloud_coloured", 5);
}

void OverlayLidarCamera::callBack(
    const sensor_msgs::ImageConstPtr& _msg_image,
    const sensor_msgs::CameraInfoConstPtr& _msg_camera_info,
    const sensor_msgs::PointCloud2ConstPtr& _msg_point_cloud)
{
    std::lock_guard<std::mutex> lock(mutex_input_);

    ROS_WARN("received image and point cloud, h: %d, w: %d", _msg_camera_info->height, _msg_camera_info->width);
    debug_count_ += 1;

    // convert message to image
    cv::Mat image;
    try
    {
        image = cv_bridge::toCvShare(
            _msg_image, 
            sensor_msgs::image_encodings::RGB8)->image;
    }
    catch(const std::exception& e)
    {
        ROS_WARN("Unable to convert from ROS image to OpenCV image");
        ROS_WARN("%s", e.what());
    }

    // convert message to pointcloud
    pcl::PointCloud<pcl::PointXYZ> cloud_lidar;
    ros::Time time_stamp;
    pcl::fromROSMsg(*_msg_point_cloud, cloud_lidar);

    // convert points from the lidar frame to the camera frame
    Eigen::Vector3d t_camera_lidar = Eigen::Vector3d(C_r_CL[0], C_r_CL[1], C_r_CL[2]); // tx, ty, tz
    Eigen::Quaterniond q_camera_lidar = Eigen::Quaterniond(C_q_CL[3], C_q_CL[0], C_q_CL[1], C_q_CL[2]); // w, x, y, z
    pcl::PointCloud<pcl::PointXYZ> cloud_camera;
    pcl::transformPointCloud(cloud_lidar, cloud_camera, t_camera_lidar, q_camera_lidar);

    // project points into the image plane to get the colours
    uint img_height = _msg_camera_info->height;
    uint img_width = _msg_camera_info->width;
    float fx = _msg_camera_info->K[0];
    float fy = _msg_camera_info->K[4];
    float cx = _msg_camera_info->K[2];
    float cy = _msg_camera_info->K[5];
    
    // ROS_WARN("fx: %f, fy: %f, cx: %f, cy: %f", fx, fy, cx, cy);

    pcl::PointCloud<pcl::PointXYZRGB> cloud_coloured;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_camera.begin(); it != cloud_camera.end(); ++it)
    {
        pcl::PointXYZRGB pt;
        pt.x = it->x;
        pt.y = it->y;
        pt.z = it->z;
        pt.r = 128;
        pt.g = 10;
        pt.b = 128;

        colourPointWithImage(_msg_camera_info, pt, image);
        cloud_coloured.push_back(pt);
    }

    sensor_msgs::PointCloud2 msg_cloud_coloured;
    pcl::toROSMsg(cloud_coloured, msg_cloud_coloured);
    msg_cloud_coloured.header = _msg_point_cloud->header;
    msg_cloud_coloured.header.frame_id = camera_frame_name_;

    // publish the point cloud
    pub_point_cloud_coloured_.publish(msg_cloud_coloured);
}

void OverlayLidarCamera::colourPointWithImage(
    const sensor_msgs::CameraInfoConstPtr& _msg_camera_info, 
    pcl::PointXYZRGB& pt, 
    const cv::Mat& image)
{
    int u = round((_msg_camera_info->K[0] * pt.x / pt.z) + _msg_camera_info->K[2]);
    int v = round((_msg_camera_info->K[4] * pt.y / pt.z) + _msg_camera_info->K[5]);

    if (u >= 0 && v > 0 && u < _msg_camera_info->width && v < _msg_camera_info->height && pt.z > 0 && pt.z < 256)
    {
        pt.r = image.at<cv::Vec3b>(v, u)[0];
        pt.g = image.at<cv::Vec3b>(v, u)[1];
        pt.b = image.at<cv::Vec3b>(v, u)[2];
    }
}


}