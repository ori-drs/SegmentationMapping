# include "overlay_lidar_multi_camera.hpp"

namespace SegmentationMapping
{

OverlayLidarMultiCamera::OverlayLidarMultiCamera():
debug_count_(0),
queue_length_(50)
{
    nh_ = ros::NodeHandle("~");
    it_ = std::make_shared<image_transport::ImageTransport>(nh_);
    
    nh_.getParam("image_topics", image_topic_names_);
    nh_.getParam("camera_info_topics", camera_info_topic_names_);
    nh_.getParam("camera_frame_names", camera_frame_names_);
    
    nh_.getParam("images_compressed", is_image_compressed_);
    nh_.getParam("lidar_topic", lidar_topic_name_);
    nh_.getParam("lidar_frame_name", lidar_frame_name_);
    
    nh_.getParam("Cf_r_CfL", Cf_r_CfL);
    nh_.getParam("Cf_q_CfL", Cf_q_CfL);
    nh_.getParam("Cl_r_ClL", Cl_r_ClL);
    nh_.getParam("Cl_q_ClL", Cl_q_ClL);
    nh_.getParam("Cr_r_CrL", Cr_r_CrL);
    nh_.getParam("Cr_q_CrL", Cr_q_CrL);

    image_transport::TransportHints hints(
        "compressed",
        ros::TransportHints());

    // initialize the subscriber to the images and intrinsics
    for (int i = 0; i < camera_frame_names_.size(); ++i)
    {
        sub_images_.push_back(getImage(i, hints));
        // ROS_WARN("%s", camera_info_topic_names_[i].c_str());
        sub_camera_infos_.push_back(std::make_shared<CameraInfoSubscriber>(
                nh_, camera_info_topic_names_[i], queue_length_));
    }

    // initialize the subscriber to the point cloud
    sub_point_cloud_ = std::make_shared<PointCloudSubscriber>(nh_, lidar_topic_name_, queue_length_);
    
    // initialize the synchronizer
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(
        SyncPolicy(300),
        *sub_images_[0], *sub_camera_infos_[0],
        *sub_images_[1], *sub_camera_infos_[1],
        *sub_images_[2], *sub_camera_infos_[2],
        *sub_point_cloud_);

    // register callback
    sync_->registerCallback(std::bind(
        &OverlayLidarMultiCamera::callBack, this,
        std::placeholders::_1, std::placeholders::_2, 
        std::placeholders::_3, std::placeholders::_4, 
        std::placeholders::_5, std::placeholders::_6, 
        std::placeholders::_7));

    // advertise publisher
    pub_point_cloud_coloured_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "point_cloud_coloured", 5);
}

void OverlayLidarMultiCamera::callBack(
    const sensor_msgs::ImageConstPtr& _msg_image_front,
    const sensor_msgs::CameraInfoConstPtr& _msg_camera_info_front,
    const sensor_msgs::ImageConstPtr& _msg_image_left,
    const sensor_msgs::CameraInfoConstPtr& _msg_camera_info_left,
    const sensor_msgs::ImageConstPtr& _msg_image_right,
    const sensor_msgs::CameraInfoConstPtr& _msg_camera_info_right,
    const sensor_msgs::PointCloud2ConstPtr& _msg_point_cloud)
{
    std::lock_guard<std::mutex> lock(mutex_input_);

    debug_count_ += 1;

    // convert message to image
    std::vector<cv::Mat> images(3, cv::Mat());
    imageRos2cv(_msg_image_front, images[0]);
    imageRos2cv(_msg_image_left, images[1]);
    imageRos2cv(_msg_image_right, images[2]);

    // convert message to pointcloud
    pcl::PointCloud<pcl::PointXYZ> cloud_lidar;
    ros::Time time_stamp;
    pcl::fromROSMsg(*_msg_point_cloud, cloud_lidar);

    // // create a point cloud to be coloured
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_coloured(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(cloud_lidar, *cloud_coloured);

    projectAndColour(Cf_r_CfL, Cf_q_CfL, _msg_camera_info_front, images[0], cloud_coloured);
    projectAndColour(Cl_r_ClL, Cl_q_ClL, _msg_camera_info_left, images[1], cloud_coloured);
    projectAndColour(Cr_r_CrL, Cr_q_CrL, _msg_camera_info_right, images[2], cloud_coloured);

    sensor_msgs::PointCloud2 msg_cloud_coloured;
    pcl::toROSMsg(*cloud_coloured, msg_cloud_coloured);
    msg_cloud_coloured.header = _msg_point_cloud->header;
    msg_cloud_coloured.header.frame_id = lidar_frame_name_;

    // publish the point cloud
    pub_point_cloud_coloured_.publish(msg_cloud_coloured);
}

std::shared_ptr<ImageSubscriber> OverlayLidarMultiCamera::getImage(
    int _index, image_transport::TransportHints _hints)
{
    if(is_image_compressed_[_index])
    {
        return std::make_shared<ImageSubscriber>(
            *it_, image_topic_names_[_index], queue_length_, _hints);
    }
    else
    {
        return std::make_shared<ImageSubscriber>(
            *it_, image_topic_names_[_index], queue_length_);
    }
}

void OverlayLidarMultiCamera::imageRos2cv(
    const sensor_msgs::ImageConstPtr& _msg_image,
    cv::Mat &_image)
{
    try
    {
        _image = cv_bridge::toCvShare(
            _msg_image, 
            sensor_msgs::image_encodings::RGB8)->image;
    }
    catch(const std::exception& e)
    {
        ROS_WARN("Unable to convert from ROS image to OpenCV image");
        ROS_WARN("%s", e.what());
    }
}

void OverlayLidarMultiCamera::projectAndColour(
    const std::vector<float> &C_r_CL,
    const std::vector<float> &C_q_CL,
    const sensor_msgs::CameraInfoConstPtr &msg_camera_info,
    const cv::Mat& image,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_coloured)
{
    // convert points from the lidar frame to the camera frame
    Eigen::Vector3d t_camera_lidar = Eigen::Vector3d(C_r_CL[0], C_r_CL[1], C_r_CL[2]); // tx, ty, tz
    Eigen::Quaterniond q_camera_lidar = Eigen::Quaterniond(C_q_CL[3], C_q_CL[0], C_q_CL[1], C_q_CL[2]); // w, x, y, z
    pcl::PointCloud<pcl::PointXYZRGB> cloud_camera;
    pcl::transformPointCloud(*cloud_coloured, cloud_camera, t_camera_lidar, q_camera_lidar);

    for(int i = 0; i < cloud_camera.points.size(); i++)
    {
        // project points into the image plane and colour it
	    int u = round((msg_camera_info->K[0] * cloud_camera.points[i].x / cloud_camera.points[i].z) + msg_camera_info->K[2]);
        int v = round((msg_camera_info->K[4] * cloud_camera.points[i].y / cloud_camera.points[i].z) + msg_camera_info->K[5]);

        // if the point is within bounds, then colour the original cloud
        if (u >= 0 && v > 0 && u < msg_camera_info->width && v < msg_camera_info->height && cloud_camera.points[i].z > 0 && cloud_camera.points[i].z < 256)
        {
            cloud_coloured->points[i].r = image.at<cv::Vec3b>(v, u)[0];
            cloud_coloured->points[i].g = image.at<cv::Vec3b>(v, u)[1];
            cloud_coloured->points[i].b = image.at<cv::Vec3b>(v, u)[2];
        }
    }
}

}