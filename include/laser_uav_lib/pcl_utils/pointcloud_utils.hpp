#ifndef POINTCLOUD_UTILS_H
#define POINTCLOUD_UTILS_H

/* include sensor_msgs for PointCloud2 */
#include <sensor_msgs/msg/point_cloud2.hpp>

/* include PCL for point cloud data types */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/io/image_depth.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

/* include geometry message */
#include <geometry_msgs/msg/point.hpp>

/* OpenCV includes for image processing */
#include <opencv2/core/core.hpp>

/* ROS includes for camera model */
#include <image_geometry/pinhole_camera_model.h>

namespace laser_uav_lib
{
    namespace pcl_utils
    {
        namespace pointcloud_utils
        {
            /**
             * @brief Converts a PCL point cloud to a ROS PointCloud2 message.
             *
             * @param pclPointCloud The PCL point cloud to be converted (pcl::PointCloud<pcl::PointXYZ>).
             * @return The converted ROS PointCloud2 message (sensor_msgs::msg::PointCloud2).
             */
            sensor_msgs::msg::PointCloud2 pclToPointCloud2(const pcl::PointCloud<pcl::PointXYZ> &pclPointCloud);

            /**
             * @brief Converts a ROS PointCloud2 message to a PCL point cloud.
             *
             * @param rosPointCloud The ROS PointCloud2 message to be converted (sensor_msgs::msg::PointCloud2).
             * @return The converted PCL point cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr).
             */
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud2ToPcl(const sensor_msgs::msg::PointCloud2 &rosPointCloud);

            /**
             * @brief Computes the distance from the camera to the centroid of the input point cloud.
             *
             * @param cloud The input point cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr).
             * @return The distance to the cloud (geometry_msgs::msg::Point).
             */
            geometry_msgs::msg::Point distanceToCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

            /**
             * @brief Converts an image to a point cloud.
             *
             * @param depth_image The depth image (cv::Mat).
             * @param camera_model_ The camera model (image_geometry::PinholeCameraModel).
             * @param depth_scale The depth scale (double).
             * @return The point cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr).
             */
            pcl::PointCloud<pcl::PointXYZ>::Ptr imageToPointCloud(const cv::Mat &depth_image,
                                                                  const image_geometry::PinholeCameraModel &camera_model_,
                                                                  double depth_scale);

            /**
             * @brief Joins two point clouds.
             *
             * @param cloud1 The first point cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr).
             * @param cloud2 The second point cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr).
             * @param transform The transformation matrix to apply to cloud2 (Eigen::Matrix4f).
             * @return The joined point cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr).
             */
            pcl::PointCloud<pcl::PointXYZ>::Ptr joinPointClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
                                                                const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2,
                                                                const Eigen::Matrix4f &transform);

            /**
             * @brief Projects a pixel into 3D space.
             *
             * @param unrect_px The pixel to be projected (cv::Point2d).
             * @param px_dist The depth of the pixel (double).
             * @param cam_info The camera information (image_geometry::PinholeCameraModel).
             * @return The projected 3D point (geometry_msgs::msg::Point).
             */
            geometry_msgs::msg::Point projectPixelTo3d(const cv::Point2d &unrect_px, const double px_dist,
                                                       const image_geometry::PinholeCameraModel cam_model);

        } // namespace pointcloud_utils
    } // namespace vision
} // namespace laser_vision

#endif // POINTCLOUD_UTILS_H