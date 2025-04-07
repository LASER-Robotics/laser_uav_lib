#include <laser_uav_lib/pcl_utils/pointcloud_utils.hpp>

namespace laser_uav_lib
{
    namespace pcl_utils
    {
        namespace pointcloud_utils
        {
            /**
             * @brief Converts a depth image to a point cloud.
             *
             * @param depth_image The depth image (cv::Mat).
             * @param camera_model_ The camera model (image_geometry::PinholeCameraModel).
             * @param depth_scale The scale of depth values (double).
             * @return The point cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr).
             */
            /* imageToPointCloud() //{ */
            pcl::PointCloud<pcl::PointXYZ>::Ptr imageToPointCloud(const cv::Mat &depth_image,
                                                                  const image_geometry::PinholeCameraModel &camera_model_,
                                                                  double depth_scale)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

                // Parâmetros intrínsecos da câmera
                double fx = camera_model_.fx(); // fx
                double fy = camera_model_.fy(); // fy
                double cx = camera_model_.cx(); // cx
                double cy = camera_model_.cy(); // cy

                for (int x = 0; x < depth_image.cols; ++x)
                {
                    for (int y = 0; y < depth_image.rows; ++y)
                    {
                        double depth_value = depth_image.at<u_int16_t>(y, x);
                        if (depth_value != 0)
                        { // Ignora pontos inválidos
                            pcl::PointXYZ point;
                            point.z = depth_value / depth_scale;
                            point.x = (x - cx) * point.z / fx;
                            point.y = (y - cy) * point.z / fy;
                            cloud->points.push_back(point);
                        }
                    }
                }

                cloud->width = cloud->points.size();
                cloud->height = 1;
                cloud->is_dense = true;

                return cloud;
            }
            //}

            /**
             * @brief Joins two point clouds.
             *
             * @param cloud1 The first point cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr).
             * @param cloud2 The second point cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr).
             * @param transform The transformation matrix to apply to cloud2 (Eigen::Matrix4f).
             * @return The joined point cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr).
             */
            /* joinPointClouds() //{ */
            pcl::PointCloud<pcl::PointXYZ>::Ptr joinPointClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
                                                                const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2,
                                                                const Eigen::Matrix4f &transform)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr joined_cloud(new pcl::PointCloud<pcl::PointXYZ>);

                // Transforming the second point cloud using the specified transformation
                pcl::transformPointCloud(*cloud2, *joined_cloud, transform);

                // Adding the first point cloud to the transformed cloud
                *joined_cloud += *cloud1;

                return joined_cloud;
            }
            //}

            /**
             * @brief Converts a PCL point cloud to a ROS PointCloud2 message.
             *
             * @param pclPointCloud The PCL point cloud to be converted (pcl::PointCloud<pcl::PointXYZ>).
             * @return The converted ROS PointCloud2 message (sensor_msgs::PointCloud2).
             */
            /* pclToPointCloud2() //{ */
            sensor_msgs::msg::PointCloud2 pclToPointCloud2(const pcl::PointCloud<pcl::PointXYZ> &pclPointCloud)
            {
                sensor_msgs::msg::PointCloud2 rosPointCloud;
                pcl::toROSMsg(pclPointCloud, rosPointCloud);
                return rosPointCloud;
            }
            //}

            /**
             * @brief Converts a ROS PointCloud2 message to a PCL point cloud.
             *
             * @param rosPointCloud The ROS PointCloud2 message to be converted (sensor_msgs::PointCloud2).
             * @return The converted PCL point cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr).
             */
            /* pointCloud2ToPcl() //{ */
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud2ToPcl(const sensor_msgs::msg::PointCloud2 &rosPointCloud)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(rosPointCloud, *pclPointCloud);
                return pclPointCloud;
            }
            //}

            /**
             * @brief Computes the distance from the camera to the centroid of the input point cloud.
             *
             * @param cloud The input point cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr).
             * @return The distance to the cloud (geometry_msgs::msg::Point).
             */
            /* distanceToCloud() //{ */
            geometry_msgs::msg::Point distanceToCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
            {
                pcl::PointXYZ centroid;
                pcl::computeCentroid(*cloud, centroid);

                // The distance from the camera to the centroid is simply the Z coordinate of the centroid
                geometry_msgs::msg::Point point;
                point.x = centroid.x;
                point.y = centroid.y;
                point.z = centroid.z;

                return point;
            }
        } // namespace pointcloud_utils
    } // namespace pcl_utils
} // namespace laser_uav_lib
