#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

/* Include C++ libraries */
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

/* OpenCV includes */
#include <opencv2/core/core.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

/* ROS 2 includes for working with OpenCV */
#include <image_geometry/pinhole_camera_model.h>

/* Include geometry message */
#include <geometry_msgs/msg/point.hpp>

/* Include helper function from our library */
#include <laser_uav_lib/vision/image_utils.hpp>

namespace laser_uav_lib
{
  namespace vision
  {
    namespace image_processing
    {

      /**
       * @brief Finds landing platform in an image based on HSV color range.
       *
       * This function detects landing platforms in an input image based on the specified HSV color range.
       *
       * @param image The input image (cv::Mat).
       * @param lp_hsv_lb_ Lower bound of the HSV color range for landing platform detection (cv::Scalar).
       * @param lp_hsv_ub_ Upper bound of the HSV color range for landing platform detection (cv::Scalar).
       * @return A vector containing the 2D points representing the position of the detected landing platforms
       * (std::vector<cv::Point2d>).
       */
      std::vector<std::vector<cv::Point>> findLandingPlatform(cv::Mat *image, cv::Scalar lp_hsv_lb_, cv::Scalar lp_hsv_ub_);

      /**
       * @brief Detects QR code boxes in an image and returns their positions.
       *
       * This function identifies boxes containing QR codes in the given input image and returns the positions of these
       * boxes.
       *
       * @param image The input image (cv::Mat*).
       * @return A vector of points indicating the positions of the detected QR code boxes (laser_uav_lib::vision::image_utils::Detected).
       */
      std::vector<laser_uav_lib::vision::image_utils::Detected> findQRCode(cv::Mat *image, int erosion_count = 9, int structure_width = 11, int structure_height = 5);

      /**
       * \brief Function to find and read barcodes in an image.
       *
       * This function scans the input image to detect and read barcodes using the ZBar library.
       * It returns a vector of detected barcodes with their corresponding data and locations.
       *
       * \param image Pointer to the image where barcodes will be searched.
       * \return A vector of detected barcodes with their data and locations.
       */
      std::vector<laser_uav_lib::vision::image_utils::Detected> findBarcode(cv::Mat *image, int erosion_count = 23, int structure_width = 17, int structure_height = 5);

    } // namespace image_processing
  } // namespace vision
} // namespace laser_uav_lib

#endif // IMAGE_PROCESSING_H
