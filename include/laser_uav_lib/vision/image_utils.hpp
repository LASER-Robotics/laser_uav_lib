#ifndef IMAGE_UTILS_H
#define IMAGE_UTILS_H

/* OpenCV includes */
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

/* ROS includes for working with OpenCV */
#include <image_geometry/pinhole_camera_model.h>

/* include geometry message */
#include <geometry_msgs/msg/point.hpp>

/* include Eigen */
#include <Eigen/Dense>

/* include ZBAR library */
#include <zbar.h>

namespace laser_uav_lib
{
  namespace vision
  {
    namespace image_utils
    {
      /**
       * @struct barcode_detect
       * @brief Structure to store the detected barcode data and its point.
       */
      struct barcode_detect
      {
        cv::Point2d point; ///< 2D point where the barcode was detected.
        std::string data;  ///< Barcode data.
      };

      /**
       * @struct Detected
       * @brief Armazena dados decodificados e pontos de contorno de um QR code.
       *
       * Esta struct mantém informações sobre um QR code detectado em uma imagem.
       *
       * @var Detected::data
       * Dados decodificados do QR code.
       *
       * @var Detected::points
       * Pontos de contorno do QR code, definindo sua forma na imagem.
       */
      struct Detected
      {
        std::string data;              /** Dados decodificados do QR code. */
        std::vector<cv::Point> points; /** Pontos de contorno do QR code. */
      };

      /**
       * @brief Function to detect parallel lines in a vector of lines.
       *
       * @param lines Vector of lines (cv::Vec4i) to analyze for parallelism.
       * @return The number of pairs of parallel lines detected.
       */
      int detectParallelLines(std::vector<cv::Vec4i> lines);

      /**
       * @brief Function to detect lines in an image.
       *
       * @param image Image where the lines will be detected.
       * @return The number of parallel lines detected.
       */
      int detectLines(cv::Mat image);

      /**
       * @brief Function to search for squares in an array of contours.
       *
       * Source: https://docs.opencv.org/3.4/de/dc0/samples_2tapi_2squares_8cpp-example.html
       *
       * @param contours The array of contours (std::vector<std::vector<cv::Point>>).
       * @param squares_contour Output parameter to store squares found (std::vector<std::vector<cv::Point>>&).
       */
      void findSquares(const std::vector<std::vector<cv::Point>> contours,
                       std::vector<std::vector<cv::Point>> &squares_contour);

      /**
       * @brief Creates a mask from a set of points.
       *
       * @param points The points defining the contour of the mask (std::vector<cv::Point>).
       * @param imageShape The shape of the image (cv::Size).
       * @return The mask image (cv::Mat).
       */
      cv::Mat createMaskFromPoints(const std::vector<cv::Point> &points, const cv::Size &imageShape);

      /**
       * @brief Function to decode a barcode in an image.
       *
       * @param image Image where the barcode will be decoded.
       * @return A structure containing the decoded barcode's data and location.
       */
      Detected qrcodeDecoder(cv::Mat image);

      /**
       * \brief Function to find rectangles in an image.
       *
       * This function processes the input image to detect rectangles.
       * It returns a vector of vectors, where each vector contains points representing a detected rectangle.
       *
       * \param image Pointer to the image where rectangles will be detected.
       * \return A vector of vectors, each containing points of a detected rectangle.
       */
      std::vector<std::vector<cv::Point>> findRectangles(cv::Mat *image, int erosion_count, int structure_width, int structure_height);

      /**
       * \brief Function to decode a barcode in an image.
       *
       * This function processes the input image to detect and decode a barcode.
       * It returns a structure containing the detected barcode's data and location.
       *
       * \param image Image where the barcode will be decoded.
       * \return A structure containing the decoded barcode's data and location.
       */
      std::string barcodeDecoder(cv::Mat image);

      /**
       * @brief Calculate the midpoint of a set of points.
       *
       * @param points The set of points.
       * @return The midpoint.
       */
      cv::Point calculateMidpoint(const std::vector<cv::Point> &points);

      /**
       * @brief Calculate the angle between two 3D vectors.
       * This function computes the angle between two 3D vectors using the cross product and dot product.
       *
       * @param vec1 The first vector (Eigen::Matrix<double, dims, 1><3>).
       * @param vec2 The second vector (Eigen::Matrix<double, dims, 1><3>).
       * @return The angle between the two vectors in radians (double).
       */
      double angleBetween(const Eigen::Matrix<double, 3, 1> &vec1, const Eigen::Matrix<double, 3, 1> &vec2);

      /**
       * @brief Calculate the angle between two 2D vectors.
       * This function computes the angle between two 2D vectors using the cross product and dot product.
       *
       * @param vec1 The first vector (Eigen::Matrix<double, dims, 1><2>).
       * @param vec2 The second vector (Eigen::Matrix<double, dims, 1><2>).
       * @return The angle between the two vectors in radians (double).
       */
      double angleBetween(const Eigen::Matrix<double, 2, 1> &vec1, const Eigen::Matrix<double, 2, 1> &vec2);

    } // namespace image_utils
  } // namespace vision
} // namespace laser_vision

#endif // IMAGE_UTILS_H