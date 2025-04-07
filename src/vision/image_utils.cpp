#include <laser_uav_lib/vision/image_utils.hpp>

namespace laser_uav_lib
{
  namespace vision
  {
    namespace image_utils
    {
      /**
       * @brief Projects a pixel to a 3D point using the camera model.
       *
       * @param unrect_px The pixel in unrectified coordinates (cv::Point2d).
       * @param px_dist The depth value of the pixel (double).
       * @param cam_model The camera model (image_geometry::PinholeCameraModel).
       * @return The 3D point (geometry_msgs::msg::Point).
       */
      /* projectPixelTo3d() //{ */
      geometry_msgs::msg::Point projectPixelTo3d(const cv::Point2d &unrect_px, const double px_dist,
                                                 const image_geometry::PinholeCameraModel cam_model)
      {
        cv::Point3d cv_pt3d_cam;

        // rectify the point using camera parameters
        cv::Point2d cv_pt2d_px_rect = cam_model.rectifyPoint(unrect_px);

        // Project the rectified pixel to 3D
        cv::Matx33d cam_intrisic_matrix = cam_model.intrinsicMatrix();

        cv_pt3d_cam.x = (cv_pt2d_px_rect.x - cam_intrisic_matrix(0, 2)) * px_dist / cam_intrisic_matrix(0, 0);
        cv_pt3d_cam.y = (cv_pt2d_px_rect.y - cam_intrisic_matrix(1, 2)) * px_dist / cam_intrisic_matrix(1, 1);
        cv_pt3d_cam.z = px_dist;

        // Convert cv::Point3d to geometry_msgs::msg::Point directly
        geometry_msgs::msg::Point pt3d_cam;
        pt3d_cam.x = cv_pt3d_cam.x;
        pt3d_cam.y = cv_pt3d_cam.y;
        pt3d_cam.z = cv_pt3d_cam.z;

        return pt3d_cam;
      }
      //}

      /**
       * @brief Finds squares in an array of contours.
       *
       * This function searches for squares in an array of contours. A square contour is characterized by having:
       * - An approximation with four vertices.
       * - A relatively large area (to filter out noisy contours).
       * - Convexity.
       * - An angle between the joint edges closer to 90 degrees.
       * - Solidity closer to 1.
       *
       * @param contours Array of contours (std::vector<std::vector<cv::Point>>).
       * @param squares_contour Vector to store found square contours (std::vector<std::vector<cv::Point>>&).
       */
      /* findSquares() //{ */
      void findSquares(const std::vector<std::vector<cv::Point>> contours,
                       std::vector<std::vector<cv::Point>> &squares_contour)
      {
        /**
         * * a square contour should have:
         * feature 1 - the approximation has four vertices,
         * feature 2 - a relatively large area (to filter out noisy contours),
         * feature 3 - be convex,
         * feature 4 - the angle between the joint edges closer to 90 degrees,
         * feature 5 - solidity closer to 1;
         */

        /* for each contour */
        for (unsigned long int i = 0; i < contours.size(); i++)
        {
          /**
           * calculate an approximation contour with perimeter closer to the original contour
           **/

          double orig_contour_peri = cv::arcLength(contours[i], true);
          double approx_contour_peri = orig_contour_peri * 0.05;
          std::vector<cv::Point> approx;
          cv::approxPolyDP(contours[i], approx, approx_contour_peri, true);

          //}

          /* //{ calculate solidity */

          double approx_area = cv::contourArea(approx);
          double solidity = cv::contourArea(contours[i]) / approx_area;

          //}

          /* validate features 1, 2, 3 and 5 */
          if (fabs(solidity - 1) < 0.1 && approx.size() == 4 && fabs(approx_area) > 100 && cv::isContourConvex(approx))
          {
            double max_cos = 0;
            for (int j = 2; j < 5; j++)
            {
              const Eigen::Vector2d vec1(approx[j % 4].x - approx[j - 1].x, approx[j % 4].y - approx[j - 1].y);

              const Eigen::Vector2d vec2(approx[j - 2].x - approx[j - 1].x, approx[j - 2].y - approx[j - 1].y);

              max_cos = std::max(max_cos, cos(laser_uav_lib::vision::image_utils::angleBetween(vec1, vec2)));
            }

            /* validate feature 4 */
            if (max_cos < 0.3)
            {
              squares_contour.push_back(approx);
            }
          }
        }
      }
      //}

      /**
       * @brief Function to find rectangles in an image.
       *
       * This function processes the input image to detect rectangles.
       * It returns a vector of vectors, where each vector contains points representing a detected rectangle.
       *
       * Configurable parameters:
       *
       * erosion_count           ->   Number of erosions and dilations that help find rectangles in the image. Activate the
       * structure_width/height   ->   Defines the dimensions of the rectangles to be identified in the image.
       * imshow in findRectangles.
       *
       * @param image Pointer to the image where rectangles will be detected.
       * @return A vector of vectors, each containing points of a detected rectangle.
       */
      /* findRectangles() //{ */
      std::vector<std::vector<cv::Point>> findRectangles(cv::Mat *image, int erosion_count, int structure_width, int structure_height)
      {
        // The image is assumed to be in grayscale
        int ksize = -1; // Kernel size for the Sobel operator

        // Create a copy of the input image
        cv::Mat image_aux = *image;

        // Declare Mat objects for gradient computations and image processing
        cv::Mat grad_x, grad_y;
        cv::Mat abs_grad_x, abs_grad_y, grad, blur, thresh, gray;

        // Convert the image to grayscale
        cv::cvtColor(image_aux, gray, cv::COLOR_BGR2GRAY);

        // Compute the gradient in the x direction using the Sobel operator
        cv::Sobel(gray, grad_x, CV_16S, 1, 0, ksize);
        // Compute the gradient in the y direction using the Sobel operator
        cv::Sobel(gray, grad_y, CV_16S, 0, 1, ksize);

        // Subtract the y-gradient from the x-gradient to highlight vertical edges
        cv::subtract(grad_x, grad_y, grad);
        // Convert the gradient image to 8-bit for further processing
        cv::convertScaleAbs(grad, grad);

        // Apply a median blur to the gradient image to reduce noise
        cv::medianBlur(grad, grad, 3);

        // Apply binary thresholding to the gradient image
        cv::threshold(grad, grad, 225, 255, cv::THRESH_BINARY);

        // Create a rectangular structuring element for morphological operations
        cv::Mat kernel =
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(structure_width, structure_height), cv::Point(-1, -1));
        // Apply morphological closing (dilation followed by erosion)
        cv::morphologyEx(grad, grad, cv::MORPH_CLOSE, kernel);
        // Apply morphological opening (erosion followed by dilation)
        cv::morphologyEx(grad, grad, cv::MORPH_OPEN, kernel);

        // Optionally display the result of morphological operations
        // cv::imshow("morphology", grad);

        // Structuring elements for erosion and dilation
        cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat element_dilation = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

        // Perform a series of erosions
        cv::erode(grad, grad, element_erode, cv::Point(-1, -1), erosion_count);
        // Perform a series of dilations
        cv::dilate(grad, grad, element_dilation, cv::Point(-1, -1), erosion_count);

        // Optionally display the result of erosion and dilation
        // cv::imshow("erosion/dilation", grad);

        // Declare vectors to store contours and hierarchy information
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        // Find contours in the processed image
        cv::findContours(grad, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Return the detected contours
        return contours;
      }
      //}

      /**
       * @brief Function to decode a barcode in an image.
       *
       * This function processes the input image to detect and decode a barcode.
       * It returns a structure containing the detected barcode's data and location.
       *
       * @param image Image where the barcode will be decoded.
       * @return A structure containing the decoded barcode's data and location.
       */
      /* barcodeDecoder() //{ */
      laser_uav_lib::vision::image_utils::Detected qrcodeDecoder(cv::Mat image)
      {
        /* Configure zbar scanner */
        zbar::ImageScanner zbar_scanner_;
        zbar_scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        zbar_scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

        /* Create a backup of the original image */
        cv::Mat original_image = image;

        // Declare Mat objects for grayscale, binary threshold, and temporary image storage
        cv::Mat grayscaleImage, image_box_;
        cv::Mat threshImage;

        // Initialize to hold the detected barcode data
        std::string detect;

        // Convert the original image to grayscale
        cv::cvtColor(original_image, grayscaleImage, cv::COLOR_BGR2GRAY);

        // // Apply binary thresholding using Otsu's method
        // cv::threshold(grayscaleImage, threshImage, 225, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        // Uncomment the following line if you want to use the thresholded image instead of the grayscale image
        // grayscaleImage = threshImage;

        // Assign the grayscale image to a temporary Mat object
        // image_box_ = grayscaleImage;

        // Define a scaling factor (currently set to 1.0, meaning no scaling)
        float scale = 1.0;

        // Resize the grayscale image and the original image based on the scaling factor
        cv::resize(grayscaleImage, grayscaleImage,
                   cv::Size(grayscaleImage.size().width * scale, grayscaleImage.size().height * scale), cv::INTER_LINEAR);
        cv::resize(original_image, original_image,
                   cv::Size(original_image.size().width * scale, original_image.size().height * scale), cv::INTER_LINEAR);

        // Display the grayscale image in a window named "box"
        // cv::imshow("box", grayscaleImage);

        /* Wrap the cv::Image in a zbar::Image */
        zbar::Image zbarImage(grayscaleImage.cols, grayscaleImage.rows, "Y800", grayscaleImage.data,
                              grayscaleImage.cols * grayscaleImage.rows);

        /* Scan the image looking for qrcodes */
        zbar_scanner_.scan(zbarImage);

        laser_uav_lib::vision::image_utils::Detected Qrcode_box;

        // Iterate over all detected symbols (qrcodes) in the image
        for (zbar::Image::SymbolIterator symbol = zbarImage.symbol_begin(); symbol != zbarImage.symbol_end(); ++symbol)
        {
          /* for each QR-code detection */
          if (symbol->get_location_size() >= 2)
          {
            Qrcode_box.data = symbol->get_data();

            std::vector<cv::Point> rect_points;

            /* //{ calculate the center of the box using the border points */
            for (int counter = 0; counter < symbol->get_location_size(); counter++)
            {
              rect_points.push_back(cv::Point(symbol->get_location_x(counter), symbol->get_location_y(counter)));
            }
            Qrcode_box.points = rect_points;
          }
        }
        // Return the struct containing the detected barcode data and its position
        return Qrcode_box;
      }
      //}

      /**
       * @brief Function to decode a barcode in an image.
       *
       * This function processes the input image to detect and decode a barcode.
       * It returns a structure containing the detected barcode's data and location.
       *
       * @param image Image where the barcode will be decoded.
       * @return A structure containing the decoded barcode's data and location.
       */
      /* barcodeDecoder() //{ */
      std::string barcodeDecoder(cv::Mat image)
      {
        /* Configure zbar scanner */
        zbar::ImageScanner zbar_scanner_;
        zbar_scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        zbar_scanner_.set_config(zbar::ZBAR_CODE39, zbar::ZBAR_CFG_ENABLE, 1);

        /* Create a backup of the original image */
        cv::Mat original_image = image;

        // Declare Mat objects for grayscale, binary threshold, and temporary image storage
        cv::Mat grayscaleImage, image_box_;
        cv::Mat threshImage;

        // Initialize to hold the detected barcode data
        std::string detect;

        // Rotate the image 90 degrees clockwise if it is in portrait mode
        if (image.rows > image.cols)
        {
          cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
        }

        // Convert the original image to grayscale
        cv::cvtColor(original_image, grayscaleImage, cv::COLOR_BGR2GRAY);

        // Apply binary thresholding using Otsu's method
        cv::threshold(grayscaleImage, threshImage, 225, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        // Uncomment the following line if you want to use the thresholded image instead of the grayscale image
        // grayscaleImage = threshImage;

        // Assign the grayscale image to a temporary Mat object
        image_box_ = grayscaleImage;

        // Define a scaling factor (currently set to 1.0, meaning no scaling)
        float scale = 1.0;

        // Resize the grayscale image and the original image based on the scaling factor
        cv::resize(grayscaleImage, grayscaleImage,
                   cv::Size(grayscaleImage.size().width * scale, grayscaleImage.size().height * scale), cv::INTER_LINEAR);
        cv::resize(original_image, original_image,
                   cv::Size(original_image.size().width * scale, original_image.size().height * scale), cv::INTER_LINEAR);

        // Display the grayscale image in a window named "box"
        // cv::imshow("box", grayscaleImage);

        /* Wrap the cv::Image in a zbar::Image */
        zbar::Image zbarImage(original_image.cols, original_image.rows, "Y800", grayscaleImage.data,
                              original_image.cols * original_image.rows);

        /* Scan the image looking for barcodes */
        zbar_scanner_.scan(zbarImage);

        // Iterate over all detected symbols (barcodes) in the image
        for (zbar::Image::SymbolIterator symbol = zbarImage.symbol_begin(); symbol != zbarImage.symbol_end(); ++symbol)
        {
          // Extract the barcode data from the symbol
          std::string barcodeData = symbol->get_data();

          // Assign the extracted barcode data and position to the detect struct
          detect = barcodeData;
        }

        // Return the struct containing the detected barcode data and its position
        return detect;
      }
      //}

      /**
       * @brief Calculate the angle between two 3D vectors.
       * This function computes the angle between two 3D vectors using the cross product and dot product.
       *
       * @param vec1 The first vector (Eigen::Vector3d).
       * @param vec2 The second vector (Eigen::Vector3d).
       * @return The angle between the two vectors in radians (double).
       */
      /* angleBetween() //{ */
      double angleBetween(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2)
      {
        const double sin_12 = vec1.cross(vec2).norm();
        const double cos_12 = vec1.dot(vec2);
        const double angle = std::atan2(sin_12, cos_12);
        return angle;
      }
      /*//}*/

      /**
       * @brief Calculate the angle between two 2D vectors.
       * This function computes the angle between two 2D vectors using the cross product and dot product.
       *
       * @param vec1 The first vector (Eigen::Vector2d).
       * @param vec2 The second vector (Eigen::Vector2d).
       * @return The angle between the two vectors in radians (double).
       */
      /* angleBetween() //{ */
      double angleBetween(const Eigen::Vector2d &vec1, const Eigen::Vector2d &vec2)
      {
        const double sin_12 = vec1.x() * vec2.y() - vec1.y() * vec2.x();
        const double cos_12 = vec1.dot(vec2);
        const double angle = std::atan2(sin_12, cos_12);
        return angle;
      }
      /*//}*/

      /**
       * @brief Function to find barcodes in an image.
       *
       * This function scans the input image to detect barcodes using the ZBar library.
       * It returns a vector of detected barcodes with their corresponding data and locations.
       *
       * @param image Pointer to the image where barcodes will be searched.
       * @return A vector of detected barcodes with their data and locations.
       */
      /* findBarcode() //{ */
      std::vector<laser_uav_lib::vision::image_utils::barcode_detect> findBarcode(cv::Mat *image)
      {
        /* Create list for barcode detection */
        std::vector<laser_uav_lib::vision::image_utils::barcode_detect> barcodeDetection;

        /* Configure ZBar scanner */
        zbar::ImageScanner zbar_scanner_;
        zbar_scanner_.set_config(zbar::ZBAR_CODE39, zbar::ZBAR_CFG_ENABLE, 1);

        /* Create backup image */
        cv::Mat original_image = *image;

        cv::Mat grayscaleImage;

        /* Convert image to grayscale */
        cv::cvtColor(original_image, grayscaleImage, cv::COLOR_BGR2GRAY);

        /* Wrap the cv::Image in a zbar::Image */
        zbar::Image zbarImage(original_image.cols, original_image.rows, "Y800", grayscaleImage.data,
                              original_image.cols * original_image.rows);

        /* Scan image looking for barcodes */
        zbar_scanner_.scan(zbarImage);
        cv::Point center = cv::Point(original_image.cols / 2, original_image.rows / 2);

        /* Iterate over detected symbols */
        for (zbar::Image::SymbolIterator symbol = zbarImage.symbol_begin(); symbol != zbarImage.symbol_end(); ++symbol)
        {
          // Extract barcode data
          std::string barcodeData = symbol->get_data();
          cv::putText(*image, barcodeData, center, cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(10, 10, 255), 2, cv::LINE_AA);
          laser_uav_lib::vision::image_utils::barcode_detect detect;
          detect.data = barcodeData;
          detect.point = center;
          barcodeDetection.push_back(detect);
        }

        return barcodeDetection;
      } //}

      /**
       * @brief Function to detect lines in an image.
       *
       * This function processes the input image to detect lines using the Canny edge detector
       * and the Hough Line Transform. It returns the number of parallel lines detected.
       *
       * @param image Image where the lines will be detected.
       * @return The number of parallel lines detected.
       */
      /* detectLines() //{ */
      int detectLines(cv::Mat image)
      {
        int parallel_lines = 0;

        cv::Mat image_aux;

        // Convert image to grayscale
        cv::cvtColor(image, image_aux, cv::COLOR_BGR2GRAY);

        // Smooth the image
        cv::GaussianBlur(image_aux, image_aux, cv::Size(3, 3), 0);

        // Detect edges
        int lowThreshold = 50;
        int highThreshold = 150;
        const int kernel_size = 3;
        cv::Canny(image_aux, image_aux, lowThreshold, highThreshold, kernel_size);

        int w = image_aux.size().width;
        int h = image_aux.size().height;

        // Use Hough Transform to find lines in the edge image
        double rho = 1.0;
        double theta = CV_PI / 180;
        int threshold = 20;
        double minLineLength = std::ceil(std::min(w, h) * 0.1); // Empirical parameter to detect barcodes
        double maxLineGap = 1;

        std::vector<cv::Vec4i> linesP;
        cv::HoughLinesP(image_aux, linesP, rho, theta, threshold, minLineLength, maxLineGap);

        if (!linesP.empty())
        {
          parallel_lines = laser_uav_lib::vision::image_utils::detectParallelLines(linesP);
        }

        return parallel_lines;
      } //}

      /**
       * @brief Function to detect parallel lines in a vector of lines.
       *
       * This function analyzes a vector of lines to determine how many pairs of parallel lines exist.
       * It is used to verify if the detected lines could form a barcode pattern.
       *
       * @param lines Vector of lines (cv::Vec4i) to analyze for parallelism.
       * @return The number of pairs of parallel lines detected.
       */
      /* detectParallelLines() //{ */
      int detectParallelLines(std::vector<cv::Vec4i> lines) //{
      {
        std::vector<cv::Vec2i> lines_parallel; // Vector to store pairs of parallel lines
        int parallel_lines_count = 0;          // Counter for parallel lines
        double margin = 0.1;                   // Margin of error to consider lines as parallel

        // Double loop to compare each line with all other lines
        for (size_t i = 0; i < lines.size(); i++)
        {
          for (size_t j = 0; j < lines.size(); j++)
          {
            if (i != j) // Ensure the line is not compared with itself
            {
              // Calculate the slope (m1) of line i
              double num1 = lines[i][3] - lines[i][1];   // y2 - y1
              double den1 = lines[i][2] - lines[i][0];   // x2 - x1
              double m1 = (den1 != 0) ? num1 / den1 : 0; // Slope m1 (avoid division by zero)

              // Calculate the slope (m2) of line j
              double num2 = lines[j][3] - lines[j][1];   // y2 - y1
              double den2 = lines[j][2] - lines[j][0];   // x2 - x1
              double m2 = (den2 != 0) ? num2 / den2 : 0; // Slope m2 (avoid division by zero)

              // Check if the difference between the slopes is within the margin
              if (std::abs(m1 - m2) < margin)
              {
                lines_parallel.push_back(cv::Vec2i(i, j)); // Add the pair of indices of the parallel lines
              }
            }
          }
        }

        // If there are detected parallel lines, update the counter
        if (!lines_parallel.empty())
        {
          parallel_lines_count = lines_parallel.size();
        }

        return parallel_lines_count; // Return the number of detected parallel lines
      } //}

      /**
       * @brief Calculate the midpoint of a set of points.
       *
       * @param points The set of points.
       * @return The midpoint.
       */
      /* calculateMidpoint() //{ */
      cv::Point calculateMidpoint(const std::vector<cv::Point> &points)
      {
        cv::Point midpoint(0, 0);
        int totalPoints = points.size();

        // Sum all coordinates of the points
        for (const auto &point : points)
        {
          midpoint.x += point.x;
          midpoint.y += point.y;
        }

        // Divide by the number of points to get the average
        midpoint.x /= totalPoints;
        midpoint.y /= totalPoints;

        return midpoint;
      }

      /**
       * @brief Creates a mask from a set of points.
       *
       * @param points The points defining the contour of the mask (std::vector<cv::Point>).
       * @param imageShape The shape of the image (cv::Size).
       * @return The mask image (cv::Mat).
       */
      /* createMaskFromPoints() //{ */
      cv::Mat createMaskFromPoints(const std::vector<cv::Point> &points, const cv::Size &imageShape)
      {
        cv::Mat mask = cv::Mat::zeros(imageShape, CV_16UC1);
        std::vector<std::vector<cv::Point>> contours;
        contours.push_back(points);
        cv::drawContours(mask, contours, 0, cv::Scalar(65535), cv::FILLED); // Preenchendo com 16 bits (2^16 - 1)
        return mask;
      }
      //}

    } // namespace image_utils
  } // namespace vision
} // namespace laser_uav_lib
