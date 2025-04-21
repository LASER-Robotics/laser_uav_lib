#include <laser_uav_lib/vision/image_processing.hpp>

namespace laser_uav_lib
{
  namespace vision
  {
    namespace image_processing
    {

      /**
       * @brief Finds landing platforms in an image based on HSV color range.
       *
       * This function detects landing platforms in an input image based on the specified HSV color range.
       *
       * @param image The input image (cv::Mat).
       * @param lp_hsv_lb_ Lower bound of the HSV color range for landing platform detection (cv::Scalar).
       * @param lp_hsv_ub_ Upper bound of the HSV color range for landing platform detection (cv::Scalar).
       * @return A vector containing the contours of the detected landing platforms
       * (std::vector<std::vector<cv::Point>>).
       **/
      /* findLandingPlatform() //{ */
      std::vector<std::vector<cv::Point>> findLandingPlatform(cv::Mat *image, cv::Scalar lp_hsv_lb_, cv::Scalar lp_hsv_ub_)
      {
        std::vector<std::vector<cv::Point>> lp_detect;

        /* blur image to reduce the noise */
        cv::Mat img_blur;
        cv::medianBlur(*image, img_blur, 3);

        /* convert to HSV */
        cv::Mat img_hsv;
        cv::cvtColor(img_blur, img_hsv, cv::COLOR_BGR2HSV);

        /* apply color segmentation using landing platform color */
        cv::Mat img_threshold;
        cv::inRange(img_hsv, lp_hsv_lb_, lp_hsv_ub_, img_threshold);

        /* search for circles in the image */
        std::vector<cv::Vec3f> found_circles;
        cv::HoughCircles(img_threshold, found_circles, cv::HOUGH_GRADIENT, 1, 67, 100, 40, 10, 300);

        /* find the contours */
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(img_threshold, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

        /* search for squares in the contours */
        std::vector<std::vector<cv::Point>> found_squares_contours;
        laser_uav_lib::vision::image_utils::findSquares(contours, found_squares_contours);

        /* for each square found */
        for (long unsigned int i = 0; i < found_squares_contours.size(); i++)
        {
          /* calculate the center of the square using moments */
          cv::Moments m = cv::moments(found_squares_contours[i]);
          cv::Point2f cv_square_ctr_img(m.m10 / (m.m00 + 1e-5), m.m01 / (m.m00 + 1e-5));

          /* try to find a circle with the same center of the square */
          for (long unsigned int j = 0; j < found_circles.size(); j++)
          {
            if (fabs(found_circles[j][0] - cv_square_ctr_img.x) < 5 && fabs(found_circles[j][1] - cv_square_ctr_img.y) < 5)
            {
              /* Calculate center landing platform */
              cv::Point2d cv_lp_ctr_img((found_circles[j][0] + cv_square_ctr_img.x) / 2,
                                        (found_circles[j][1] + cv_square_ctr_img.y) / 2);

              /* Add new landing platform to array */
              lp_detect.push_back(found_squares_contours[i]);

              /* remove circle from the vector to avoid unnecessary comparisons */
              found_circles.erase(found_circles.begin() + j);
            }
          }
        }
        return lp_detect;
      }
      // }

      /**
       * @brief Detects QR code boxes in an image and returns their positions.
       *
       * This function identifies boxes containing QR codes in the given input image and returns the positions of these boxes.
       *
       * @param image The input image (cv::Mat*).
       * @param erosion_count Number of erosion and dilation operations (default: 9).
       * @param structure_width Width of the structuring element for morphology operations (default: 11).
       * @param structure_height Height of the structuring element for morphology operations (default: 5).
       * @return A vector of 2D points indicating the positions of the detected QR code boxes (std::vector<cv::Point>).
       */
      /* findQRCode() //{ */
      std::vector<laser_uav_lib::vision::image_utils::Detected> findQRCode(cv::Mat *image, int erosion_count, int structure_width, int structure_height)
      {
        /* Create a vector of the points to return */
        std::vector<laser_uav_lib::vision::image_utils::Detected> box_detect;

        std::vector<std::vector<cv::Point>> contours = laser_uav_lib::vision::image_utils::findRectangles(image, erosion_count, structure_width, structure_height);

        // Create a copy of the input image
        cv::Mat image_aux = *image;
        // Vector to store bounding rectangles for each contour
        std::vector<cv::Rect> boundRect(contours.size());

        // Iterate over each contour
        for (long unsigned int i = 0; i < contours.size(); i++)
        {
          // Compute the bounding rectangle for each contour
          boundRect[i] = cv::boundingRect(contours[i]);

          // float max_side;
          // float min_side;

          // // Determine the larger and smaller sides of the rectangle
          // if (boundRect[i].width > boundRect[i].height)
          // {
          //   max_side = boundRect[i].width;
          //   min_side = boundRect[i].height;
          // }
          // else
          // {
          //   max_side = boundRect[i].height;
          //   min_side = boundRect[i].width;
          // }

          float ratio = float(boundRect[i].width) / boundRect[i].height;

          if (ratio > 0.8 && ratio < 1.2)
          {

            cv::Point qrcode_position = cv::Point(boundRect[i].x, boundRect[i].y);

            cv::rectangle(*image, boundRect[i], cv::Scalar(0, 255, 0));
            // int scale = 1.0;
            // Extract the region of interest (ROI) containing the potential qrcode
            cv::Mat image_box(image_aux, boundRect[i]);

            // int w, h;

            // Decode the qrcode and store the result
            laser_uav_lib::vision::image_utils::Detected barcodeDetected = laser_uav_lib::vision::image_utils::qrcodeDecoder(image_box);
            if (!barcodeDetected.data.empty())
            {
              barcodeDetected.points = contours[i];
              box_detect.push_back(barcodeDetected);
              // Draw the decoded qrcode data on the image
              cv::putText(*image, barcodeDetected.data, qrcode_position, cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(10, 10, 255), 2,
                          cv::LINE_AA);
            }
          }
        }

        return box_detect;
      }
      // }

      /**
       * \brief Function to find and read barcodes in an image.
       *
       * This function scans the input image to detect and read barcodes using the ZBar library.
       * It returns a vector of detected barcodes with their corresponding data and locations.
       *
       * @param image Pointer to the image where barcodes will be searched.
       * @return A vector of detected barcodes with their data and locations.
       */
      /* findAndReadBarcode() //{ */
      std::vector<laser_uav_lib::vision::image_utils::Detected> findBarcode(cv::Mat *image, int erosion_count, int structure_width, int structure_height)
      {
        // Rotate the image 90 degrees clockwise
        // cv::rotate(*image, *image, cv::ROTATE_90_CLOCKWISE);

        // int erosion_count = 23;   // Number of erosion and dilation operations
        // int structure_width = 17; // Width of the structuring element for morphology operations
        // int structure_height = 5; // Height of the structuring element for morphology operations

        // Find rectangles in the image and store their contours
        std::vector<std::vector<cv::Point>> contours = laser_uav_lib::vision::image_utils::findRectangles(image, erosion_count, structure_width, structure_height);

        // Create a copy of the input image
        cv::Mat image_aux = *image;
        // Vector to store bounding rectangles for each contour
        std::vector<cv::Rect> boundRect(contours.size());
        // Vector to store detected barcodes
        std::vector<laser_uav_lib::vision::image_utils::Detected> barcodeDetection;

        // Iterate over each contour
        for (long unsigned int i = 0; i < contours.size(); i++)
        {
          // Compute the bounding rectangle for each contour
          boundRect[i] = cv::boundingRect(contours[i]);

          // float max_side;
          // float min_side;

          // // Determine the larger and smaller sides of the rectangle
          // if (boundRect[i].width > boundRect[i].height)
          // {
          //   max_side = boundRect[i].width;
          //   min_side = boundRect[i].height;
          // }
          // else
          // {
          //   max_side = boundRect[i].height;
          //   min_side = boundRect[i].width;
          // }

          // // Calculate the aspect ratio of the bounding rectangle
          // float ratio = float(boundRect[i].width) / boundRect[i].height;

          // Optionally draw the contours of the rectangle
          // cv::drawContours(*image, boundRect[i], i, (255,0,0), 1);

          // BARCODE IS 10 x 6 (w x h)
          // Check if the aspect ratio is within the expected range for barcodes
          // if (ratio > 1.4 && ratio < 1.8)
          // {
          // int scale = 1.0;
          // Extract the region of interest (ROI) containing the potential barcode
          cv::Mat image_box(image_aux, boundRect[i]);
          cv::Mat image_box_(image_aux, boundRect[i]);

          // int w, h;

          // // Determine the width and height of the ROI
          // if (image_box.size().height > image_box.size().width)
          // {
          //   h = image_box.size().height;
          //   w = image_box.size().width;
          // }
          // else
          // {
          //   h = image_box.size().width;
          //   w = image_box.size().height;
          // }

          // Optionally put text on the image displaying the dimensions of the ROI
          // cv::putText(*image, cv::String(std::to_string(w) + "x" + std::to_string(h)), cv::Point(boundRect[i].x,
          // boundRect[i].y), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(10, 10, 255), 2, cv::LINE_AA);
          // Draw the bounding rectangle on the image
          cv::rectangle(*image, boundRect[i], cv::Scalar(0, 255, 0));

          // Get the position of the barcode
          cv::Point barcode_position = cv::Point(boundRect[i].x, boundRect[i].y);

          // Ignore small rectangles
          // if (w > 0 && h > 0 && (w < 120 || h < 70))
          // {
          // Optionally print the width and height
          // std::cout << w << ", " << h << "\n";
          // COUNT # OF PARALLEL LINES IN IMAGE. IF ITS HIGH, ITS PROBABLY A BARCODE
          // int number_of_parallel_lines = detectLines(image_box);
          // if(number_of_parallel_lines > 20){

          // Decode the barcode and store the result
          std::string detectString = laser_uav_lib::vision::image_utils::barcodeDecoder(image_box);
          laser_uav_lib::vision::image_utils::Detected barcodeDetected;
          if (!detectString.empty())
          {
            barcodeDetected.data = detectString;
            barcodeDetected.points = contours[i];

            barcodeDetection.push_back(barcodeDetected);
            // Draw the decoded barcode data on the image
            cv::putText(*image, detectString, barcode_position, cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(10, 10, 255), 2,
                        cv::LINE_AA);
          }
          // }
          // }
        }
        // Return the detected barcodes
        return barcodeDetection;
      }

      // }
    } // namespace image_processing
  } // namespace vision
} // namespace laser_uav_lib
