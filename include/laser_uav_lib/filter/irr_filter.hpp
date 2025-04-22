#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <tuple>

namespace laser_uav_lib
{

    class IIRFilter
    {

    public:
        /**
         * @brief Constructor of new IIRFilter object (coefficients are in the canonical form)
         *
         * @param a gains in the feedback branch of the IIR filter be advised that the coefficient a[0] is not used and should be set to zero.
         * @param b gains in the feedforward branch, using only this coefficients results in FIR filter
         * @param node_logger Optional ROS2 logger for output messages
         */
        IIRFilter(const std::vector<double> &a, const std::vector<double> &b,
                  const rclcpp::Logger &node_logger = rclcpp::get_logger("iir_filter"));
        IIRFilter(const rclcpp::Logger &node_logger = rclcpp::get_logger("iir_filter"));

        double iterate(const double input);

        std::tuple<std::vector<double>, std::vector<double>> getCoeffs();
        std::vector<double> getInputBuffer();
        std::vector<double> getOutputBuffer();

    private:
        std::vector<double> a_;
        std::vector<double> b_;
        size_t order_;
        std::vector<double> buffer_;
        std::vector<double> input_buffer_;
        std::vector<double> processed_output_buffer_;
        rclcpp::Logger logger_;
    };

} // namespace mrs_lib

#endif