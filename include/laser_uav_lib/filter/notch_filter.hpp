#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp> // Changed from ros/ros.h

#include <laser_uav_lib/filter/irr_filter.hpp>

namespace laser_uav_lib
{

    class NotchFilter
    {

    public:
        NotchFilter(const double &sample_rate, const double &frequency_in, const double &bandwidth_in);

        double iterate(const double &sample_in); // Changed to const reference
        std::pair<std::vector<double>, std::vector<double>> getInternalFilterCoeffs() const;

    private:
        std::unique_ptr<laser_uav_lib::IIRFilter> filter;
        rclcpp::Logger logger_; // Added logger
    };

} // namespace laser_uav_lib

#endif