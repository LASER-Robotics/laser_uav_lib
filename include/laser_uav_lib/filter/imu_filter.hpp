#ifndef IMU_FILTER_HPP_
#define IMU_FILTER_HPP_

#include <memory>
#include <vector>
#include <Eigen/Dense>                           // Se você usa Eigen para alguma manipulação de dados aqui
#include <laser_uav_lib/filter/irr_filter.hpp>   // Assumindo que o header do IIRFilter está aqui
#include <laser_uav_lib/filter/notch_filter.hpp> // Assumindo que o header do NotchFilter está aqui
#include <rclcpp/rclcpp.hpp>                     // Changed from ros/ros.h

namespace laser_uav_lib
{

    struct ImuData
    {
        double timestamp;
        std::string frame_id;
        std::array<double, 4> orientation; // Quaternion (x, y, z, w)
        std::array<double, 9> orientation_covariance;
        double angular_velocity_x;
        double angular_velocity_y;
        double angular_velocity_z;
        std::array<double, 9> angular_velocity_covariance;
        double linear_acceleration_x;
        double linear_acceleration_y;
        double linear_acceleration_z;
        std::array<double, 9> linear_acceleration_covariance;
    };

    class ImuFilter
    {
    public:
        ImuFilter(bool acc_iir_enabled, const std::vector<double> &acc_iir_a, const std::vector<double> &acc_iir_b,
                  bool acc_notch_enabled, double acc_notch_sampling_rate, const Eigen::MatrixXd &acc_notch_frequencies, double acc_notch_bandwidth,
                  bool gyro_iir_enabled, const std::vector<double> &gyro_iir_a, const std::vector<double> &gyro_iir_b,
                  bool gyro_notch_enabled, double gyro_notch_sampling_rate, const Eigen::MatrixXd &gyro_notch_frequencies, double gyro_notch_bandwidth);

        ImuData filter(const ImuData &imu_data);
        ImuData filterAccelerometer(const ImuData &imu_data);
        ImuData filterGyro(const ImuData &imu_data);

    private:
        rclcpp::Logger logger_ = rclcpp::get_logger("imu_filter");

        struct NotchFilterContainer
        {
            std::unique_ptr<laser_uav_lib::NotchFilter> notch_filter_x;
            std::unique_ptr<laser_uav_lib::NotchFilter> notch_filter_y;
            std::unique_ptr<laser_uav_lib::NotchFilter> notch_filter_z;
        };

        struct IirFilterContainer
        {
            std::unique_ptr<laser_uav_lib::IIRFilter> iir_filter_x;
            std::unique_ptr<laser_uav_lib::IIRFilter> iir_filter_y;
            std::unique_ptr<laser_uav_lib::IIRFilter> iir_filter_z;
        };

        bool acc_iir_filter_enabled_;
        IirFilterContainer acc_iir_filter_;
        bool acc_notch_filter_enabled_;
        std::vector<std::shared_ptr<NotchFilterContainer>> acc_notch_filter_vector_;
        double acc_notch_filter_sampling_rate_;

        bool gyro_iir_filter_enabled_;
        IirFilterContainer gyro_iir_filter_;
        bool gyro_notch_filter_enabled_;
        std::vector<std::shared_ptr<NotchFilterContainer>> gyro_notch_filter_vector_;
        double gyro_notch_filter_sampling_rate_;
    };

} // namespace my_imu_filter

#endif // IMU_FILTER_HPP_