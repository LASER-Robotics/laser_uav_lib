#include <laser_uav_lib/filter/imu_filter.hpp>

#include <iostream> // Para logging (substitua por seu sistema de logging preferido)

namespace laser_uav_lib
{

    ImuFilter::ImuFilter(bool acc_iir_enabled, const std::vector<double> &acc_iir_a, const std::vector<double> &acc_iir_b,
                         bool acc_notch_enabled, double acc_notch_sampling_rate, const Eigen::MatrixXd &acc_notch_frequencies, double acc_notch_bandwidth,
                         bool gyro_iir_enabled, const std::vector<double> &gyro_iir_a, const std::vector<double> &gyro_iir_b,
                         bool gyro_notch_enabled, double gyro_notch_sampling_rate, const Eigen::MatrixXd &gyro_notch_frequencies, double gyro_notch_bandwidth)
        : acc_iir_filter_enabled_(acc_iir_enabled),
          acc_notch_filter_enabled_(acc_notch_enabled),
          acc_notch_filter_sampling_rate_(acc_notch_sampling_rate),
          gyro_iir_filter_enabled_(gyro_iir_enabled),
          gyro_notch_filter_enabled_(gyro_notch_enabled),
          gyro_notch_filter_sampling_rate_(gyro_notch_sampling_rate)
    {
        // Inicialização dos filtros IIR para aceleração
        if (acc_iir_filter_enabled_)
        {
            acc_iir_filter_.iir_filter_x = std::make_unique<laser_uav_lib::IIRFilter>(acc_iir_a, acc_iir_b);
            acc_iir_filter_.iir_filter_y = std::make_unique<laser_uav_lib::IIRFilter>(acc_iir_a, acc_iir_b);
            acc_iir_filter_.iir_filter_z = std::make_unique<laser_uav_lib::IIRFilter>(acc_iir_a, acc_iir_b);
        }

        // Inicialização dos filtros Notch para aceleração
        if (acc_notch_filter_enabled_)
        {
            for (int i = 0; i < acc_notch_frequencies.cols(); ++i)
            {
                std::shared_ptr<NotchFilterContainer> nfc = std::make_shared<NotchFilterContainer>();
                nfc->notch_filter_x = std::make_unique<laser_uav_lib::NotchFilter>(acc_notch_filter_sampling_rate_, acc_notch_frequencies(0, i), acc_notch_bandwidth);
                nfc->notch_filter_y = std::make_unique<laser_uav_lib::NotchFilter>(acc_notch_filter_sampling_rate_, acc_notch_frequencies(0, i), acc_notch_bandwidth);
                nfc->notch_filter_z = std::make_unique<laser_uav_lib::NotchFilter>(acc_notch_filter_sampling_rate_, acc_notch_frequencies(0, i), acc_notch_bandwidth);
                acc_notch_filter_vector_.push_back(nfc);
            }
        }

        // Inicialização dos filtros IIR para giroscópio
        if (gyro_iir_filter_enabled_)
        {
            gyro_iir_filter_.iir_filter_x = std::make_unique<laser_uav_lib::IIRFilter>(gyro_iir_a, gyro_iir_b);
            gyro_iir_filter_.iir_filter_y = std::make_unique<laser_uav_lib::IIRFilter>(gyro_iir_a, gyro_iir_b);
            gyro_iir_filter_.iir_filter_z = std::make_unique<laser_uav_lib::IIRFilter>(gyro_iir_a, gyro_iir_b);
        }

        // Inicialização dos filtros Notch para giroscópio
        if (gyro_notch_filter_enabled_)
        {
            for (int i = 0; i < gyro_notch_frequencies.cols(); ++i)
            {
                std::shared_ptr<NotchFilterContainer> nfc = std::make_shared<NotchFilterContainer>();
                nfc->notch_filter_x = std::make_unique<laser_uav_lib::NotchFilter>(gyro_notch_filter_sampling_rate_, gyro_notch_frequencies(0, i), gyro_notch_bandwidth);
                nfc->notch_filter_y = std::make_unique<laser_uav_lib::NotchFilter>(gyro_notch_filter_sampling_rate_, gyro_notch_frequencies(0, i), gyro_notch_bandwidth);
                nfc->notch_filter_z = std::make_unique<laser_uav_lib::NotchFilter>(gyro_notch_filter_sampling_rate_, gyro_notch_frequencies(0, i), gyro_notch_bandwidth);
                gyro_notch_filter_vector_.push_back(nfc);
            }
        }

        RCLCPP_INFO(logger_, "IMU filter initialized!");
    }

    ImuData ImuFilter::filter(const ImuData &imu_data)
    {
        ImuData filtered_data = imu_data;
        filtered_data = filterAccelerometer(filtered_data);
        filtered_data = filterGyro(filtered_data);
        return filtered_data;
    }

    ImuData ImuFilter::filterAccelerometer(const ImuData &imu_data)
    {
        ImuData filtered_data = imu_data;

        if (acc_notch_filter_enabled_)
        {
            for (const auto &notch_filter : acc_notch_filter_vector_)
            {
                filtered_data.linear_acceleration_x = notch_filter->notch_filter_x->iterate(filtered_data.linear_acceleration_x);
                filtered_data.linear_acceleration_y = notch_filter->notch_filter_y->iterate(filtered_data.linear_acceleration_y);
                filtered_data.linear_acceleration_z = notch_filter->notch_filter_z->iterate(filtered_data.linear_acceleration_z);
            }
        }

        if (acc_iir_filter_enabled_)
        {
            filtered_data.linear_acceleration_x = acc_iir_filter_.iir_filter_x->iterate(filtered_data.linear_acceleration_x);
            filtered_data.linear_acceleration_y = acc_iir_filter_.iir_filter_y->iterate(filtered_data.linear_acceleration_y);
            filtered_data.linear_acceleration_z = acc_iir_filter_.iir_filter_z->iterate(filtered_data.linear_acceleration_z);
        }

        return filtered_data;
    }

    ImuData ImuFilter::filterGyro(const ImuData &imu_data)
    {
        ImuData filtered_data = imu_data;

        if (gyro_notch_filter_enabled_)
        {
            for (const auto &notch_filter : gyro_notch_filter_vector_)
            {
                filtered_data.angular_velocity_x = notch_filter->notch_filter_x->iterate(filtered_data.angular_velocity_x);
                filtered_data.angular_velocity_y = notch_filter->notch_filter_y->iterate(filtered_data.angular_velocity_y);
                filtered_data.angular_velocity_z = notch_filter->notch_filter_z->iterate(filtered_data.angular_velocity_z);
            }
        }

        if (gyro_iir_filter_enabled_)
        {
            filtered_data.angular_velocity_x = gyro_iir_filter_.iir_filter_x->iterate(filtered_data.angular_velocity_x);
            filtered_data.angular_velocity_y = gyro_iir_filter_.iir_filter_y->iterate(filtered_data.angular_velocity_y);
            filtered_data.angular_velocity_z = gyro_iir_filter_.iir_filter_z->iterate(filtered_data.angular_velocity_z);
        }

        return filtered_data;
    }

} // namespace my_imu_filter