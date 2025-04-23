#include <gtest/gtest.h>
#include <laser_uav_lib/filter/imu_filter.hpp>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace laser_uav_lib
{

    class ImuFilterTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            if (!rclcpp::ok())
                rclcpp::init(0, nullptr);
        }

        void TearDown() override
        {
            if (!rclcpp::ok())
                rclcpp::shutdown();
        }

        ImuData generateConstantImuData(double acc_x = 0.0, double acc_y = 0.0, double acc_z = 0.0,
                                        double gyro_x = 0.0, double gyro_y = 0.0, double gyro_z = 0.0)
        {
            ImuData data;
            data.timestamp = 0.0;
            data.frame_id = "imu_frame";
            data.orientation = {0.0, 0.0, 0.0, 1.0};
            data.orientation_covariance.fill(0.0);
            data.angular_velocity_x = gyro_x;
            data.angular_velocity_y = gyro_y;
            data.angular_velocity_z = gyro_z;
            data.angular_velocity_covariance.fill(0.0);
            data.linear_acceleration_x = acc_x;
            data.linear_acceleration_y = acc_y;
            data.linear_acceleration_z = acc_z;
            data.linear_acceleration_covariance.fill(0.0);
            return data;
        }
    };

    // Teste para verificar se os filtros de aceleração estão desabilitados corretamente
    TEST_F(ImuFilterTest, AccelerometerFiltersDisabled)
    {
        ImuFilter filter(false, {}, {}, false, 0.0, Eigen::MatrixXd::Zero(1, 0), 0.0,
                         false, {}, {}, false, 0.0, Eigen::MatrixXd::Zero(1, 0), 0.0);
        ImuData input_data = generateConstantImuData(1.0, 2.0, 3.0);
        ImuData output_data = filter.filterAccelerometer(input_data);
        ASSERT_EQ(output_data.linear_acceleration_x, 1.0);
        ASSERT_EQ(output_data.linear_acceleration_y, 2.0);
        ASSERT_EQ(output_data.linear_acceleration_z, 3.0);
    }

    // Teste para verificar se os filtros de giroscópio estão desabilitados corretamente
    TEST_F(ImuFilterTest, GyroFiltersDisabled)
    {
        ImuFilter filter(false, {}, {}, false, 0.0, Eigen::MatrixXd::Zero(1, 0), 0.0,
                         false, {}, {}, false, 0.0, Eigen::MatrixXd::Zero(1, 0), 0.0);
        ImuData input_data = generateConstantImuData(0.0, 0.0, 0.0, 0.1, 0.2, 0.3);
        ImuData output_data = filter.filterGyro(input_data);
        ASSERT_EQ(output_data.angular_velocity_x, 0.1);
        ASSERT_EQ(output_data.angular_velocity_y, 0.2);
        ASSERT_EQ(output_data.angular_velocity_z, 0.3);
    }

    // Teste simples do filtro IIR para aceleração
    TEST_F(ImuFilterTest, SimpleAccelerometerIirFilter)
    {
        std::vector<double> a = {1.0};
        std::vector<double> b = {0.5, 0.5};
        ImuFilter filter(true, a, b, false, 0.0, Eigen::MatrixXd::Zero(1, 0), 0.0,
                         false, {}, {}, false, 0.0, Eigen::MatrixXd::Zero(1, 0), 0.0);
        ImuData input1 = generateConstantImuData(2.0, 2.0, 2.0);
        ImuData output1 = filter.filterAccelerometer(input1);
        ASSERT_NEAR(output1.linear_acceleration_x, 1.0, 1e-9);
        ASSERT_NEAR(output1.linear_acceleration_y, 1.0, 1e-9);
        ASSERT_NEAR(output1.linear_acceleration_z, 1.0, 1e-9);

        ImuData input2 = generateConstantImuData(4.0, 4.0, 4.0);
        ImuData output2 = filter.filterAccelerometer(input2);
        ASSERT_NEAR(output2.linear_acceleration_x, 3.0, 1e-9);
        ASSERT_NEAR(output2.linear_acceleration_y, 3.0, 1e-9);
        ASSERT_NEAR(output2.linear_acceleration_z, 3.0, 1e-9);
    }

    // Teste simples do filtro Notch para aceleração
    TEST_F(ImuFilterTest, SimpleAccelerometerNotchFilter)
    {
        double sampling_rate = 100.0; // Hz
        Eigen::MatrixXd frequencies(1, 1);
        frequencies(0, 0) = 20.0; // Hz
        double bandwidth = 2.0;   // Hz
        ImuFilter filter(false, {}, {}, true, sampling_rate, frequencies, bandwidth,
                         false, {}, {}, false, 0.0, Eigen::MatrixXd::Zero(1, 0), 0.0);
        ImuData input_data;
        input_data.linear_acceleration_x = sin(2.0 * M_PI * 20.0 * 0.1);
        ImuData output_data = filter.filterAccelerometer(input_data);
        ASSERT_LT(std::abs(output_data.linear_acceleration_x), std::abs(input_data.linear_acceleration_x));

        ImuData input_off_notch;
        input_off_notch.linear_acceleration_x = sin(2.0 * M_PI * 10.0 * 0.1);
        ImuData output_off_notch = filter.filterAccelerometer(input_off_notch);
        ASSERT_NEAR(std::abs(output_off_notch.linear_acceleration_x), std::abs(input_off_notch.linear_acceleration_x), 0.1);
    }

    // Teste com múltiplos filtros Notch para aceleração
    TEST_F(ImuFilterTest, MultipleAccelerometerNotchFilters)
    {
        double sampling_rate = 100.0; // Hz
        Eigen::MatrixXd frequencies(1, 2);
        frequencies(0, 0) = 20.0; // Hz
        frequencies(0, 1) = 40.0; // Hz
        double bandwidth = 2.0;   // Hz
        ImuFilter filter(false, {}, {}, true, sampling_rate, frequencies, bandwidth,
                         false, {}, {}, false, 0.0, Eigen::MatrixXd::Zero(1, 0), 0.0);
        ImuData input_data_20hz;
        input_data_20hz.linear_acceleration_x = sin(2.0 * M_PI * 20.0 * 0.1);
        ImuData output_data_20hz = filter.filterAccelerometer(input_data_20hz);
        ASSERT_LT(std::abs(output_data_20hz.linear_acceleration_x), std::abs(input_data_20hz.linear_acceleration_x));

        ImuData input_data_40hz;
        input_data_40hz.linear_acceleration_x = sin(2.0 * M_PI * 40.0 * 0.1);
        ImuData output_data_40hz = filter.filterAccelerometer(input_data_40hz);
        ASSERT_LT(std::abs(output_data_40hz.linear_acceleration_x), std::abs(input_data_40hz.linear_acceleration_x));
    }

    // Teste combinando IIR e Notch para aceleração
    TEST_F(ImuFilterTest, CombinedAccelerometerIirAndNotch)
    {
        std::vector<double> a = {1.0};
        std::vector<double> b = {0.5, 0.5};
        double sampling_rate = 100.0; // Hz
        Eigen::MatrixXd frequencies(1, 1);
        frequencies(0, 0) = 20.0; // Hz
        double bandwidth = 2.0;   // Hz
        ImuFilter filter(true, a, b, true, sampling_rate, frequencies, bandwidth,
                         false, {}, {}, false, 0.0, Eigen::MatrixXd::Zero(1, 0), 0.0);
        ImuData input_data;
        input_data.linear_acceleration_x = 2.0 + sin(2.0 * M_PI * 20.0 * 0.1);
        ImuData output_data = filter.filterAccelerometer(input_data);
        // A saída deve ser atenuada na frequência de notch e ter a média móvel aplicada
        ASSERT_LT(std::abs(output_data.linear_acceleration_x - 1.0), std::abs(input_data.linear_acceleration_x - 2.0));
    }

    // Adicione testes similares para o filtro IIR e Notch aplicados aos dados do giroscópio.

} // namespace laser_uav_lib