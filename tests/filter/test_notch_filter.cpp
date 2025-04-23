#include <gtest/gtest.h>
#include <laser_uav_lib/filter/notch_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <tuple>
#include <iostream>

using namespace laser_uav_lib;

class NotchFilterTest : public ::testing::Test
{
protected:
    rclcpp::Logger logger = rclcpp::get_logger("NotchFilterTest");

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

    std::complex<double> calculate_frequency_response(double omega, double omega0, double r)
    {
        std::complex<double> num_z_minus_1 = std::polar(1.0, -omega);
        std::complex<double> num_z_minus_2 = std::polar(1.0, -2.0 * omega);
        std::complex<double> den_z_minus_1 = std::polar(1.0, -omega);
        std::complex<double> den_z_minus_2 = std::polar(1.0, -2.0 * omega);
        std::complex<double> e_jw0 = std::polar(1.0, omega0);
        std::complex<double> e_minus_jw0 = std::polar(1.0, -omega0);

        std::complex<double> numerator = (1.0 - 2.0 * std::cos(omega0) * num_z_minus_1 + num_z_minus_2);
        std::complex<double> denominator = (1.0 - 2.0 * r * std::cos(omega0) * den_z_minus_1 + r * r * den_z_minus_2);

        return numerator / denominator;
    }

    std::pair<std::vector<double>, std::vector<double>> get_filter_coeffs(const NotchFilter &filter)
    {
        return filter.getInternalFilterCoeffs();
    }
};

TEST_F(NotchFilterTest, Initialization)
{
    double sample_rate = 1000.0;
    double frequency = 60.0;
    double bandwidth = 5.0;

    NotchFilter notch_filter(sample_rate, frequency, bandwidth);
    auto [a_coeffs, b_coeffs] = get_filter_coeffs(notch_filter);

    std::cout << "[Initialization Test] Coefficients a: ";
    for (double val : a_coeffs)
        std::cout << val << " ";
    std::cout << std::endl;
    std::cout << "[Initialization Test] Coefficients b: ";
    for (double val : b_coeffs)
        std::cout << val << " ";
    std::cout << std::endl;

    // Adicione aqui asserções mais robustas baseadas na sua lógica de inicialização
    ASSERT_EQ(b_coeffs.size(), 3); // Ajuste conforme a ordem esperada
    ASSERT_EQ(a_coeffs.size(), 3); // Ajuste conforme a ordem esperada
}

TEST_F(NotchFilterTest, AttenuationAtNotchFrequency)
{
    double sample_rate = 1000.0;
    double frequency = 60.0;
    double bandwidth = 10.0;
    NotchFilter notch_filter(sample_rate, frequency, bandwidth);

    double amplitude = 1.0;
    double time = 0.0;
    double dt = 1.0 / sample_rate;
    int num_samples = 2000;

    for (int i = 0; i < num_samples; ++i)
    {
        double input_signal = amplitude * std::sin(2.0 * M_PI * frequency * time);
        double output_signal = notch_filter.iterate(input_signal);

        if (i > 500)
        {
            // std::cout << "[Attenuation Test] Amostra " << i << ", Output: " << output_signal << std::endl;
            ASSERT_LE(std::abs(output_signal), 0.1 * amplitude);
        }
        time += dt;
    }
}

TEST_F(NotchFilterTest, PassThroughAwayFromNotchFrequency)
{
    double sample_rate = 1000.0;
    double frequency_notch = 60.0;
    double bandwidth = 10.0;
    NotchFilter notch_filter(sample_rate, frequency_notch, bandwidth);

    auto calculate_magnitude_response = [&](double freq_test)
    {
        auto [a_coeffs, b_coeffs] = get_filter_coeffs(notch_filter);
        std::complex<double> numerator(0.0, 0.0);
        std::complex<double> denominator(0.0, 0.0);
        std::complex<double> jw = std::complex<double>(0.0, 2.0 * M_PI * freq_test / sample_rate);

        for (size_t i = 0; i < b_coeffs.size(); ++i)
        {
            numerator += b_coeffs[i] * std::pow(std::exp(-jw), static_cast<double>(i));
        }
        for (size_t i = 0; i < a_coeffs.size(); ++i)
        {
            denominator += a_coeffs[i] * std::pow(std::exp(-jw), static_cast<double>(i));
        }
        return std::abs(numerator / denominator);
    };

    double tolerance_magnitude = 0.1;

    auto check_pass_through = [&](double freq_test)
    {
        double magnitude_response = calculate_magnitude_response(freq_test);
        std::cout << "[PassThrough Test] Frequência de teste: " << freq_test << " Hz, Magnitude esperada: " << magnitude_response << std::endl;
        ASSERT_NEAR(magnitude_response, 1.0, tolerance_magnitude);

        double amplitude = 1.0;
        double time = 0.0;
        double dt = 1.0 / sample_rate;
        int num_samples = 3000;
        for (int i = 0; i < num_samples; ++i)
        {
            double input_signal = amplitude * std::sin(2.0 * M_PI * freq_test * time);
            double output_signal = notch_filter.iterate(input_signal);
            if (i > 2500)
            {
                // std::cout << "[PassThrough Test] Frequência de teste: " << freq_test << " Hz, Amostra " << i << ", Output: " << output_signal << ", Magnitude saída: " << std::abs(output_signal) << std::endl;
                ASSERT_NEAR(std::abs(output_signal), amplitude * magnitude_response, tolerance_magnitude * amplitude);
            }
            time += dt;
        }
    };

    check_pass_through(20.0);
    check_pass_through(150.0);
}

TEST_F(NotchFilterTest, ResponseToMixedFrequencies)
{
    double sample_rate = 1000.0;
    double frequency_notch = 60.0;
    double bandwidth = 10.0;
    NotchFilter notch_filter(sample_rate, frequency_notch, bandwidth);
    auto [a_coeffs, b_coeffs] = get_filter_coeffs(notch_filter);
    std::cout << "[Mixed Frequencies Test] Coeffs a: ";
    for (double v : a_coeffs)
        std::cout << v << " ";
    std::cout << std::endl;
    std::cout << "[Mixed Frequencies Test] Coeffs b: ";
    for (double v : b_coeffs)
        std::cout << v << " ";
    std::cout << std::endl;

    double amplitude_notch = 0.5;
    double amplitude_other = 1.0;
    double freq_other = 150.0;
    double time = 0.0;
    double dt = 1.0 / sample_rate;
    int num_samples = 5000;

    std::vector<double> output_buffer;

    for (int i = 0; i < num_samples; ++i)
    {
        double input_signal = amplitude_other * std::sin(2.0 * M_PI * freq_other * time) +
                              amplitude_notch * std::sin(2.0 * M_PI * frequency_notch * time);
        output_buffer.push_back(notch_filter.iterate(input_signal));
        // if (i > 4500) std::cout << "[Mixed Frequencies Test] Amostra " << i << ", Output: " << output_buffer.back() << std::endl;
        time += dt;
    }

    int start_index = 3000;
    double sum_at_notch = 0.0;
    double sum_at_other = 0.0;
    int num_analysis_samples = 1000;

    for (int i = start_index; i < start_index + num_analysis_samples; ++i)
    {
        sum_at_notch += std::abs(output_buffer[i] * std::sin(2.0 * M_PI * frequency_notch * (start_index + i) * dt));
        sum_at_other += std::abs(output_buffer[i] * std::sin(2.0 * M_PI * freq_other * (start_index + i) * dt));
    }

    std::cout << "[Mixed Frequencies Test] Sum at notch: " << sum_at_notch / num_analysis_samples << ", Expected max: " << 0.2 * amplitude_notch << std::endl;
    std::cout << "[Mixed Frequencies Test] Sum at other: " << sum_at_other / num_analysis_samples << ", Expected near: " << amplitude_other / 2.0 << std::endl;

    ASSERT_LE(sum_at_notch / num_analysis_samples, 0.2 * amplitude_notch);
    ASSERT_NEAR(sum_at_other / num_analysis_samples, amplitude_other / 2.0, 0.2);
}

TEST_F(NotchFilterTest, DifferentBandwidths)
{
    double sample_rate = 1000.0;
    double frequency_notch = 60.0;

    NotchFilter narrow_filter(sample_rate, frequency_notch, 2.0);
    NotchFilter wide_filter(sample_rate, frequency_notch, 20.0);

    auto calculate_magnitude_response = [&](NotchFilter &filter, double freq_test)
    {
        auto [a_coeffs, b_coeffs] = get_filter_coeffs(filter);
        std::complex<double> numerator(0.0, 0.0);
        std::complex<double> denominator(0.0, 0.0);
        std::complex<double> jw = std::complex<double>(0.0, 2.0 * M_PI * freq_test / sample_rate);

        for (size_t i = 0; i < b_coeffs.size(); ++i)
        {
            numerator += b_coeffs[i] * std::pow(std::exp(-jw), static_cast<double>(i));
        }
        for (size_t i = 0; i < a_coeffs.size(); ++i)
        {
            denominator += a_coeffs[i] * std::pow(std::exp(-jw), static_cast<double>(i));
        }
        return std::abs(numerator / denominator);
    };

    double test_frequency = 55.0;
    double mag_narrow = calculate_magnitude_response(narrow_filter, test_frequency);
    double mag_wide = calculate_magnitude_response(wide_filter, test_frequency);

    std::cout << "[Different Bandwidths Test] Narrow BW Magnitude at " << test_frequency << " Hz: " << mag_narrow << std::endl;
    std::cout << "[Different Bandwidths Test] Wide BW Magnitude at " << test_frequency << " Hz: " << mag_wide << std::endl;

    // Um filtro notch mais estreito deve ter uma atenuação maior perto da frequência de notch
    ASSERT_LE(mag_narrow, mag_wide);

    double amplitude = 1.0;
    double time = 0.0;
    double dt = 1.0 / sample_rate;
    int num_samples = 2000;

    double output_narrow_last = 0.0;
    double output_wide_last = 0.0;

    for (int i = 0; i < num_samples; ++i)
    {
        double input_signal = amplitude * std::sin(2.0 * M_PI * test_frequency * time);
        output_narrow_last = narrow_filter.iterate(input_signal);
        output_wide_last = wide_filter.iterate(input_signal);
        time += dt;
    }

    std::cout << "[Different Bandwidths Test] Last output (narrow): " << output_narrow_last << std::endl;
    std::cout << "[Different Bandwidths Test] Last output (wide): " << output_wide_last << std::endl;

    // Comparar as amplitudes das saídas (após a estabilização)
    ASSERT_LE(std::abs(output_narrow_last), std::abs(output_wide_last) + 0.1);
}