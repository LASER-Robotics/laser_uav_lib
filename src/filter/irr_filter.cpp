#include <laser_uav_lib/filter/irr_filter.hpp>

namespace laser_uav_lib
{

    // | ------------------------ IIRFilter ------------------------ |

    /* IIRFilter constructor //{ */

    IIRFilter::IIRFilter(const std::vector<double> &a_in, const std::vector<double> &b_in,
                         const rclcpp::Logger &node_logger)
        : logger_(node_logger)
    {

        a_ = a_in;
        b_ = b_in;

        // get the order of the filter based on the more prominent part of the coefficients
        order_ = std::max(a_.size(), b_.size()) - 1;

        a_.resize(order_ + 1, 0.0);
        b_.resize(order_ + 1, 0.0);

        buffer_.resize(order_ + 1, 0.0);
        for (size_t i = 0; i < order_ + 1; i++)
        {
            RCLCPP_INFO_STREAM(logger_, "a: " << a_[i] << " b: " << b_[i]);
        }
        RCLCPP_INFO(logger_, "filter order : %d", order_);
        RCLCPP_INFO(logger_, "IIR filter initialized!");
    }

    //}

    /* IIRFilter constructor //{ */

    IIRFilter::IIRFilter(const rclcpp::Logger &node_logger)
        : logger_(node_logger)
    {
        a_.resize(1, 0.0);
        b_.resize(1, 1.0);
        order_ = 0;
        buffer_.resize(order_ + 1, 0.0);
    }

    //}

    /**
     * @brief Iterates the filter with the given input value.
     *
     * @param input The input value to be filtered.
     * @return The output value after filtering.
     */
    /* iterate() //{ */

    double IIRFilter::iterate(const double input)
    {
        double output = 0;

        buffer_[0] = input;

        for (size_t i = 1; i < order_ + 1; i++)
        {
            buffer_[0] += (-a_[i]) * buffer_[i];
            output += (b_[i]) * buffer_[i];
        }

        output += buffer_[0] * b_[0];

        for (size_t i = order_; i > 0; i--)
        {
            buffer_[i] = buffer_[i - 1];
        }

        return output;
    }

    //}

    /* getCoeffs //{ */

    std::tuple<std::vector<double>, std::vector<double>> IIRFilter::getCoeffs()
    {
        return std::make_tuple(a_, b_);
    }

    //}

    /* getBuffer //{ */

    std::vector<double> IIRFilter::getBuffer()
    {
        return buffer_;
    }

    //}

} // namespace mrs_lib