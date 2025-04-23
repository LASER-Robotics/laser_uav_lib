#include <laser_uav_lib/filter/irr_filter.hpp>

namespace laser_uav_lib
{

    // | ------------------------ IIRFilter ------------------------ |

    /* IIRFilter constructor //{ */

    IIRFilter::IIRFilter(const std::vector<double> &a_in, const std::vector<double> &b_in)
        : logger_(rclcpp::get_logger("IRRFilter"))
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
        RCLCPP_INFO(logger_, "filter order : %ld", order_);
        RCLCPP_INFO(logger_, "IIR filter initialized!");
    }

    //}

    /* IIRFilter constructor //{ */

    IIRFilter::IIRFilter() : logger_(rclcpp::get_logger("IRRFilter"))
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
        double output = 0.0;

        // Atualizar o buffer de entrada
        input_buffer_.insert(input_buffer_.begin(), input);
        if (input_buffer_.size() > order_ + 1)
        {
            input_buffer_.pop_back();
        }

        // Calcular a parte feedforward (entrada)
        for (size_t i = 0; i < b_.size(); ++i)
        {
            if (i < input_buffer_.size())
            {
                output += b_[i] * input_buffer_[i];
            }
        }

        // Calcular a parte feedback (saída)
        for (size_t i = 1; i < a_.size(); ++i)
        {
            if (i <= processed_output_buffer_.size())
            {
                output -= a_[i] * processed_output_buffer_[i - 1];
            }
        }

        // Atualizar o buffer de saída
        processed_output_buffer_.insert(processed_output_buffer_.begin(), output);
        if (processed_output_buffer_.size() > order_)
        {
            processed_output_buffer_.pop_back();
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

    std::vector<double> IIRFilter::getInputBuffer()
    {
        return input_buffer_;
    }

    std::vector<double> IIRFilter::getOutputBuffer()
    {
        return processed_output_buffer_;
    }

    //}

} // namespace mrs_lib