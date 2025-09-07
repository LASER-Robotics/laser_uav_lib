#ifndef LASER_UAV_LIB_KALMAN_FILTER_EKF_HPP
#define LASER_UAV_LIB_KALMAN_FILTER_EKF_HPP

#include <Eigen/Dense>
#include <memory>
#include <optional>

#include <laser_uav_lib/kalman_filter/kalman_filter.hpp> // Inclui a interface base

namespace laser_uav_lib
{
    template <int States, int Inputs, int Measurements>
    class EKF : public KalmanFilter<States, Inputs, Measurements>
    {
    public:
        EKF();
        virtual ~EKF() = default;

        void predict(const Eigen::Matrix<double, Inputs, 1> &u, double dt) override;
        void correct(const Eigen::Matrix<double, Measurements, 1> &z) override;

        const Eigen::Matrix<double, States, 1> &get_state() const override { return x_; }
        const Eigen::Matrix<double, States, States> &get_covariance() const override { return P_; }

    protected:
        // Variáveis de estado e matrizes
        Eigen::Matrix<double, States, 1> x_;
        Eigen::Matrix<double, States, States> P_;
        Eigen::Matrix<double, States, States> F_;
        Eigen::Matrix<double, States, States> Q_;
        Eigen::Matrix<double, Measurements, States> H_;
        Eigen::Matrix<double, Measurements, Measurements> R_;
        Eigen::Matrix<double, Measurements, 1> z_;

        // Funções virtuais com corpo vazio. As classes filhas PODEM, mas não PRECISAM, sobrescrevê-las.
        virtual void state_transition_model(const Eigen::Matrix<double, States, 1> &x_in, const Eigen::Matrix<double, Inputs, 1> &u, double dt, Eigen::Matrix<double, States, 1> &x_out) const {}
        virtual void calculate_jacobian_F(const Eigen::Matrix<double, States, 1> &x, const Eigen::Matrix<double, Inputs, 1> &u, double dt, Eigen::Matrix<double, States, States> &F_out) const {}
        virtual void measurement_model(const Eigen::Matrix<double, States, 1> &x, Eigen::Matrix<double, Measurements, 1> &z_out) const {}
        virtual void calculate_jacobian_H(const Eigen::Matrix<double, States, 1> &x, Eigen::Matrix<double, Measurements, States> &H_out) const {}
    };
}

#endif // LASER_UAV_LIB_KALMAN_FILTER_EKF_HPP