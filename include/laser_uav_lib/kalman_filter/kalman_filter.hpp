#ifndef LASER_UAV_LIB_KALMAN_FILTER_HPP
#define LASER_UAV_LIB_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <memory>
#include <Eigen/Dense>

namespace laser_uav_lib
{
    /**
     * @brief Interface (classe base abstrata) para filtros de Kalman.
     * @tparam States O número de estados no vetor de estado 'x'.
     * @tparam Inputs O número de entradas no vetor de controle 'u'.
     * @tparam Measurements O número de medições no vetor 'z'.
     */
    template <int States, int Inputs, int Measurements>
    class KalmanFilter
    {
    public:
        virtual ~KalmanFilter() = default;

        /**
         * @brief Executa o passo de predição do filtro.
         *
         * @param u O vetor de entrada de controle.
         * @param dt O intervalo de tempo (delta t).
         */
        virtual void predict(const Eigen::Matrix<double, Inputs, 1> &u, double dt) = 0;

        /**
         * @brief Executa o passo de correção (atualização) do filtro.
         *
         * @param z O vetor de medição.
         */
        virtual void correct(const Eigen::Matrix<double, Measurements, 1> &z) = 0;

        /**
         * @brief Retorna o estado estimado atual.
         *
         * @return const Eigen::Matrix<double, States, 1>& O vetor de estado 'x'.
         */
        virtual const Eigen::Matrix<double, States, 1> &get_state() const = 0;

        /**
         * @brief Retorna a matriz de covariância da incerteza atual.
         *
         * @return const Eigen::Matrix<double, States, States>& A matriz de covariância 'P'.
         */
        virtual const Eigen::Matrix<double, States, States> &get_covariance() const = 0;
    };

} // namespace laser_uav_lib
#endif // LASER_UAV_LIB_KALMAN_FILTER_HPP