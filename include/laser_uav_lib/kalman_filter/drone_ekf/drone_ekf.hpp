#ifndef LASER_UAV_LIB_DRONE_EKF_HPP
#define LASER_UAV_LIB_DRONE_EKF_HPP

#include <Eigen/Dense>
#include <optional>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

// Inclui os tipos de mensagem ROS para as medições
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <laser_uav_lib/kalman_filter/ekf/ekf.hpp>
#include <laser_uav_lib/kalman_filter/kalman_filter.hpp> // Inclui a interface base
#include <laser_uav_lib/attitude_converter/attitude_converter.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <type_traits>
#include <sstream>

namespace laser_uav_lib
{
    /**
     * @brief Namespace que define os índices do vetor de estado para fácil acesso
     * e legibilidade do código.
     */
    namespace State
    {
        enum
        {
            PX = 0,  ///< Posição no eixo X do referencial inercial.
            PY = 1,  ///< Posição no eixo Y do referencial inercial.
            PZ = 2,  ///< Posição no eixo Z do referencial inercial.
            QW = 3,  ///< Componente W (real) do quaternião de orientação.
            QX = 4,  ///< Componente X (i) do quaternião de orientação.
            QY = 5,  ///< Componente Y (j) do quaternião de orientação.
            QZ = 6,  ///< Componente Z (k) do quaternião de orientação.
            VX = 7,  ///< Velocidade linear no eixo X do referencial do CORPO.
            VY = 8,  ///< Velocidade linear no eixo Y do referencial do CORPO.
            VZ = 9,  ///< Velocidade linear no eixo Z do referencial do CORPO.
            WX = 10, ///< Velocidade angular no eixo X do referencial do CORPO (Roll rate).
            WY = 11, ///< Velocidade angular no eixo Y do referencial do CORPO (Pitch rate).
            WZ = 12  ///< Velocidade angular no eixo Z do referencial do CORPO (Yaw rate).
        };
    }

    // --- Definição das Dimensões ---
    constexpr int STATES = 13;       ///< Número total de estados no vetor de estado.
    constexpr int INPUTS = 4;        ///< Número de entradas de controle (força de cada motor).
    constexpr int MEASUREMENTS = 13; ///< Número máximo de medições (usado na odometria).

    /**
     * @brief Estrutura para agrupar todas as medições disponíveis em um único passo de tempo.
     * O uso de std::optional permite que qualquer medição de sensor esteja ausente no passo de correção,
     * tornando o filtro flexível a falhas ou ausência de sensores.
     */
    struct MeasurementPackage
    {
        std::optional<nav_msgs::msg::Odometry> openvins;     ///< Medição de odometria (posição, orientação, velocidades).
        std::optional<nav_msgs::msg::Odometry> fast_lio;     ///< Medição de odometria (posição, orientação, velocidades).
        std::optional<nav_msgs::msg::Odometry> px4_odometry; ///< Medição de odometria (posição, orientação, velocidades).
        std::optional<sensor_msgs::msg::Imu> imu;            ///< Medição de IMU (aceleração, velocidade angular).
        std::optional<geometry_msgs::msg::Point> gps;        ///< Medição de GPS (posição).
    };

    /**
     * @brief Estrutura para os ganhos de ruído do PROCESSO (modelo)
     */
    struct ProcessNoiseGains
    {
        double position = 0.01;
        double orientation = 0.01;
        double linear_velocity = 0.1;
        double angular_velocity = 0.1;
    };

    /**
     * @brief Estrutura para os ganhos de ruído da MEDIÇÃO (sensores)
     */
    struct MeasurementNoiseGains
    {
        ProcessNoiseGains px4_odometry;
        ProcessNoiseGains openvins;
        ProcessNoiseGains fast_lio;
        ProcessNoiseGains imu;
        ProcessNoiseGains gps;
    };

    /**
     * @brief Implementação do Filtro de Kalman Estendido (EKF) para um quadrotor.
     * * Esta classe herda da classe base EKF e a especializa para estimar o estado de
     * um drone (posição, orientação e velocidades) fundindo dados de múltiplos sensores.
     * Utiliza diferenciação automática para o cálculo das matrizes Jacobianas,
     * simplificando a implementação e reduzindo a chance de erros.
     */
    class DroneEKF : public EKF<STATES, INPUTS, MEASUREMENTS>
    {
    public:
        /**
         * @brief Construtor do DroneEKF.
         * * @param mass A massa do drone em kg.
         * @param arm_length O comprimento do braço do drone (distância do centro ao motor) em metros.
         * @param thrust_coeff O coeficiente de empuxo dos motores.
         * @param torque_coeff O coeficiente de torque dos motores.
         * @param inertia A matriz 3x3 de inércia do drone.
         * @param verbosity O nível de log para depuração ("SILENT", "INFO", "DEBUG").
         */
        DroneEKF(const double &mass, const double &arm_length, const double &thrust_coeff, const double &torque_coeff, const Eigen::Matrix3d &inertia, const std::string &verbosity = "INFO");

        ~DroneEKF() = default;

        /**
         * @brief Executa o passo de predição do EKF.
         * * Projeta o estado atual e a covariância para o próximo passo de tempo,
         * com base no modelo de movimento do drone e nas entradas de controle.
         * * @param u O vetor de controle (forças dos 4 motores).
         * @param dt O intervalo de tempo (delta t) desde a última predição, em segundos.
         */
        void predict(const Eigen::Matrix<double, INPUTS, 1> &u, double dt) override;

        /**
         * @brief Executa o passo de correção do EKF usando as medições disponíveis.
         * * Este método atua como um despachante, chamando os métodos de correção
         * específicos para cada sensor presente no `MeasurementPackage`.
         * * @param measurements Um pacote contendo os dados dos sensores disponíveis.
         */
        void correct(const MeasurementPackage &measurements);

        // A sobrecarga do método 'correct' da classe base é necessária, mesmo que não seja usada diretamente.
        void correct(const Eigen::Matrix<double, MEASUREMENTS, 1> &z) override
        {
            // Esta implementação pode ser deixada vazia ou lançar um erro,
            // já que a correção é feita pelo método que aceita MeasurementPackage.
        }

        /**
         * @brief Reseta o estado e a covariância do filtro para os valores iniciais.
         * Útil para reiniciar a estimativa após uma falha ou quando o drone está em um estado conhecido.
         */
        void reset();

        // --- Getters & Setters ---
        const Eigen::Matrix<double, STATES, 1> &get_state() const override { return x_; }
        const Eigen::Matrix<double, STATES, STATES> &get_covariance() const override { return P_; }

        /**
         * @brief Define a matriz de ruído do processo (Q).
         * @param Q A nova matriz de covariância do ruído do processo.
         */
        void set_process_noise(const Eigen::Matrix<double, STATES, STATES> &Q) { Q_ = Q; }

        /**
         * @brief Define o nível de verbosidade dos logs.
         * @param verbosity A string de verbosidade ("ALL", "DEBUG", "INFO", "WARNING", "ERROR", "SILENT").
         */
        void set_verbosity(const std::string &verbosity);

        /**
         * @brief Obtém o nível de verbosidade atual.
         * @return std::string O nível de verbosidade.
         */
        std::string get_verbosity() const;

        /**
         * @brief Define os ganhos de ruído do processo.
         * @param gains Os novos ganhos de ruído do processo.
         */
        void set_process_noise_gains(const ProcessNoiseGains &gains);

        /**
         * @brief Define os ganhos de ruído da medição.
         * @param gains Os novos ganhos de ruído da medição.
         */
        void set_measurement_noise_gains(const MeasurementNoiseGains &gains);

    private:
        // --- MÉTODOS DE CORREÇÃO INTERNOS ---

        /** @brief Corrige o estado usando uma medição de odometria. */
        void correct_odometry(const nav_msgs::msg::Odometry &odom);

        /** @brief Corrige o estado usando uma medição da IMU. */
        void correct_imu(const sensor_msgs::msg::Imu &imu);

        /** @brief Corrige o estado usando uma medição de GPS. */
        void correct_gps(const geometry_msgs::msg::Point &gps);

        // --- MODELO MATEMÁTICO E JACOBIANAS ---

        /**
         * @brief Modelo de transição de estado não-linear do drone.
         * * Implementa as equações diferenciais que descrevem a física do movimento do drone.
         * É um método template para poder ser usado com `autodiff::real` para diferenciação automática.
         * * @param x O vetor de estado atual.
         * @param u O vetor de controle aplicado.
         * @return Eigen::Matrix<T, STATES, 1> O vetor com as derivadas do estado (x_dot).
         */
        template <typename T>
        Eigen::Matrix<T, STATES, 1> state_transition_model(const Eigen::Matrix<T, STATES, 1> &x, const Eigen::Matrix<T, INPUTS, 1> &u) const;

        /**
         * @brief Calcula a matriz Jacobiana F (derivada do modelo de transição de estado em relação ao estado).
         * Utiliza diferenciação automática para evitar o cálculo manual.
         * * @param x O vetor de estado atual.
         * @param u O vetor de controle atual.
         */
        void calculate_jacobian_F(const Eigen::Matrix<double, STATES, 1> &x, const Eigen::Matrix<double, INPUTS, 1> &u);

        /**
         * @brief Funde múltiplas fontes de odometria usando a ponderação pela inversa da variância.
         * @param measurements O pacote de medições contendo as fontes de odometria.
         * @return Um std::optional contendo um par com o vetor de medição fundido (z) e
         * a sua matriz de covariância combinada (R). Retorna um optional vazio se nenhuma
         * fonte de odometria estiver disponível.
         */
        std::optional<std::pair<Eigen::Matrix<double, 13, 1>, Eigen::Matrix<double, 13, 13>>> fuse_odometry(const MeasurementPackage &measurements);

        /**
         * @brief Atualiza a matriz de covariância do ruído do processo (Q).
         */
        void update_Q_matrix();

        // --- MEMBROS DO FILTRO ---
        double dt_; ///< Armazena o último intervalo de tempo (dt) para uso na Jacobiana.

        // --- PARÂMETROS FÍSICOS ---
        double mass_;                              ///< Massa do drone [kg].
        double arm_length_;                        ///< Comprimento do braço do drone [m].
        double thrust_coefficient_;                ///< Coeficiente de empuxo.
        double torque_coefficient_;                ///< Coeficiente de torque.
        Eigen::Matrix3d inertia_tensor_;           ///< Tensor de inércia.
        Eigen::Matrix3d inertia_tensor_inv_;       ///< Inversa do tensor de inércia (pré-calculada).
        static constexpr double GRAVITY = 9.80665; ///< Aceleração da gravidade [m/s^2].

        // --- LOGS E DEPURAÇÃO ---
        rclcpp::Logger logger_; ///< Logger do ROS2 para mensagens.
        std::string verbosity_; ///< Nível de log atual como string.
        bool is_debug_;         ///< Flag booleana para otimizar verificações de log.

        ProcessNoiseGains q_gains_;     ///< Ganhos de ruído do processo.
        MeasurementNoiseGains r_gains_; ///< Ganhos de ruído da medição.
    };
} // namespace laser_uav_lib

#endif // LASER_UAV_LIB_DRONE_EKF_HPP