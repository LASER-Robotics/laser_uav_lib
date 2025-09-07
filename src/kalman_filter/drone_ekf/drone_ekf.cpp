#include <laser_uav_lib/kalman_filter/drone_ekf/drone_ekf.hpp>
#include <rclcpp/logging.hpp>
#include <sstream> // Necessário para std::stringstream

namespace laser_uav_lib
{
    /**
     * @brief Construtor do DroneEKF.
     * @param mass A massa do drone em kg.
     * @param arm_length O comprimento do braço do drone (distância do centro ao motor) em metros.
     * @param thrust_coeff O coeficiente de empuxo dos motores.
     * @param torque_coeff O coeficiente de torque dos motores.
     * @param inertia A matriz 3x3 de inércia do drone.
     * @param verbosity O nível de log para depuração ("SILENT", "INFO", "DEBUG").
     */
    DroneEKF::DroneEKF(const double &mass, const double &arm_length, const double &thrust_coeff, const double &torque_coeff, const Eigen::Matrix3d &inertia, const std::string &verbosity)
        : mass_(mass),
          arm_length_(arm_length),
          thrust_coefficient_(thrust_coeff),
          torque_coefficient_(torque_coeff),
          inertia_tensor_(inertia),
          logger_(rclcpp::get_logger("DroneEKF")),
          q_gains_(),
          r_gains_()
    {
        // Define o nível de verbosidade e a flag de depuração.
        set_verbosity(verbosity);
        RCLCPP_INFO(logger_, "--- CONSTRUTOR DRONE EKF ---");

        if (is_debug_)
        {
            // Mudança para o formato de stream, igual ao código antigo.
            RCLCPP_DEBUG_STREAM(logger_, "Entradas: mass=" << mass << ", arm_length=" << arm_length);
        }

        // Pré-calcula a inversa da matriz de inércia para otimização em tempo de execução.
        inertia_tensor_inv_ = inertia.inverse();

        // --- Inicialização do Estado (x_) ---
        // Zera todo o vetor de estado.
        x_.setZero();
        // Define a orientação inicial como um quaternião identidade (sem rotação).
        x_(State::QW) = 1.0;

        // --- Inicialização da Covariância (P_) ---
        // Inicia com uma matriz identidade, representando incerteza igual em todos os estados.
        P_.setIdentity();
        P_ *= 0.1; // Define uma incerteza inicial moderada.

        // --- Inicialização das Matrizes de Ruído e Jacobiana ---
        F_.setIdentity();  // A Jacobiana F é inicializada como identidade.
        Q_.setIdentity();  // Covariância do ruído do processo.
        update_Q_matrix(); // Assume um ruído de processo baixo inicialmente.

        if (is_debug_)
        {
            RCLCPP_DEBUG_STREAM(logger_, "Saída: Estado inicial x_ = ");
            RCLCPP_DEBUG_STREAM(logger_, "     ├ dPos/dt:      " << x_.template segment<3>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "     ├ dQuat/dt:     " << x_.template segment<4>(State::QW).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "     ├ dVel.Lin/dt:  " << x_.template segment<3>(State::VX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "     └ dVel.Ang/dt:  " << x_.template segment<3>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "Saída: Incerteza inicial (trace P) = " << P_.trace());
        }
    }

    /**
     * @brief Define os ganhos de ruído do processo (modelo).
     * @param gains A estrutura com os novos ganhos.
     */
    void DroneEKF::set_process_noise_gains(const ProcessNoiseGains &gains)
    {
        RCLCPP_INFO(logger_, "Atualizando ganhos de ruído do processo (Q).");
        q_gains_ = gains;
        update_Q_matrix(); // Reconstrói a matriz Q com os novos ganhos
    }

    /**
     * @brief Define os ganhos de ruído da medição (sensores).
     * @param gains A estrutura com os novos ganhos.
     */
    void DroneEKF::set_measurement_noise_gains(const MeasurementNoiseGains &gains)
    {
        RCLCPP_INFO(logger_, "Atualizando ganhos de ruído da medição (R).");
        r_gains_ = gains;
    }

    /**
     * @brief Atualiza a matriz de covariância de ruído do processo (Q)
     * com base nos valores atuais de q_gains_.
     */
    void DroneEKF::update_Q_matrix()
    {
        Q_.setZero();
        Q_.block<3, 3>(State::PX, State::PX) = Eigen::Matrix3d::Identity() * q_gains_.position;
        Q_.block<4, 4>(State::QW, State::QW) = Eigen::Matrix4d::Identity() * q_gains_.orientation;
        Q_.block<3, 3>(State::VX, State::VX) = Eigen::Matrix3d::Identity() * q_gains_.linear_velocity;
        Q_.block<3, 3>(State::WX, State::WX) = Eigen::Matrix3d::Identity() * q_gains_.angular_velocity;

        if (is_debug_)
        {
            RCLCPP_DEBUG_STREAM(logger_, "Matriz de ruído do processo Q atualizada. Trace: " << Q_.trace());
        }
    }

    /**
     * @brief Reseta o estado e a covariância do filtro para os valores iniciais.
     * Útil para reiniciar a estimativa após uma falha ou quando o drone está em um estado conhecido.
     */
    void DroneEKF::reset()
    {
        RCLCPP_INFO(logger_, "--- RESET DO FILTRO EKF ---");

        // Reseta o estado para a posição de origem, sem rotação e sem velocidades.
        x_.setZero();
        x_(State::QW) = 1.0;

        // Reseta a incerteza para o valor inicial.
        P_.setIdentity();
        P_ *= 0.1;

        // Reinicializa as matrizes de transição e ruído.
        F_.setIdentity();
        update_Q_matrix();

        RCLCPP_INFO(logger_, "Estado e incerteza resetados para os valores padrão.");
    }

    /**
     * @brief Define o nível de verbosidade dos logs.
     * @param verbosity A string de verbosidade ("ALL", "DEBUG", "INFO", "WARNING", "ERROR", "SILENT").
     */
    void DroneEKF::set_verbosity(const std::string &verbosity)
    {
        verbosity_ = verbosity;
        // Atualiza a flag booleana para otimizar as verificações de log em tempo real.
        is_debug_ = (verbosity_ == "DEBUG" || verbosity_ == "ALL");

        // Configura o nível do logger do ROS2 com base na string fornecida.
        if (verbosity_ == "SILENT")
        {
            logger_.set_level(rclcpp::Logger::Level::Fatal);
        }
        else if (verbosity_ == "ERROR")
        {
            logger_.set_level(rclcpp::Logger::Level::Error);
        }
        else if (verbosity_ == "WARNING")
        {
            logger_.set_level(rclcpp::Logger::Level::Warn);
        }
        else if (is_debug_)
        { // "ALL" ou "DEBUG"
            logger_.set_level(rclcpp::Logger::Level::Debug);
        }
        else
        { // O padrão é "INFO"
            logger_.set_level(rclcpp::Logger::Level::Info);
        }

        RCLCPP_INFO_STREAM(logger_, "Nível de verbosidade definido para: " << verbosity_);
    }

    /**
     * @brief Obtém o nível de verbosidade atual.
     * @return std::string O nível de verbosidade.
     */
    std::string DroneEKF::get_verbosity() const
    {
        return verbosity_;
    }

    /**
     * @brief Modelo de transição de estado não-linear do drone.
     * Implementa as equações diferenciais que descrevem a física do movimento do drone.
     * É um método template para poder ser usado com `autodiff::real` para diferenciação automática.
     * @param x O vetor de estado atual.
     * @param u O vetor de controle aplicado.
     * @return Eigen::Matrix<T, STATES, 1> O vetor com as derivadas do estado (x_dot).
     */
    template <typename T>
    Eigen::Matrix<T, STATES, 1> DroneEKF::state_transition_model(
        const Eigen::Matrix<T, STATES, 1> &x,
        const Eigen::Matrix<T, INPUTS, 1> &u) const
    {

        // if constexpr (std::is_same<T, double>::value)
        // {
        //     if (is_debug_)
        //     {
        //         // Logs de entrada detalhados e em várias linhas.
        //         RCLCPP_DEBUG_STREAM(logger_, "  -> state_transition_model");
        //         RCLCPP_DEBUG_STREAM(logger_, "     Entrada (x):");
        //         RCLCPP_DEBUG_STREAM(logger_, "     ├ dPos/dt:      " << x.template segment<3>(State::PX).transpose());
        //         RCLCPP_DEBUG_STREAM(logger_, "     ├ dQuat/dt:     " << x.template segment<4>(State::QW).transpose());
        //         RCLCPP_DEBUG_STREAM(logger_, "     ├ dVel.Lin/dt:  " << x.template segment<3>(State::VX).transpose());
        //         RCLCPP_DEBUG_STREAM(logger_, "     └ dVel.Ang/dt:  " << x.template segment<3>(State::WX).transpose());
        //         RCLCPP_DEBUG_STREAM(logger_, "     Entrada (u): " << u.transpose());
        //     }
        // }

        // --- 1. Extração de Variáveis do Vetor de Estado ---
        Eigen::Quaternion<T> q(x(State::QW), x(State::QX), x(State::QY), x(State::QZ));
        q.normalize(); // Normaliza para garantir que a rotação seja válida.
        Eigen::Matrix<T, 3, 1> v_body = x.template segment<3>(State::VX);
        Eigen::Matrix<T, 3, 1> w_body = x.template segment<3>(State::WX);
        Eigen::Matrix<T, STATES, 1> x_dot; // Vetor para armazenar as derivadas.

        // --- 2. Cinemática: Derivadas de Posição e Orientação ---
        // A derivada da posição é a velocidade linear rotacionada para o referencial inercial.
        x_dot.template segment<3>(State::PX) = q.toRotationMatrix() * v_body;

        // A derivada do quaternião é calculada com base na velocidade angular.
        Eigen::Quaternion<T> w_quat(T(0), w_body.x(), w_body.y(), w_body.z());
        Eigen::Quaternion<T> q_dot_quat = q * w_quat;
        x_dot.template segment<4>(State::QW) = T(0.5) * Eigen::Matrix<T, 4, 1>(
                                                            q_dot_quat.w(), q_dot_quat.x(), q_dot_quat.y(), q_dot_quat.z());

        // --- 3. Dinâmica: Derivadas das Velocidades ---
        // A força de empuxo total atua ao longo do eixo Z do corpo do drone.
        T total_thrust = u.sum();
        Eigen::Matrix<T, 3, 1> thrust_force_body(T(0), T(0), total_thrust);

        // A força da gravidade é definida no referencial inercial e rotacionada para o referencial do corpo.
        Eigen::Matrix<T, 3, 1> g_inertial(T(0), T(0), T(-GRAVITY));
        Eigen::Matrix<T, 3, 1> g_body = q.toRotationMatrix().transpose() * g_inertial;

        // Aceleração linear (2ª Lei de Newton): a = (F_thrust/m) + g - w x v
        x_dot.template segment<3>(State::VX) = (thrust_force_body / T(mass_)) + g_body - w_body.cross(v_body);

        // Os torques são gerados pela diferença de empuxo entre os motores.
        Eigen::Matrix<T, 3, 1> tau;
        tau << T(arm_length_) * (u(0) + u(3) - u(1) - u(2)),      // Torque de Roll (eixo X)
            T(arm_length_) * (u(2) + u(3) - u(0) - u(1)),         // Torque de Pitch (eixo Y)
            T(torque_coefficient_) * (u(1) + u(3) - u(0) - u(2)); // Torque de Yaw (eixo Z)

        // Aceleração angular (Equação de Euler): α = I⁻¹ * (τ - w x (I * w))
        x_dot.template segment<3>(State::WX) = inertia_tensor_inv_.template cast<T>() * (tau - w_body.cross(inertia_tensor_.template cast<T>() * w_body));

        // if constexpr (std::is_same<T, double>::value)
        // {
        //     if (is_debug_)
        //     {
        //         // Logs de saída detalhados e em várias linhas.
        //         RCLCPP_DEBUG_STREAM(logger_, "     Cálculo (Torques tau): " << tau.transpose());
        //         RCLCPP_DEBUG_STREAM(logger_, "     Saída (x_dot):");
        //         RCLCPP_DEBUG_STREAM(logger_, "     ├ dPos/dt:      " << x_dot.template segment<3>(State::PX).transpose());
        //         RCLCPP_DEBUG_STREAM(logger_, "     ├ dQuat/dt:     " << x_dot.template segment<4>(State::QW).transpose());
        //         RCLCPP_DEBUG_STREAM(logger_, "     ├ dVel.Lin/dt:  " << x_dot.template segment<3>(State::VX).transpose());
        //         RCLCPP_DEBUG_STREAM(logger_, "     └ dVel.Ang/dt:  " << x_dot.template segment<3>(State::WX).transpose());
        //     }
        // }

        return x_dot;
    }

    /**
     * @brief Calcula a matriz Jacobiana F (derivada do modelo de transição de estado em relação ao estado).
     * Utiliza diferenciação automática para evitar o cálculo manual.
     * @param x O vetor de estado atual.
     * @param u O vetor de controle atual.
     */
    void DroneEKF::calculate_jacobian_F(
        const Eigen::Matrix<double, STATES, 1> &x,
        const Eigen::Matrix<double, INPUTS, 1> &u)
    {

        // if (is_debug_)
        // {
        //     RCLCPP_DEBUG_STREAM(logger_, "Calculando Jacobiana F..."); //
        // }

        // Converte os vetores para o tipo `autodiff::real` para a diferenciação.
        autodiff::VectorXreal x_ad = x;
        autodiff::VectorXreal u_ad = u;

        // Cria uma função lambda que representa o modelo de estado discretizado (x_k+1 = x_k + f(x,u)*dt).
        auto model_for_autodiff = [&](const autodiff::VectorXreal &x_arg) -> autodiff::VectorXreal
        {
            return x_arg + this->state_transition_model<autodiff::real>(x_arg, u_ad.cast<autodiff::real>()) * this->dt_;
        };

        // A biblioteca `autodiff` calcula a Jacobiana da função lambda em relação ao estado `x_ad`.
        F_ = autodiff::jacobian(model_for_autodiff, wrt(x_ad), at(x_ad));

        // if (is_debug_)
        // {
        //     RCLCPP_DEBUG_STREAM(logger_, "     Saída (Matriz F trace): " << F_.trace()); //
        // }
    }

    /**
     * @brief Executa o passo de predição do EKF.
     * Projeta o estado atual e a covariância para o próximo passo de tempo.
     * @param u O vetor de controle (forças dos 4 motores).
     * @param dt O intervalo de tempo (delta t) desde a última predição, em segundos.
     */
    void DroneEKF::predict(const Eigen::Matrix<double, INPUTS, 1> &u, double dt)
    {
        RCLCPP_DEBUG_STREAM(logger_, "--- PREDICT ---");
        if (is_debug_)
        {
            // Logs de entrada e estado ANTES, em formato detalhado.
            RCLCPP_DEBUG_STREAM(logger_, "Entradas: u = " << u.transpose() << ", dt = " << dt);
            RCLCPP_DEBUG_STREAM(logger_, "Estado (x) ANTES:");
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Posição (p):    " << x_.segment<3>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Quatérnion (q): " << x_.segment<4>(State::QW).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Vel. Linear (v): " << x_.segment<3>(State::VX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  └ Vel. Angular (w):" << x_.segment<3>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "Incerteza (trace P) ANTES: " << P_.trace());
        }

        dt_ = dt; // Armazena dt para ser usado no cálculo da Jacobiana.

        // 1. Lineariza o modelo de movimento no ponto atual para obter F_.
        calculate_jacobian_F(x_, u);

        // 2. Propaga a incerteza (covariância) para o próximo passo: P = F * P * F^T + Q
        P_ = F_ * P_ * F_.transpose() + Q_;

        RCLCPP_DEBUG_STREAM(logger_, "Matriz Q (Ruído do Processo):\n"
                                         << Q_);

        // 3. Propaga o estado usando o modelo não-linear: x = x + x_dot * dt
        Eigen::Matrix<double, STATES, 1> x_dot = state_transition_model<double>(x_, u);
        x_ += x_dot * dt;

        // 4. Normaliza o quaternião para garantir que ele permaneça uma rotação válida.
        x_.segment<4>(State::QW).normalize();

        if (is_debug_)
        {
            // Logs do estado DEPOIS, em formato detalhado.
            RCLCPP_DEBUG_STREAM(logger_, "Estado (x) DEPOIS:");
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Posição (p):    " << x_.segment<3>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Quatérnion (q): " << x_.segment<4>(State::QW).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Vel. Linear (v): " << x_.segment<3>(State::VX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  └ Vel. Angular (w):" << x_.segment<3>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "Incerteza (trace P) DEPOIS: " << P_.trace());
        }
    }

    /**
     * @brief Executa o passo de correção do EKF usando as medições disponíveis.
     * Este método atua como um despachante, chamando os métodos de correção
     * específicos para cada sensor presente no `MeasurementPackage`.
     * @param measurements Um pacote contendo os dados dos sensores disponíveis.
     */

    void DroneEKF::correct(const MeasurementPackage &measurements)
    {
        RCLCPP_DEBUG_STREAM(logger_, "--- CORRECT ---");

        if (is_debug_)
        {
            RCLCPP_DEBUG_STREAM(logger_, "Estado (x) Antes:");
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Posição (p):    " << x_.segment<3>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Quatérnion (q): " << x_.segment<4>(State::QW).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Vel. Linear (v): " << x_.segment<3>(State::VX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  └ Vel. Angular (w):" << x_.segment<3>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "Incerteza (trace P) ANTES: " << P_.trace());
        }
        // --- ETAPA 1: Pré-calcular o tamanho total das medições ---
        // Isso é crucial para inicializar as matrizes com o tamanho correto e evitar erros.
        int total_measurements = 0;
        constexpr int ODOM_MEASUREMENTS = 13; // P, Q, V, W
        constexpr int IMU_MEASUREMENTS = 3;   // Apenas velocidade angular
        constexpr int GPS_MEASUREMENTS = 2;   // Posição XY

        if (measurements.px4_odometry)
            total_measurements += ODOM_MEASUREMENTS;
        if (measurements.openvins)
            total_measurements += ODOM_MEASUREMENTS;
        if (measurements.fast_lio)
            total_measurements += ODOM_MEASUREMENTS;
        if (measurements.imu)
            total_measurements += IMU_MEASUREMENTS;
        if (measurements.gps)
            total_measurements += GPS_MEASUREMENTS;

        // Se não houver medições, não há nada a fazer.
        if (total_measurements == 0)
        {
            RCLCPP_DEBUG_STREAM(logger_, "Nenhuma medição disponível. Pulando passo de correção.");
            return;
        }

        // --- ETAPA 2: Inicializar as matrizes combinadas com o tamanho final e ZERADAS ---
        // Isso garante que os blocos fora da diagonal em R sejam zero, o que é correto
        // para ruídos de sensores não correlacionados.
        Eigen::VectorXd z = Eigen::VectorXd::Zero(total_measurements);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(total_measurements, STATES);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(total_measurements, total_measurements);

        int current_row = 0;         // Ponteiro para a linha atual onde os dados serão inseridos
        bool odom_processed = false; // Flag para o tratamento especial do quaternião
        int odom_start_row = -1;     // Linha de início da primeira medição de odometria

        // --- ETAPA 3: Preencher as matrizes em blocos, de forma segura ---

        // Processa Odometria da PX4 (se disponível)
        if (measurements.px4_odometry)
        {
            const auto &odom = measurements.px4_odometry.value();
            H.block<ODOM_MEASUREMENTS, ODOM_MEASUREMENTS>(current_row, 0).setIdentity();

            z.segment<ODOM_MEASUREMENTS>(current_row) << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
                odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z,
                odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;

            Eigen::Map<const Eigen::Matrix<double, 6, 6>> pose_cov(odom.pose.covariance.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> twist_cov(odom.twist.covariance.data());
            R.block<3, 3>(current_row + State::PX, current_row + State::PX) = pose_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * r_gains_.px4_odometry.position;
            R.block<3, 3>(current_row + State::QX, current_row + State::QX) = pose_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * r_gains_.px4_odometry.orientation;
            R.block<3, 3>(current_row + State::VX, current_row + State::VX) = twist_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * r_gains_.px4_odometry.linear_velocity;
            R.block<3, 3>(current_row + State::WX, current_row + State::WX) = twist_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * r_gains_.px4_odometry.angular_velocity;
            R(current_row + State::QW, current_row + State::QW) = 0.1; // O erro na componente W não é usado/não tem significado

            if (!odom_processed)
            {
                odom_start_row = current_row;
                odom_processed = true;
            }
            current_row += ODOM_MEASUREMENTS;
            RCLCPP_DEBUG_STREAM(logger_, "Adicionando medição de Odometria da PX4.");
        }

        // Processa Odometria do OpenVINS (se disponível)
        if (measurements.openvins)
        {
            // Lógica idêntica à da PX4, apenas com a fonte de dados diferente
            const auto &odom = measurements.openvins.value();
            H.block<ODOM_MEASUREMENTS, ODOM_MEASUREMENTS>(current_row, 0).setIdentity();
            z.segment<ODOM_MEASUREMENTS>(current_row) << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z, odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> pose_cov(odom.pose.covariance.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> twist_cov(odom.twist.covariance.data());
            R.block<3, 3>(current_row + State::PX, current_row + State::PX) = pose_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * r_gains_.openvins.position;
            R.block<3, 3>(current_row + State::QX, current_row + State::QX) = pose_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * r_gains_.openvins.orientation;
            R.block<3, 3>(current_row + State::VX, current_row + State::VX) = twist_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * r_gains_.openvins.linear_velocity;
            R.block<3, 3>(current_row + State::WX, current_row + State::WX) = twist_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * r_gains_.openvins.angular_velocity;
            R(current_row + State::QW, current_row + State::QW) = 0.1; // O erro na componente W não é usado/não tem significado

            if (!odom_processed)
            {
                odom_start_row = current_row;
                odom_processed = true;
            }
            current_row += ODOM_MEASUREMENTS;
            RCLCPP_DEBUG_STREAM(logger_, "Adicionando medição de Odometria do OpenVINS.");
        }

        // Processa Odometria do FastLIO (se disponível) - Adapte conforme necessário
        if (measurements.fast_lio)
        {
            // Lógica idêntica
            const auto &odom = measurements.fast_lio.value();
            H.block<ODOM_MEASUREMENTS, ODOM_MEASUREMENTS>(current_row, 0).setIdentity();

            z.segment<ODOM_MEASUREMENTS>(current_row) << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
                odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z,
                odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;

            Eigen::Map<const Eigen::Matrix<double, 6, 6>> pose_cov(odom.pose.covariance.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> twist_cov(odom.twist.covariance.data());
            R.block<3, 3>(current_row + State::PX, current_row + State::PX) = pose_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * r_gains_.fast_lio.position;
            R.block<3, 3>(current_row + State::QX, current_row + State::QX) = pose_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * r_gains_.fast_lio.orientation;
            R.block<3, 3>(current_row + State::VX, current_row + State::VX) = twist_cov.block<3, 3>(0, 0) + Eigen::Matrix3d::Identity() * r_gains_.fast_lio.linear_velocity;
            R.block<3, 3>(current_row + State::WX, current_row + State::WX) = twist_cov.block<3, 3>(3, 3) + Eigen::Matrix3d::Identity() * r_gains_.fast_lio.angular_velocity;
            R(current_row + State::QW, current_row + State::QW) = 0.1; // O erro na componente W não é usado/não tem significado

            if (!odom_processed)
            {
                odom_start_row = current_row;
                odom_processed = true;
            }
            current_row += ODOM_MEASUREMENTS;
            RCLCPP_DEBUG_STREAM(logger_, "Adicionando medição de Odometria do FastLIO.");
        }

        // Processa IMU (se disponível)
        if (measurements.imu)
        {
            const auto &imu = measurements.imu.value();
            H.block<IMU_MEASUREMENTS, IMU_MEASUREMENTS>(current_row, State::WX).setIdentity();
            z.segment<IMU_MEASUREMENTS>(current_row) << imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z;
            R.block<IMU_MEASUREMENTS, IMU_MEASUREMENTS>(current_row, current_row) = Eigen::Map<const Eigen::Matrix<double, 3, 3>>(imu.angular_velocity_covariance.data()) + Eigen::Matrix3d::Identity() * r_gains_.imu.angular_velocity;

            current_row += IMU_MEASUREMENTS;
            RCLCPP_DEBUG_STREAM(logger_, "Adicionando medição de IMU (Vel. Angular).");
        }

        // Processa GPS (se disponível)
        if (measurements.gps)
        {
            const auto &gps = measurements.gps.value();
            H(current_row, State::PX) = 1.0;
            H(current_row + 1, State::PY) = 1.0;
            z.segment<GPS_MEASUREMENTS>(current_row) << gps.x, gps.y;
            R(current_row, current_row) = r_gains_.gps.position;
            R(current_row + 1, current_row + 1) = r_gains_.gps.position;

            current_row += GPS_MEASUREMENTS;
            RCLCPP_DEBUG_STREAM(logger_, "Adicionando medição de GPS (Pos. XY).");
        }

        // --- ETAPA 4: Verificações de segurança antes da atualização ---

        if (z.hasNaN())
        {
            RCLCPP_WARN_STREAM(logger_, "Medições combinadas contêm NaN! Correção ignorada.");
            return;
        }

        // Flooring da covariância: Garante que nenhuma variância na diagonal de R seja zero.
        // Esta é uma proteção crucial contra o colapso do filtro.
        double min_variance = 1e-6;
        for (int i = 0; i < total_measurements; ++i)
        {
            if (R(i, i) < min_variance)
            {
                R(i, i) = min_variance;
            }
        }

        if (is_debug_)
        {
            RCLCPP_DEBUG_STREAM(logger_, "Total de medições para correção: " << total_measurements);
            RCLCPP_DEBUG_STREAM(logger_, "Incerteza (trace P) ANTES: " << P_.trace());
        }

        // --- ETAPA 5: Execução da atualização do filtro ---

        // Modelo de medição h(x). Para os estados que medimos, é uma identidade.
        Eigen::VectorXd z_pred = H * x_;

        // Inovação y = z - h(x)
        Eigen::VectorXd y = z - z_pred;

        // Tratamento especial GENERALIZADO para o erro do quaternião.
        if (odom_processed)
        {
            Eigen::Quaterniond q_z(z(odom_start_row + State::QW), z(odom_start_row + State::QX), z(odom_start_row + State::QY), z(odom_start_row + State::QZ));
            q_z.normalize();
            Eigen::Quaterniond q_x(x_(State::QW), x_(State::QX), x_(State::QY), x_(State::QZ));
            Eigen::Quaterniond q_error_odom = q_z * q_x.inverse();

            // A inovação para a orientação é representada por um vetor de rotação de 3 dimensões
            y.segment<3>(odom_start_row + State::QX) = 2.0 * q_error_odom.vec();
            y(odom_start_row + State::QW) = 0; // O erro na componente W não é usado/não tem significado
        }

        // Covariância da Inovação S = H * P * H^T + R
        Eigen::MatrixXd S = H * P_ * H.transpose() + R;

        // Ganho de Kalman K = P * H^T * S^-1
        // A inversão de S é onde o crash acontecia. Agora S deve ser bem-comportada.
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        if (is_debug_)
        {
            RCLCPP_DEBUG_STREAM(logger_, "--- Confiança do Filtro (Norma do Ganho de Kalman K) ---");
            int print_row = 0;

            if (measurements.px4_odometry)
            {
                double norm = K.block(0, print_row, STATES, ODOM_MEASUREMENTS).norm();
                RCLCPP_DEBUG_STREAM(logger_, "  ├─ PX4 Odom: " << norm);
                print_row += ODOM_MEASUREMENTS;
            }
            if (measurements.openvins)
            {
                double norm = K.block(0, print_row, STATES, ODOM_MEASUREMENTS).norm();
                RCLCPP_DEBUG_STREAM(logger_, "  ├─ OpenVINS: " << norm);
                print_row += ODOM_MEASUREMENTS;
            }
            if (measurements.fast_lio)
            {
                double norm = K.block(0, print_row, STATES, ODOM_MEASUREMENTS).norm();
                RCLCPP_DEBUG_STREAM(logger_, "  ├─ FastLIO:  " << norm);
                print_row += ODOM_MEASUREMENTS;
            }
            if (measurements.imu)
            {
                double norm = K.block(0, print_row, STATES, IMU_MEASUREMENTS).norm();
                RCLCPP_DEBUG_STREAM(logger_, "  ├─ IMU (W):  " << norm);
                print_row += IMU_MEASUREMENTS;
            }
            if (measurements.gps)
            {
                double norm = K.block(0, print_row, STATES, GPS_MEASUREMENTS).norm();
                RCLCPP_DEBUG_STREAM(logger_, "  └─ GPS (XY): " << norm);
                print_row += GPS_MEASUREMENTS;
            }
        }

        // Atualização do estado: x = x + K * y
        x_ += K * y;

        // Atualização da covariância: P = (I - K * H) * P
        Eigen::Matrix<double, STATES, STATES> I;
        I.setIdentity();
        P_ = (I - K * H) * P_;

        // Normalização do quaternião para evitar desvios numéricos
        x_.segment<4>(State::QW).normalize();

        if (is_debug_)
        {
            RCLCPP_DEBUG_STREAM(logger_, "Estado (x) DEPOIS:");
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Posição (p):    " << x_.segment<3>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Quatérnion (q): " << x_.segment<4>(State::QW).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  ├ Vel. Linear (v): " << x_.segment<3>(State::VX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "  └ Vel. Angular (w):" << x_.segment<3>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "Inovação (norma y): " << y.norm());
            RCLCPP_DEBUG_STREAM(logger_, "Incerteza (trace P) DEPOIS: " << P_.trace());
        }
    }

    // void DroneEKF::correct(const MeasurementPackage &measurements)
    // {
    //     RCLCPP_DEBUG_STREAM(logger_, "--- CORRECT (Dispatcher) ---");

    //     // Verifica a existência e processa cada tipo de medição.
    //     if (measurements.px4_odometry)
    //     {
    //         correct_odometry(measurements.px4_odometry.value());
    //     }
    //     // if (measurements.fast_lio)
    //     // {
    //     //     correct_odometry(measurements.fast_lio.value());
    //     // }
    //     // if (measurements.openvins)
    //     // {
    //     //     correct_odometry(measurements.openvins.value());
    //     // }
    //     if (measurements.imu)
    //     {
    //         correct_imu(measurements.imu.value());
    //     }
    //     if (measurements.gps)
    //     {
    //         correct_gps(measurements.gps.value());
    //     }
    // }

    /**
     * @brief Corrige o estado usando uma medição de odometria.
     */
    void DroneEKF::correct_odometry(const nav_msgs::msg::Odometry &odom)
    {
        constexpr int ODOM_MEASUREMENTS = STATES;

        // 1. Constrói o vetor de medição (z) a partir da mensagem de odometria.
        Eigen::Matrix<double, ODOM_MEASUREMENTS, 1> z;
        z << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
            odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z,
            odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;

        if (is_debug_)
        {
            // Logs de entrada e estado ANTES, em formato detalhado.
            RCLCPP_DEBUG_STREAM(logger_, "--> correct_odometry");
            RCLCPP_DEBUG_STREAM(logger_, "    Entrada (z) completa:");
            RCLCPP_DEBUG_STREAM(logger_, "      ├ Posição (p):    " << z.segment<3>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "      ├ Quatérnion (q): " << z.segment<4>(State::QW).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "      ├ Vel. Linear (v): " << z.segment<3>(State::VX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "      └ Vel. Angular (w):" << z.segment<3>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "    Estado (x) ANTES: ");
            RCLCPP_DEBUG_STREAM(logger_, "      ├ Posição (p):    " << x_.segment<3>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "      ├ Quatérnion (q): " << x_.segment<4>(State::QW).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "      ├ Vel. Linear (v): " << x_.segment<3>(State::VX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "      └ Vel. Angular (w):" << x_.segment<3>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "    Incerteza (trace P) ANTES: " << P_.trace());
        }

        if (z.hasNaN())
        {
            RCLCPP_WARN_STREAM(logger_, "Medições de odometria contêm NaN! Correção ignorada.");
            return;
        }

        // 2. Constrói a matriz H. Como a odometria mede os estados diretamente, H é a identidade.
        Eigen::Matrix<double, ODOM_MEASUREMENTS, STATES> H;
        H.setIdentity();

        // 3. Constrói a matriz de ruído da medição (R) a partir das covariâncias da mensagem.
        Eigen::Matrix<double, ODOM_MEASUREMENTS, ODOM_MEASUREMENTS> R;
        R.setZero();

        Eigen::Map<const Eigen::Matrix<double, 6, 6>> pose_cov(odom.pose.covariance.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 6>> twist_cov(odom.twist.covariance.data());
        R.block<3, 3>(State::PX, State::PX) = pose_cov.block<3, 3>(0, 0);
        R.block<3, 3>(State::QX, State::QX) = pose_cov.block<3, 3>(3, 3);
        R.block<3, 3>(State::VX, State::VX) = twist_cov.block<3, 3>(0, 0);
        R.block<3, 3>(State::WX, State::WX) = twist_cov.block<3, 3>(3, 3);
        if (R(State::QW, State::QW) == 0)
            R(State::QW, State::QW) = 0.01;

        // 4. Calcula a inovação (y = z - h(x)).
        Eigen::Matrix<double, ODOM_MEASUREMENTS, 1> y = z - x_;

        // Tratamento especial para o erro do quaternião.
        Eigen::Quaterniond q_z(z(State::QW), z(State::QX), z(State::QY), z(State::QZ));
        q_z.normalize();
        Eigen::Quaterniond q_x(x_(State::QW), x_(State::QX), x_(State::QY), x_(State::QZ));
        Eigen::Quaterniond q_error_odom = q_z * q_x.inverse();
        y.segment<3>(State::QX) = 2.0 * q_error_odom.vec();
        y(State::QW) = 0;

        // 5. Calcula a covariância da inovação (S) e o Ganho de Kalman (K).
        Eigen::Matrix<double, ODOM_MEASUREMENTS, ODOM_MEASUREMENTS> S = H * P_ * H.transpose() + R;
        Eigen::Matrix<double, STATES, ODOM_MEASUREMENTS> K = P_ * H.transpose() * S.inverse();

        // 6. Atualiza o estado e a covariância.
        x_ += K * y;
        P_ = (Eigen::Matrix<double, STATES, STATES>::Identity() - K * H) * P_;
        x_.segment<4>(State::QW).normalize();

        if (is_debug_)
        {
            // Logs da inovação e estado DEPOIS, em formato detalhado.
            RCLCPP_DEBUG_STREAM(logger_, "    Inovação (y):");
            RCLCPP_DEBUG_STREAM(logger_, "      ├ Posição (p):    " << y.segment<3>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "      ├ Quatérnion (q): " << y.segment<4>(State::QW).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "      ├ Vel. Linear (v): " << y.segment<3>(State::VX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "      └ Vel. Angular (w):" << y.segment<3>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "    Saída: Estado (x) DEPOIS: ");
            RCLCPP_DEBUG_STREAM(logger_, "      ├ Posição (p):    " << x_.segment<3>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "      ├ Quatérnion (q): " << x_.segment<4>(State::QW).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "      ├ Vel. Linear (v): " << x_.segment<3>(State::VX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "      └ Vel. Angular (w):" << x_.segment<3>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "    Saída: Incerteza (trace P) DEPOIS: " << P_.trace());
        }
    }

    /**
     * @brief Corrige o estado usando uma medição da IMU.
     */
    void DroneEKF::correct_imu(const sensor_msgs::msg::Imu &imu)
    {
        constexpr int IMU_MEASUREMENTS = 3;

        // 1. Vetor de medição z (apenas velocidades angulares).
        Eigen::Matrix<double, IMU_MEASUREMENTS, 1> z;
        z << imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z;

        if (is_debug_)
        {
            // Logs em formato stream.
            RCLCPP_DEBUG_STREAM(logger_, "--> correct_imu");
            RCLCPP_DEBUG_STREAM(logger_, "    Entrada (z - vel angular): " << z.transpose());
            RCLCPP_DEBUG_STREAM(logger_, "    Estado (Wx,y,z) ANTES: " << x_.segment<IMU_MEASUREMENTS>(State::WX).transpose());
        }

        // 2. Matriz H que mapeia os estados de velocidade angular para a medição.
        Eigen::Matrix<double, IMU_MEASUREMENTS, STATES> H;
        H.setZero();
        H.block<3, 3>(0, State::WX) = Eigen::Matrix3d::Identity();

        // 3. Matriz R a partir da covariância da IMU.
        Eigen::Matrix<double, IMU_MEASUREMENTS, IMU_MEASUREMENTS> R = Eigen::Map<const Eigen::Matrix<double, 3, 3>>(imu.angular_velocity_covariance.data());

        // 4. Inovação (diferença entre medição e predição).
        Eigen::Matrix<double, IMU_MEASUREMENTS, 1> y = z - x_.segment<IMU_MEASUREMENTS>(State::WX);

        // 5. Cálculo de S e K.
        Eigen::Matrix<double, IMU_MEASUREMENTS, IMU_MEASUREMENTS> S = H * P_ * H.transpose() + R;
        Eigen::Matrix<double, STATES, IMU_MEASUREMENTS> K = P_ * H.transpose() * S.inverse();

        // 6. Atualização do estado e covariância.
        x_ += K * y;
        P_ = (Eigen::Matrix<double, STATES, STATES>::Identity() - K * H) * P_;
        x_.segment<4>(State::QW).normalize();

        if (is_debug_)
        {
            // Logs em formato stream.
            RCLCPP_DEBUG_STREAM(logger_, "    Inovação (y): " << y.transpose());
            RCLCPP_DEBUG_STREAM(logger_, "    Saída: Estado (Wx,y,z) DEPOIS: " << x_.segment<IMU_MEASUREMENTS>(State::WX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "    Saída: Incerteza (trace P) DEPOIS: " << P_.trace());
        }
    }

    /**
     * @brief Corrige o estado usando uma medição de GPS.
     */
    void DroneEKF::correct_gps(const geometry_msgs::msg::Point &gps)
    {
        constexpr int GPS_MEASUREMENTS = 2;

        // 1. Vetor de medição z (posição X, Y).
        Eigen::Matrix<double, GPS_MEASUREMENTS, 1> z;
        z << gps.x, gps.y;

        if (is_debug_)
        {
            // Logs em formato stream.
            RCLCPP_DEBUG_STREAM(logger_, "--> correct_gps");
            RCLCPP_DEBUG_STREAM(logger_, "    Entrada (z - pos XY): " << z.transpose());
            RCLCPP_DEBUG_STREAM(logger_, "    Estado (Px,y) ANTES: " << x_.segment<GPS_MEASUREMENTS>(State::PX).transpose());
        }

        // 2. Matriz H que mapeia os estados de posição para a medição.
        Eigen::Matrix<double, GPS_MEASUREMENTS, STATES> H;
        H.setZero();
        H(0, State::PX) = 1.0;
        H(1, State::PY) = 1.0;

        // 3. Matriz R fixa para o ruído do GPS.
        Eigen::Matrix<double, GPS_MEASUREMENTS, GPS_MEASUREMENTS> R;
        R << 0.5, 0,
            0, 0.5;

        // 4. Inovação.
        Eigen::Matrix<double, GPS_MEASUREMENTS, 1> y = z - x_.segment<GPS_MEASUREMENTS>(State::PX);

        // 5. Cálculo de S e K.
        Eigen::Matrix<double, GPS_MEASUREMENTS, GPS_MEASUREMENTS> S = H * P_ * H.transpose() + R;
        Eigen::Matrix<double, STATES, GPS_MEASUREMENTS> K = P_ * H.transpose() * S.inverse();

        // 6. Atualização do estado e covariância.
        x_ += K * y;
        P_ = (Eigen::Matrix<double, STATES, STATES>::Identity() - K * H) * P_;
        x_.segment<4>(State::QW).normalize();

        if (is_debug_)
        {
            // Logs em formato stream.
            RCLCPP_DEBUG_STREAM(logger_, "    Inovação (y): " << y.transpose());
            RCLCPP_DEBUG_STREAM(logger_, "    Saída: Estado (Px,y) DEPOIS: " << x_.segment<GPS_MEASUREMENTS>(State::PX).transpose());
            RCLCPP_DEBUG_STREAM(logger_, "    Saída: Incerteza (trace P) DEPOIS: " << P_.trace());
        }
    }

    /**
     * @brief Funde múltiplas fontes de odometria usando a ponderação pela inversa da variância.
     * @param measurements O pacote de medições contendo as fontes de odometria.
     * @return Um std::optional contendo um par com o vetor de medição fundido (z) e
     * a sua matriz de covariância combinada (R). Retorna um optional vazio se nenhuma
     * fonte de odometria estiver disponível.
     */
    std::optional<std::pair<Eigen::Matrix<double, 13, 1>, Eigen::Matrix<double, 13, 13>>>
    DroneEKF::fuse_odometry(const MeasurementPackage &measurements)
    {
        std::vector<nav_msgs::msg::Odometry> odom_sources;
        if (measurements.px4_odometry)
            odom_sources.push_back(measurements.px4_odometry.value());
        if (measurements.openvins)
            odom_sources.push_back(measurements.openvins.value());
        if (measurements.fast_lio)
            odom_sources.push_back(measurements.fast_lio.value());

        if (odom_sources.empty())
        {
            return std::nullopt; // Nenhuma fonte para fundir
        }

        if (is_debug_)
        {
            RCLCPP_DEBUG_STREAM(logger_, "Iniciando a fusão de " << odom_sources.size() << " fontes de odometria.");
        }

        // Se houver apenas uma fonte, retorna-a diretamente
        if (odom_sources.size() == 1)
        {
            const auto &odom = odom_sources[0];
            Eigen::Matrix<double, 13, 1> z;
            z << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
                odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z,
                odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;

            Eigen::Matrix<double, 13, 13> R = Eigen::MatrixXd::Zero(13, 13);
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> pose_cov(odom.pose.covariance.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> twist_cov(odom.twist.covariance.data());
            R.block<3, 3>(State::PX, State::PX) = pose_cov.block<3, 3>(0, 0);
            // Atenção: A covariância do pose tem 6x6. O quaternião precisa de 4x4.
            // Aqui assumimos que a covariância da orientação está nos últimos 3x3 do pose_cov.
            // O ideal é ter uma matriz 7x7 para a pose.
            R.block<3, 3>(State::QX, State::QX) = pose_cov.block<3, 3>(3, 3);
            R(State::QW, State::QW) = (R(State::QX, State::QX) + R(State::QY, State::QY) + R(State::QZ, State::QZ)) / 3.0; // Estimativa para R de QW
            R.block<3, 3>(State::VX, State::VX) = twist_cov.block<3, 3>(0, 0);
            R.block<3, 3>(State::WX, State::WX) = twist_cov.block<3, 3>(3, 3);

            return std::make_pair(z, R);
        }

        // Acumuladores para a fusão ponderada
        Eigen::MatrixXd R_inv_sum = Eigen::MatrixXd::Zero(13, 13);
        Eigen::VectorXd z_weighted_sum = Eigen::VectorXd::Zero(13);

        for (const auto &odom : odom_sources)
        {
            Eigen::VectorXd z_k(13);
            z_k << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
                odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z,
                odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;

            Eigen::MatrixXd R_k = Eigen::MatrixXd::Zero(13, 13);
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> pose_cov(odom.pose.covariance.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 6>> twist_cov(odom.twist.covariance.data());
            R_k.block<3, 3>(State::PX, State::PX) = pose_cov.block<3, 3>(0, 0);
            R_k.block<3, 3>(State::QX, State::QX) = pose_cov.block<3, 3>(3, 3);
            R_k(State::QW, State::QW) = (R_k(State::QX, State::QX) + R_k(State::QY, State::QY) + R_k(State::QZ, State::QZ)) / 3.0;
            R_k.block<3, 3>(State::VX, State::VX) = twist_cov.block<3, 3>(0, 0);
            R_k.block<3, 3>(State::WX, State::WX) = twist_cov.block<3, 3>(3, 3);

            // Adiciona um pequeno valor à diagonal para garantir que a matriz seja invertível
            R_k.diagonal().array() += 1e-9;

            // Calcula o peso (inversa da covariância)
            Eigen::MatrixXd R_k_inv = R_k.inverse();

            // Acumula os pesos e a soma ponderada
            R_inv_sum += R_k_inv;
            z_weighted_sum += R_k_inv * z_k;
        }

        // Covariância fundida é a inversa da soma dos pesos
        Eigen::MatrixXd R_fused = R_inv_sum.inverse();
        // Medição fundida é a soma ponderada multiplicada pela nova covariância
        Eigen::VectorXd z_fused = R_fused * z_weighted_sum;

        if (is_debug_)
        {
            RCLCPP_DEBUG_STREAM(logger_, "Fusão de odometria concluída. Norma de z_fused: " << z_fused.norm());
        }

        return std::make_pair(z_fused, R_fused);
    }

    // /**
    //  * @brief Funde múltiplas fontes de odometria usando a ponderação pela inversa da variância.
    //  * @param measurements O pacote de medições contendo as fontes de odometria.
    //  * @return Um std::optional contendo um par com o vetor de medição fundido (z) e
    //  * a sua matriz de covariância combinada (R). Retorna um optional vazio se nenhuma
    //  * fonte de odometria estiver disponível.
    //  */
    // std::optional<std::pair<Eigen::Matrix<double, 13, 1>, Eigen::Matrix<double, 13, 13>>>
    // DroneEKF::fuse_odometry(const MeasurementPackage &measurements)
    // {
    //     std::vector<nav_msgs::msg::Odometry> odom_sources;
    //     if (measurements.px4_odometry)
    //         odom_sources.push_back(measurements.px4_odometry.value());
    //     if (measurements.openvins)
    //         odom_sources.push_back(measurements.openvins.value());
    //     if (measurements.fast_lio)
    //         odom_sources.push_back(measurements.fast_lio.value());

    //     if (odom_sources.empty())
    //     {
    //         return std::nullopt; // Nenhuma fonte para fundir
    //     }

    //     if (is_debug_)
    //     {
    //         RCLCPP_DEBUG_STREAM(logger_, "Iniciando a fusão de " << odom_sources.size() << " fontes de odometria.");
    //     }

    //     // Se houver apenas uma fonte, retorna-a diretamente, evitando cálculos desnecessários
    //     if (odom_sources.size() == 1)
    //     {
    //         const auto &odom = odom_sources[0];
    //         Eigen::Matrix<double, 13, 1> z;
    //         z << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
    //             odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
    //             odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z,
    //             odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;

    //         Eigen::Matrix<double, 13, 13> R = Eigen::MatrixXd::Zero(13, 13);
    //         Eigen::Map<const Eigen::Matrix<double, 6, 6>> pose_cov(odom.pose.covariance.data());
    //         Eigen::Map<const Eigen::Matrix<double, 6, 6>> twist_cov(odom.twist.covariance.data());
    //         R.block<3, 3>(State::PX, State::PX) = pose_cov.block<3, 3>(0, 0);
    //         R.block<3, 3>(State::QX, State::QX) = pose_cov.block<3, 3>(3, 3);
    //         R(State::QW, State::QW) = (R(State::QX, State::QX) + R(State::QY, State::QY) + R(State::QZ, State::QZ)) / 3.0; // Estimativa para R de QW
    //         R.block<3, 3>(State::VX, State::VX) = twist_cov.block<3, 3>(0, 0);
    //         R.block<3, 3>(State::WX, State::WX) = twist_cov.block<3, 3>(3, 3);

    //         return std::make_pair(z, R);
    //     }

    //     // Acumuladores para a fusão ponderada
    //     Eigen::MatrixXd R_inv_sum = Eigen::MatrixXd::Zero(13, 13);
    //     Eigen::VectorXd z_weighted_sum = Eigen::VectorXd::Zero(13);

    //     for (const auto &odom : odom_sources)
    //     {
    //         Eigen::VectorXd z_k(13);
    //         z_k << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
    //             odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
    //             odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z,
    //             odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;

    //         Eigen::MatrixXd R_k = Eigen::MatrixXd::Zero(13, 13);
    //         Eigen::Map<const Eigen::Matrix<double, 6, 6>> pose_cov(odom.pose.covariance.data());
    //         Eigen::Map<const Eigen::Matrix<double, 6, 6>> twist_cov(odom.twist.covariance.data());
    //         R_k.block<3, 3>(State::PX, State::PX) = pose_cov.block<3, 3>(0, 0);
    //         R_k.block<3, 3>(State::QX, State::QX) = pose_cov.block<3, 3>(3, 3);
    //         R_k(State::QW, State::QW) = (R_k(State::QX, State::QX) + R_k(State::QY, State::QY) + R_k(State::QZ, State::QZ)) / 3.0;
    //         R_k.block<3, 3>(State::VX, State::VX) = twist_cov.block<3, 3>(0, 0);
    //         R_k.block<3, 3>(State::WX, State::WX) = twist_cov.block<3, 3>(3, 3);

    //         // 3. Garante uma covariância mínima para evitar singularidade
    //         // Itera pela diagonal de R_k. Se um valor for muito pequeno ou zero,
    //         // substitui-o por um valor pequeno, mas seguro (ex: 1e-6).
    //         for (int i = 0; i < 13; ++i)
    //         {
    //             if (R_k(i, i) < 1e-9)
    //             {
    //                 R_k(i, i) = 1e-6; // Piso de covariância para garantir estabilidade
    //             }
    //         }

    //         Eigen::MatrixXd R_k_inv = R_k.inverse();
    //         R_inv_sum += R_k_inv;
    //         z_weighted_sum += R_k_inv * z_k;
    //     }

    //     Eigen::MatrixXd R_fused = R_inv_sum.inverse();
    //     Eigen::VectorXd z_fused = R_fused * z_weighted_sum;

    //     if (is_debug_)
    //     {
    //         RCLCPP_DEBUG_STREAM(logger_, "Fusão de odometria concluída. Norma de z_fused: " << z_fused.norm());
    //     }

    //     return std::make_pair(z_fused, R_fused);
    // }

    // /**
    //  * @brief Executa o passo de correção do EKF usando as medições disponíveis.
    //  * Este método atua como um despachante, chamando os métodos de correção
    //  * específicos para cada sensor presente no `MeasurementPackage`.
    //  * @param measurements Um pacote contendo os dados dos sensores disponíveis.
    //  */
    // void DroneEKF::correct(const MeasurementPackage &measurements)
    // {
    //     if (is_debug_)
    //     {
    //         RCLCPP_DEBUG_STREAM(logger_, "\n--- CORRECT (com Pré-Fusão) ---");
    //         RCLCPP_DEBUG_STREAM(logger_, "Estado (x) ANTES: ");
    //         RCLCPP_DEBUG_STREAM(logger_, "  ├ Posição (p):    " << x_.segment<3>(State::PX).transpose());
    //         RCLCPP_DEBUG_STREAM(logger_, "  ├ Quatérnion (q): " << x_.segment<4>(State::QW).transpose());
    //         RCLCPP_DEBUG_STREAM(logger_, "  ├ Vel. Linear (v): " << x_.segment<3>(State::VX).transpose());
    //         RCLCPP_DEBUG_STREAM(logger_, "  └ Vel. Angular (w):" << x_.segment<3>(State::WX).transpose());
    //         RCLCPP_DEBUG_STREAM(logger_, "Incerteza (trace P) ANTES: " << P_.trace());
    //     }

    //     // --- 1. Fundir as medições de odometria ---
    //     auto fused_odom = fuse_odometry(measurements);

    //     // --- 2. Calcular o tamanho total e inicializar matrizes ---
    //     int total_measurements = (fused_odom ? 13 : 0) +
    //                              (measurements.imu ? 3 : 0) +
    //                              (measurements.gps ? 2 : 0);

    //     if (total_measurements == 0)
    //     {
    //         RCLCPP_DEBUG_STREAM(logger_, "Nenhuma medição disponível para correção.");
    //         return;
    //     }

    //     Eigen::VectorXd z(total_measurements);
    //     Eigen::MatrixXd H = Eigen::MatrixXd::Zero(total_measurements, STATES);
    //     Eigen::MatrixXd R = Eigen::MatrixXd::Zero(total_measurements, total_measurements);
    //     int current_row = 0;

    //     // --- 3. Adicionar a odometria (já fundida) às matrizes principais ---
    //     if (fused_odom)
    //     {
    //         H.block<13, 13>(current_row, 0).setIdentity();
    //         z.segment<13>(current_row) = fused_odom->first;
    //         R.block<13, 13>(current_row, current_row) = fused_odom->second;
    //         current_row += 13;
    //     }

    //     // --- 4. Adicionar outras medições ---
    //     if (measurements.imu)
    //     {
    //         const auto &imu = measurements.imu.value();
    //         H.block<3, 3>(current_row, State::WX).setIdentity();
    //         z.segment<3>(current_row) << imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z;
    //         R.block<3, 3>(current_row, current_row) = Eigen::Map<const Eigen::Matrix<double, 3, 3>>(imu.angular_velocity_covariance.data());
    //         current_row += 3;
    //     }

    //     if (measurements.gps)
    //     {
    //         const auto &gps = measurements.gps.value();
    //         H(current_row, State::PX) = 1.0;
    //         H(current_row + 1, State::PY) = 1.0;
    //         z.segment<2>(current_row) << gps.x, gps.y;
    //         R(current_row, current_row) = 0.5;
    //         R(current_row + 1, current_row + 1) = 0.5;
    //         current_row += 2;
    //     }

    //     // --- 5. Executar a Atualização do Filtro ---
    //     if (z.hasNaN())
    //     {
    //         RCLCPP_WARN_STREAM(logger_, "Vetor de medição 'z' contém NaN! Correção ignorada.");
    //         return;
    //     }

    //     Eigen::VectorXd y = z - (H * x_);

    //     // Tratamento especial para o erro do quaternião, se a odometria foi fundida
    //     if (fused_odom)
    //     {
    //         Eigen::Quaterniond q_z(z(State::QW), z(State::QX), z(State::QY), z(State::QZ));
    //         q_z.normalize();
    //         Eigen::Quaterniond q_x(x_(State::QW), x_(State::QX), x_(State::QY), x_(State::QZ));
    //         Eigen::Quaterniond q_error = q_z * q_x.inverse();
    //         y.segment<3>(State::QX) = 2.0 * q_error.vec();
    //         y(State::QW) = 0;
    //     }

    //     Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    //     if (S.hasNaN() || std::abs(S.determinant()) < 1e-9)
    //     {
    //         RCLCPP_WARN_STREAM(logger_, "Matriz S é singular ou contém NaN. Correção ignorada.");
    //         return;
    //     }
    //     Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    //     x_ += K * y;
    //     Eigen::Matrix<double, STATES, STATES> I;
    //     I.setIdentity();
    //     P_ = (I - K * H) * P_;
    //     x_.segment<4>(State::QW).normalize();

    //     if (is_debug_)
    //     {
    //         RCLCPP_DEBUG_STREAM(logger_, "Saída: Estado (x) DEPOIS: ");
    //         RCLCPP_DEBUG_STREAM(logger_, "  ├ Posição (p):    " << x_.segment<3>(State::PX).transpose());
    //         RCLCPP_DEBUG_STREAM(logger_, "  ├ Quatérnion (q): " << x_.segment<4>(State::QW).transpose());
    //         RCLCPP_DEBUG_STREAM(logger_, "  ├ Vel. Linear (v): " << x_.segment<3>(State::VX).transpose());
    //         RCLCPP_DEBUG_STREAM(logger_, "  └ Vel. Angular (w):" << x_.segment<3>(State::WX).transpose());
    //         RCLCPP_DEBUG_STREAM(logger_, "Saída: Incerteza (trace P) DEPOIS: " << P_.trace());
    //     }
    // }

} // namespace laser_uav_lib