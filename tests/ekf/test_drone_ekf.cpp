#include <gtest/gtest.h>
#include <laser_uav_lib/kalman_filter/drone_ekf/drone_ekf.hpp>
#include <laser_uav_lib/attitude_converter/attitude_converter.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

// Defina uma tolerância padrão para as verificações de estabilidade
constexpr double STATE_TOLERANCE = 1e-9;

// Constantes de teste para os parâmetros do drone
const double TEST_MASS = 1.60;                             // mass do YAML
const double TEST_ARM_LENGTH = 0.258;                      // calculado de motors_positions
const double TEST_THRUST_COEFF = 1.0;                      // precisa ser calculado ou estimado
const double TEST_TORQUE_COEFF = TEST_THRUST_COEFF * 0.59; // c_tau do YAML
const Eigen::Matrix3d TEST_INERTIA =
    Eigen::Vector3d(0.4953, 0.4953, 0.3413).asDiagonal(); // inertia do YAML

const Eigen::Matrix<double, 4, 2> TEST_MOTOR_POSITIONS = (Eigen::Matrix<double, 4, 2>() << 0.185, -0.18, //< Motor 0 (Frontal-Direito)
                                                          -0.185, 0.18,                                  //< Motor 1 (Traseiro-Esquerdo)
                                                          0.185, 0.18,                                   //< Motor 2 (Frontal-Esquerdo)
                                                          -0.185, -0.18)                                 //< Motor 3 (Traseiro-Direito)
                                                             .finished();

/**
 * @class DroneEKFTest
 * @brief Conjunto de testes para a classe DroneEKF.
 * * Esta classe de teste (test fixture) configura um ambiente comum para todos os testes
 * do Extended Kalman Filter (EKF) do drone. Ela inicializa uma instância do EKF
 * com parâmetros padrão antes da execução de cada teste.
 */
class DroneEKFTest : public ::testing::Test
{
protected:
    std::unique_ptr<laser_uav_lib::DroneEKF> ekf;
    const double hover_thrust_per_motor = (TEST_MASS * 9.80665) / 4.0;

    /**
     * @brief Configura o ambiente de teste.
     * * Este método é executado antes de cada teste. Ele cria uma nova instância
     * do DroneEKF para garantir que os testes sejam independentes e comecem
     * a partir de um estado limpo.
     */
    void SetUp() override
    {
        ekf = std::make_unique<laser_uav_lib::DroneEKF>(
            TEST_MASS, TEST_MOTOR_POSITIONS, TEST_THRUST_COEFF, TEST_TORQUE_COEFF, TEST_INERTIA, "INFO");
    }

    /**
     * @brief Destrói o ambiente de teste.
     * * Este método é executado após cada teste. Ele garante que a memória alocada
     * para o EKF seja liberada corretamente.
     */
    void TearDown() override
    {
        ekf.reset();
    }

    /**
     * @brief Executa a etapa de predição do filtro por múltiplos passos.
     * @param u Vetor de controle (força de cada motor).
     * @param steps Número de passos de simulação a serem executados.
     * @param dt Intervalo de tempo entre os passos.
     */
    void run_simulation(const Eigen::Vector4d &u, int steps, double dt)
    {
        for (int i = 0; i < steps; ++i)
        {
            ekf->predict(u, dt);
        }
    }

    /**
     * @brief Executa um ciclo completo de predição e correção por múltiplos passos.
     * @param u Vetor de controle para a etapa de predição.
     * @param measurements Pacote de medições para a etapa de correção.
     * @param steps Número de ciclos de simulação (predição + correção) a serem executados.
     * @param dt Intervalo de tempo entre os passos.
     */
    void run_full_simulation(const Eigen::Vector4d &u,
                             const laser_uav_lib::MeasurementPackage &measurements,
                             int steps,
                             double dt)
    {
        for (int i = 0; i < steps; ++i)
        {
            ekf->predict(u, dt);
            ekf->correct(measurements);
        }
    }

    /**
     * @brief Função auxiliar para imprimir o estado do drone de forma formatada.
     * @param title Título para a seção de impressão.
     * @param state Vetor de estado do EKF.
     */
    void print_state(const std::string &title, const Eigen::VectorXd &state)
    {
        // Usamos 'auto' para que a função seja compatível com diferentes tipos de vetores do Eigen
        // e assumimos que a enumeração State está disponível para obter os índices.
        // Se não estiver, podemos usar os valores numéricos diretamente (PX=0, QW=3, VX=7, WX=10).

        std::cout << "--- " << title << " ---" << std::endl;
        std::cout << "     Estado (x):" << std::endl;
        std::cout << "     ├ Posição (p):    " << state.template segment<3>(laser_uav_lib::State::PX).transpose() << std::endl;
        std::cout << "     ├ Quatérnion (q): " << state.template segment<4>(laser_uav_lib::State::QW).transpose() << std::endl;
        std::cout << "     ├ Vel. Linear (v):  " << state.template segment<3>(laser_uav_lib::State::VX).transpose() << std::endl;
        std::cout << "     └ Vel. Angular (w): " << state.template segment<3>(laser_uav_lib::State::WX).transpose() << std::endl;
        std::cout << "----------------------------------------" << std::endl
                  << std::endl;
    }
};

// =============================================================================
// TESTES BÁSICOS DE INICIALIZAÇÃO E COMPORTAMENTO FUNDAMENTAL
// =============================================================================

/**
 * @test Initialization
 * @brief Verifica se o estado inicial do EKF está correto.
 * * Confirma que, após a inicialização, o drone está no estado padrão:
 * - Posição e velocidade (linear e angular) são nulas.
 * - A orientação é representada por um quaternião identidade (QW=1, QX=QY=QZ=0).
 * - A covariância inicial é positiva, indicando alguma incerteza inicial.
 */
TEST_F(DroneEKFTest, Initialization)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");

    const auto &state = ekf->get_state();
    const auto &covariance = ekf->get_covariance();

    EXPECT_DOUBLE_EQ(state(laser_uav_lib::State::QW), 1.0);
    EXPECT_DOUBLE_EQ(state.segment<3>(laser_uav_lib::State::PX).norm(), 0.0);
    EXPECT_DOUBLE_EQ(state.segment<3>(laser_uav_lib::State::QX).norm(), 0.0);
    EXPECT_DOUBLE_EQ(state.segment<3>(laser_uav_lib::State::VX).norm(), 0.0);
    EXPECT_DOUBLE_EQ(state.segment<3>(laser_uav_lib::State::WX).norm(), 0.0);
    ASSERT_GT(covariance.trace(), 0.0);
}

/**
 * @test PredictionIncreasesUncertainty
 * @brief Verifica se a etapa de predição aumenta a incerteza do estado.
 * * Executa um único passo de predição e confirma que o traço da matriz de
 * covariância aumentou. Isso é esperado, pois a predição propaga o estado
 * no tempo, acumulando incerteza do modelo de processo.
 */
TEST_F(DroneEKFTest, PredictionIncreasesUncertainty)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");
    Eigen::Vector4d u_hover = Eigen::Vector4d::Constant(hover_thrust_per_motor + 0.2);
    auto initial_trace = ekf->get_covariance().trace();
    run_simulation(u_hover, 5, 0.02);
    auto final_trace = ekf->get_covariance().trace();
    EXPECT_GT(final_trace, initial_trace);
}

/**
 * @test CorrectionDecreasesUncertainty
 * @brief Verifica se a etapa de correção diminui a incerteza do estado.
 * * Após um passo de predição, uma medição de odometria é fornecida ao filtro.
 * O teste confirma que o traço da covariância diminui após a correção,
 * pois a nova informação da medição reduz a incerteza do estado.
 */
TEST_F(DroneEKFTest, CorrectionDecreasesUncertainty)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");

    // --- Arrange ---
    // Captura a incerteza inicial do filtro.
    auto initial_trace = ekf->get_covariance().trace();

    laser_uav_lib::MeasurementPackage measurements;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.00;

    odom_msg.pose.pose.orientation.w = 1.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;

    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    for (int i = 0; i < 36; ++i)
    {
        odom_msg.pose.covariance[i] = (i % 7 == 0) ? 0.001 : 0.0;
        odom_msg.twist.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
    }

    measurements.px4_odometry = odom_msg;

    // --- Act ---
    // Executa 5 ciclos completos de predição e correção.
    run_full_simulation(Eigen::Vector4d::Zero(), measurements, 100, 0.02);

    auto final_trace = ekf->get_covariance().trace();

    // --- Assert ---
    // A asserção principal: a incerteza final deve ser menor que a inicial,
    // mostrando que o filtro está usando as medições para refinar sua estimativa.
    EXPECT_LT(final_trace, initial_trace);

    // Log para visualização do resultado do teste
    std::cout << "\n--- Teste de Convergência da Correção ---" << std::endl;
    std::cout << "Traço Inicial: " << initial_trace << std::endl;
    std::cout << "Traço Final (após 5 ciclos): " << final_trace << std::endl;
    std::cout << "Resultado: " << (final_trace < initial_trace ? "PASSOU" : "FALHOU") << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
}

/**
 * @test QuaternionNormalization
 * @brief Garante que o quaternião de orientação permaneça normalizado.
 * * Após um grande número de passos de simulação, o teste verifica se a norma
 * do quaternião do estado continua muito próxima de 1.0. Isso é crucial
 * para a estabilidade numérica e para uma representação válida da orientação.
 */
TEST_F(DroneEKFTest, QuaternionNormalization)
{
    ekf->set_verbosity("SILENT");

    run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor), 1000, 0.01);
    const auto &state = ekf->get_state();
    double norm_q = std::sqrt(
        std::pow(state(laser_uav_lib::State::QW), 2) +
        std::pow(state(laser_uav_lib::State::QX), 2) +
        std::pow(state(laser_uav_lib::State::QY), 2) +
        std::pow(state(laser_uav_lib::State::QZ), 2));
    EXPECT_NEAR(norm_q, 1.0, 1e-6);
}

/**
 * @test ResetRestoresInitialState
 * @brief Verifica se a função reset restaura o estado inicial do filtro.
 * * Simula um voo e, em seguida, chama o método `reset()`. O teste confirma
 * que o estado do filtro (posição, velocidade, orientação) retorna aos
 * valores padrão de inicialização.
 */
TEST_F(DroneEKFTest, ResetRestoresInitialState)
{
    ekf->set_verbosity("SILENT");

    run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor + 1.0), 100, 0.02);
    ekf->reset();

    const auto &state = ekf->get_state();
    EXPECT_DOUBLE_EQ(state(laser_uav_lib::State::QW), 1.0);
    EXPECT_DOUBLE_EQ(state.segment<3>(laser_uav_lib::State::PX).norm(), 0.0);
    EXPECT_DOUBLE_EQ(state.segment<3>(laser_uav_lib::State::VX).norm(), 0.0);
}

// // =============================================================================
// // TESTES DE CENÁRIOS DE VOO BÁSICOS
// // =============================================================================

/**
 * @test HoverShouldRemainStationary
 * @brief Simula um voo pairado e verifica a estabilidade.
 * * Aplica um comando de controle correspondente à força necessária para anular
 * a gravidade (hover). O teste verifica se o drone permanece essencialmente
 * estacionário, com posição, velocidade linear e velocidade angular próximas de zero.
 */
TEST_F(DroneEKFTest, HoverShouldRemainStationary)
{
    ekf->set_verbosity("SILENT");
    Eigen::Vector4d u_hover = Eigen::Vector4d::Constant(hover_thrust_per_motor);
    run_simulation(u_hover, 100, 0.02);
    const auto &state = ekf->get_state();

    EXPECT_NEAR(state.segment<3>(laser_uav_lib::State::PX).norm(), 0.0, 1e-3);
    EXPECT_NEAR(state.segment<3>(laser_uav_lib::State::VX).norm(), 0.0, 1e-3);
    EXPECT_NEAR(state.segment<3>(laser_uav_lib::State::WX).norm(), 0.0, 1e-3);
}

/**
 * @test FreeFallWhenNoThrust
 * @brief Simula uma queda livre sem nenhuma força dos motores.
 * * Define o comando de controle como zero e simula. O teste verifica se
 * o drone se comporta como esperado sob a ação da gravidade: a posição Z
 * se torna negativa e a velocidade Z também se torna negativa.
 */
TEST_F(DroneEKFTest, FreeFallWhenNoThrust)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");

    run_simulation(Eigen::Vector4d::Zero(), 100, 0.02);
    const auto &state = ekf->get_state();
    EXPECT_LT(state(laser_uav_lib::State::PZ), 0.0);
    EXPECT_LT(state(laser_uav_lib::State::VZ), 0.0);
}

// // =============================================================================
// // TESTES DE MOVIMENTO VERTICAL
// // =============================================================================

/**
 * @test UpwardMovement
 * @brief Testa o movimento vertical para cima.
 * * Aplica uma força nos motores maior que a necessária para pairar.
 * Verifica se o drone ganha altitude (posição Z > 0) e adquire velocidade
 * vertical positiva (velocidade Z > 0), sem desvios significativos nos eixos X e Y.
 */
TEST_F(DroneEKFTest, UpwardMovement)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");

    Eigen::Vector4d u_up = Eigen::Vector4d::Constant(hover_thrust_per_motor + 1.0);
    run_simulation(u_up, 50, 0.02);
    const auto &state = ekf->get_state();

    EXPECT_GT(state(laser_uav_lib::State::PZ), 0.0);
    EXPECT_GT(state(laser_uav_lib::State::VZ), 0.0);
    EXPECT_NEAR(state(laser_uav_lib::State::PX), 0.0, 1e-4);
    EXPECT_NEAR(state(laser_uav_lib::State::PY), 0.0, 1e-4);
}

/**
 * @test DownwardMovement
 * @brief Testa o movimento vertical para baixo.
 * * Primeiro, o drone sobe um pouco. Em seguida, aplica-se uma força menor
 * que a necessária para pairar. O teste verifica se o drone adquire
 * uma velocidade vertical negativa (velocidade Z < 0), indicando descida.
 */
TEST_F(DroneEKFTest, DownwardMovement)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");

    // Primeiro sobe um pouco
    run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor + 1.0), 5, 0.02);

    // Agora desce
    run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor - 1.0), 10, 0.02);

    // Agora desce
    // run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor + 1.0), 5, 0.02);

    // run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor), 100, 0.02);

    const auto &state = ekf->get_state();

    // Verifica se a velocidade Z é negativa (descida)
    EXPECT_LT(state(laser_uav_lib::State::VZ), 0.0);
}

// // =============================================================================
// // TESTES DE MOVIMENTO HORIZONTAL
// // =============================================================================

/**
 * @test ForwardMovement
 * @brief Testa o movimento para frente (eixo X positivo).
 * * Aplica um comando diferencial nos motores para gerar um torque de arfagem (pitch)
 * negativo, fazendo o drone inclinar e se mover para frente. Verifica se a
 * posição e a velocidade no eixo X se tornam positivas e se a orientação (QY)
 * reflete a inclinação para frente.
 */
TEST_F(DroneEKFTest, ForwardMovement)
{
    double diff = 0.5;
    Eigen::Vector4d u_forward(
        hover_thrust_per_motor - diff,
        hover_thrust_per_motor + diff,
        hover_thrust_per_motor - diff,
        hover_thrust_per_motor + diff);

    run_simulation(u_forward, 50, 0.02);
    const auto &state = ekf->get_state();
    EXPECT_GT(state(laser_uav_lib::State::PX), 0.0);
    EXPECT_GT(state(laser_uav_lib::State::VX), 0.0);
    EXPECT_GT(state(laser_uav_lib::State::QY), 0.0);

    EXPECT_NEAR(state(laser_uav_lib::State::PY), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::PZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::VY), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::VZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QX), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QZ), 0.0, STATE_TOLERANCE);
}

/**
 * @test BackwardMovement
 * @brief Testa o movimento para trás (eixo X negativo).
 * * Aplica um comando diferencial nos motores para gerar um torque de arfagem (pitch)
 * positivo, fazendo o drone inclinar e se mover para trás. Verifica se a
 * posição e a velocidade no eixo X se tornam negativas.
 */
TEST_F(DroneEKFTest, BackwardMovement)
{
    double diff = 0.5;
    Eigen::Vector4d u_backward(
        hover_thrust_per_motor + diff,
        hover_thrust_per_motor - diff,
        hover_thrust_per_motor + diff,
        hover_thrust_per_motor - diff);

    run_simulation(u_backward, 50, 0.02);
    const auto &state = ekf->get_state();
    EXPECT_LT(state(laser_uav_lib::State::PX), 0.0);
    EXPECT_LT(state(laser_uav_lib::State::VX), 0.0);
    EXPECT_LT(state(laser_uav_lib::State::QY), 0.0);

    EXPECT_NEAR(state(laser_uav_lib::State::PY), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::PZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::VY), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::VZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QX), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QZ), 0.0, STATE_TOLERANCE);
}

/**
 * @test RightwardMovement
 * @brief Testa o movimento para a direita (eixo Y positivo).
 * * Aplica um comando diferencial para gerar um torque de rolagem (roll) positivo,
 * fazendo o drone inclinar e se mover para a direita. Verifica se a posição e
 * a velocidade no eixo Y se tornam positivas.
 */
TEST_F(DroneEKFTest, RightwardMovement)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");

    double diff = 0.5;
    Eigen::Vector4d u_right(
        hover_thrust_per_motor - diff,
        hover_thrust_per_motor + diff,
        hover_thrust_per_motor + diff,
        hover_thrust_per_motor - diff);

    run_simulation(u_right, 50, 0.02);
    const auto &state = ekf->get_state();
    EXPECT_LT(state(laser_uav_lib::State::PY), 0.0);
    EXPECT_LT(state(laser_uav_lib::State::VY), 0.0);
    EXPECT_GT(state(laser_uav_lib::State::QX), 0.0);

    EXPECT_NEAR(state(laser_uav_lib::State::PX), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::PZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::VX), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::VZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QY), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QZ), 0.0, STATE_TOLERANCE);
}

/**
 * @test LeftwardMovement
 * @brief Testa o movimento para a esquerda (eixo Y negativo).
 * * Aplica um comando diferencial para gerar um torque de rolagem (roll) negativo,
 * fazendo o drone inclinar e se mover para a esquerda. Verifica se a posição e
 * a velocidade no eixo Y se tornam negativas.
 */
TEST_F(DroneEKFTest, LeftwardMovement)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");
    double diff = 0.5;
    Eigen::Vector4d u_left(
        hover_thrust_per_motor + diff,
        hover_thrust_per_motor - diff,
        hover_thrust_per_motor - diff,
        hover_thrust_per_motor + diff);

    run_simulation(u_left, 50, 0.02);
    const auto &state = ekf->get_state();
    EXPECT_GT(state(laser_uav_lib::State::PY), 0.0);
    EXPECT_GT(state(laser_uav_lib::State::VY), 0.0);
    EXPECT_LT(state(laser_uav_lib::State::QX), 0.0);

    EXPECT_NEAR(state(laser_uav_lib::State::PX), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::PZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::VX), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::VZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QY), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QZ), 0.0, STATE_TOLERANCE);
}

// // =============================================================================
// // TESTES DE ROTAÇÃO
// // =============================================================================

/**
 * @test YawRotation
 * @brief Testa a rotação em torno do eixo Z (guinada).
 * * Aplica um comando diferencial que gera um torque de guinada (yaw). Verifica se
 * a velocidade angular em Z (WZ) se torna positiva e se o ângulo de guinada,
 * extraído do quaternião final, também é positivo.
 */
TEST_F(DroneEKFTest, YawRotation)
{
    // ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    ekf->set_verbosity("DEBUG");
    double diff = 0.1;
    Eigen::Vector4d u_yaw(
        hover_thrust_per_motor + diff,
        hover_thrust_per_motor + diff,
        hover_thrust_per_motor - diff,
        hover_thrust_per_motor - diff);

    run_simulation(u_yaw, 500, 0.01);
    const auto &state = ekf->get_state();

    EXPECT_GT(state(laser_uav_lib::State::WZ), 0.0);
    EXPECT_GT(state(laser_uav_lib::State::QZ), 0.0);

    // EXPECT_NEAR(state(laser_uav_lib::State::PX), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::PY), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::PZ), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::VX), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::VY), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::VZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QX), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QY), 0.0, STATE_TOLERANCE);
}

/**
 * @test YawRotation
 * @brief Testa a rotação em torno do eixo Z (guinada).
 * * Aplica um comando diferencial que gera um torque de guinada (yaw). Verifica se
 * a velocidade angular em Z (WZ) se torna positiva e se o ângulo de guinada,
 * extraído do quaternião final, também é positivo.
 */
TEST_F(DroneEKFTest, TwoYawRotation)
{
    // ekf->set_verbosity("SILENT");
    ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");
    double diff = 0.1;
    Eigen::Vector4d u_yaw(
        hover_thrust_per_motor - diff,
        hover_thrust_per_motor - diff,
        hover_thrust_per_motor + diff,
        hover_thrust_per_motor + diff);

    run_simulation(u_yaw, 300, 0.01);
    const auto &state = ekf->get_state();

    EXPECT_LT(state(laser_uav_lib::State::WZ), 0.0);
    EXPECT_LT(state(laser_uav_lib::State::QZ), 0.0);

    // EXPECT_NEAR(state(laser_uav_lib::State::PX), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::PY), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::PZ), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::VX), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::VY), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::VZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QX), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QY), 0.0, STATE_TOLERANCE);
}

/**
 * @test PitchRotation
 * @brief Testa a rotação em torno do eixo Y (arfagem).
 * * Aplica um comando diferencial para gerar um torque de arfagem (pitch).
 * Verifica se a velocidade angular em Y (WY) se torna positiva e se o componente
 * QY do quaternião também se torna positivo, indicando a rotação.
 */
TEST_F(DroneEKFTest, PitchRotation)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");
    double diff = 0.2;
    Eigen::Vector4d u_pitch(
        hover_thrust_per_motor - diff,
        hover_thrust_per_motor + diff,
        hover_thrust_per_motor - diff,
        hover_thrust_per_motor + diff);

    run_simulation(u_pitch, 50, 0.02);
    const auto &state = ekf->get_state();
    EXPECT_GT(state(laser_uav_lib::State::WY), 0.0);
    EXPECT_GT(state(laser_uav_lib::State::QY), 0.0);

    // EXPECT_NEAR(state(laser_uav_lib::State::PZ), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::VZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QX), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::WX), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::WZ), 0.0, STATE_TOLERANCE);
}

/**
 * @test RollRotation
 * @brief Testa a rotação em torno do eixo X (rolagem).
 * * Aplica um comando diferencial para gerar um torque de rolagem (roll).
 * Verifica se a velocidade angular em X (WX) se torna positiva e se o componente
 * QX do quaternião também se torna positivo, indicando a rotação.
 */
TEST_F(DroneEKFTest, RollRotation)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");
    double diff = 0.2;
    Eigen::Vector4d u_roll(
        hover_thrust_per_motor - diff,
        hover_thrust_per_motor + diff,
        hover_thrust_per_motor + diff,
        hover_thrust_per_motor - diff);

    run_simulation(u_roll, 50, 0.02);
    const auto &state = ekf->get_state();
    EXPECT_GT(state(laser_uav_lib::State::WX), 0.0);
    EXPECT_GT(state(laser_uav_lib::State::QX), 0.0);

    // EXPECT_NEAR(state(laser_uav_lib::State::PZ), 0.0, STATE_TOLERANCE);
    // EXPECT_NEAR(state(laser_uav_lib::State::VZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QY), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::QZ), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::WY), 0.0, STATE_TOLERANCE);
    EXPECT_NEAR(state(laser_uav_lib::State::WZ), 0.0, STATE_TOLERANCE);
}

// // =============================================================================
// // TESTES DE MANOBRAS COMPLEXAS
// // =============================================================================

/**
 * @test Takeoff
 * @brief Simula uma manobra de decolagem e estabilização.
 * * Aplica um forte impulso inicial para subir e depois reduz o impulso para
 * o nível de voo pairado. Verifica se o drone atinge uma altitude positiva
 * e estabiliza sua velocidade vertical perto de zero.
 */
TEST_F(DroneEKFTest, Takeoff)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");
    Eigen::Vector4d u_takeoff = Eigen::Vector4d::Constant(hover_thrust_per_motor + 2.0);
    run_simulation(u_takeoff, 25, 0.02);

    Eigen::Vector4d u_takeoff_2 = Eigen::Vector4d::Constant(hover_thrust_per_motor - 2.0);
    run_simulation(u_takeoff_2, 25, 0.02);

    Eigen::Vector4d u_hover = Eigen::Vector4d::Constant(hover_thrust_per_motor);
    run_simulation(u_hover, 75, 0.02);

    const auto &state = ekf->get_state();
    EXPECT_GT(state(laser_uav_lib::State::PZ), 0.1);
    EXPECT_NEAR(state(laser_uav_lib::State::VZ), 0.0, 0.2);
}

/**
 * @test Land
 * @brief Simula uma manobra de pouso.
 * * Primeiro, o drone sobe para uma altitude. Em seguida, um impulso menor que
 * o de voo pairado é aplicado. O teste verifica se a altitude do drone diminui
 * em relação à altitude inicial e se ele adquire uma velocidade vertical negativa.
 */

TEST_F(DroneEKFTest, Land)
{
    // Silencia o log interno do EKF para não poluir a saída do teste
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");

    // --- FASE 1: DECOLAGEM ---
    // Aplica um forte impulso para subir e ganhar altitude.
    run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor + 2.0), 25, 0.02);

    // --- FASE 2: FRENAGEM E ESTABILIZAÇÃO ---
    // Aplica um impulso menor que o de hover para frear a subida e estabilizar perto de Vz=0.
    run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor - 2.0), 25, 0.02);
    const auto state_before_land = ekf->get_state();

    // Verifica se o drone realmente saiu do chão.
    EXPECT_GT(state_before_land(laser_uav_lib::State::PZ), 0.1);

    // --- FASE 3: POUSO CONTROLADO ---
    // Aplica um impulso fraco para iniciar uma descida controlada.
    run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor - 2.0), 25, 0.02);

    // --- FASE 4: TENTATIVA DE RECUPERAÇÃO ---
    // Aplica um impulso para frear a descida (simula a aproximação do solo).
    run_simulation(Eigen::Vector4d::Constant(hover_thrust_per_motor + 2.0), 25, 0.02);
    const auto state_after_land = ekf->get_state();

    EXPECT_LT(state_after_land(laser_uav_lib::State::PZ), state_before_land(laser_uav_lib::State::PZ));

    EXPECT_NEAR(state_after_land(laser_uav_lib::State::PZ), 0.0, 1e-6);
    EXPECT_NEAR(state_after_land(laser_uav_lib::State::VZ), 0.0, 1e-6);
}

// // =============================================================================
// // TESTES DE ROBUSTEZ E CASOS ESPECIAIS
// // =============================================================================

/**
 * @test IgnoreInvalidMeasurements
 * @brief Verifica se o filtro ignora medições inválidas.
 * * Fornece ao filtro uma medição de odometria contendo valores NaN (Not a Number).
 * O teste confirma que o filtro não lança uma exceção e que seu estado
 * permanece inalterado, demonstrando robustez a dados corrompidos.
 */
TEST_F(DroneEKFTest, IgnoreInvalidMeasurements)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");
    ekf->predict(Eigen::Vector4d::Zero(), 0.02);
    auto initial_state = ekf->get_state();

    laser_uav_lib::MeasurementPackage invalid_measurement;
    nav_msgs::msg::Odometry bad_odom;
    bad_odom.pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
    invalid_measurement.px4_odometry = bad_odom;

    EXPECT_NO_THROW(ekf->correct(invalid_measurement));
    auto final_state = ekf->get_state();
    EXPECT_TRUE(final_state.isApprox(initial_state));
}

/**
 * @test CorrectionWithHighNoiseHasLittleEffect
 * @brief Verifica o comportamento do filtro com medições de alta incerteza.
 * * Fornece uma medição com valores de covariância muito altos (ruído elevado).
 * O teste espera que o estado do filtro mude muito pouco, pois o filtro deve
 * dar mais peso à sua própria predição do que a uma medição não confiável.
 */
TEST_F(DroneEKFTest, CorrectionWithHighNoiseHasLittleEffect)
{
    // ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");
    ekf->predict(Eigen::Vector4d::Constant(hover_thrust_per_motor), 0.02);
    auto state_before = ekf->get_state();
    auto covariance_before = ekf->get_covariance();

    laser_uav_lib::MeasurementPackage noisy_measurement;
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.orientation.w = 1.0;
    for (int i = 0; i < 36; ++i)
    {
        odom.pose.covariance[i] = (i % 7 == 0) ? 1e6 : 0.0;
        odom.twist.covariance[i] = (i % 7 == 0) ? 1e6 : 0.0;
    }
    noisy_measurement.px4_odometry = odom;

    ekf->correct(noisy_measurement);
    auto state_after = ekf->get_state();
    auto covariance_after = ekf->get_covariance();

    std::cout << "State before correction: " << state_before.transpose() << std::endl;
    std::cout << "State after correction: " << state_after.transpose() << std::endl;

    // O estado deve mudar muito pouco devido à alta incerteza da medição
    EXPECT_TRUE((state_after - state_before).norm() < 1e-3);

    // A covariância deve diminuir (como sempre acontece na correção),
    // mas muito pouco devido à alta incerteza da medição
    EXPECT_LT(covariance_after.trace(), covariance_before.trace());

    // A redução da covariância deve ser pequena (menos de 10% do valor original)
    double covariance_reduction = covariance_before.trace() - covariance_after.trace();
    double relative_reduction = covariance_reduction / covariance_before.trace();
    EXPECT_LT(relative_reduction, 0.1); // Menos de 10% de redução
}

/**
 * @test CovarianceDecreasesOverMultipleCorrections
 * @brief Confirma que a incerteza diminui com múltiplas correções.
 * * Executa vários ciclos de predição-correção. O teste verifica se o traço
 * da covariância total diminui e converge para um valor baixo, mostrando que
 * o filtro está efetivamente refinando sua estimativa de estado ao longo do tempo.
 */
TEST_F(DroneEKFTest, CovarianceDecreasesOverMultipleCorrections)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");
    double dt = 0.02;
    for (int i = 0; i < 10; ++i)
    {
        ekf->predict(Eigen::Vector4d::Constant(hover_thrust_per_motor), dt);
        laser_uav_lib::MeasurementPackage measurements;
        nav_msgs::msg::Odometry odom;
        odom.pose.pose.orientation.w = 1.0;
        for (int j = 0; j < 36; ++j)
            odom.pose.covariance[j] = (j % 7 == 0) ? 0.05 : 0.0;
        measurements.px4_odometry = odom;
        ekf->correct(measurements);
    }
    EXPECT_LT(ekf->get_covariance().trace(), 1.0);
}

// // =============================================================================
// // TESTES DE DESEMPENHO EM LONGO PRAZO
// // =============================================================================

/**
 * @test LongTermStability
 * @brief Avalia a estabilidade numérica do filtro em uma simulação longa.
 * * Executa o filtro por um grande número de passos com comandos de controle
 * variáveis e correções periódicas. O teste verifica se o estado permanece
 * finito (sem NaNs ou infinitos) e se a covariância não diverge, garantindo
 * a estabilidade do filtro a longo prazo.
 */
TEST_F(DroneEKFTest, LongTermStability)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");
    const int steps = 1000;
    const double dt = 0.01;

    for (int i = 0; i < steps; ++i)
    {
        // Comando alternado para testar estabilidade
        Eigen::Vector4d u = Eigen::Vector4d::Constant(hover_thrust_per_motor + 0.2 * std::sin(i * 0.1));
        ekf->predict(u, dt);

        // Correção periódica
        if (i % 10 == 0)
        {
            laser_uav_lib::MeasurementPackage meas;
            nav_msgs::msg::Odometry odom;
            odom.pose.pose.orientation.w = 1.0;
            for (int j = 0; j < 36; ++j)
                odom.pose.covariance[j] = (j % 7 == 0) ? 0.01 : 0.0;
            meas.px4_odometry = odom;
            ekf->correct(meas);
        }
    }

    const auto &state = ekf->get_state();
    EXPECT_TRUE(state.allFinite());
    EXPECT_GT(ekf->get_covariance().trace(), 0.0);
    EXPECT_LT(ekf->get_covariance().trace(), 10.0);
}

// =============================================================================
// TESTES DE DESEMPENHO
// =============================================================================

/**
 * @test ExecutionTimeBenchmark
 * @brief Mede e imprime o tempo médio de execução das funções predict e correct.
 * * Este teste não verifica a corretude do filtro, mas sim seu desempenho.
 * Ele executa as funções predict e correct um grande número de vezes para
 * calcular um tempo médio de execução estável, que é então impresso no console.
 * Isso é útil para garantir que o EKF atenda aos requisitos de tempo real.
 */
TEST_F(DroneEKFTest, ExecutionTimeBenchmark)
{
    ekf->set_verbosity("SILENT");
    // ekf->set_verbosity("INFO");
    // ekf->set_verbosity("DEBUG");

    // Número de iterações para obter uma média de tempo estável
    const int num_iterations = 1000;
    const double dt = 0.01; // Intervalo de tempo típico

    // --- Preparação dos dados de entrada ---

    // Comando de controle para a predição (voo pairado)
    Eigen::Vector4d u_hover = Eigen::Vector4d::Constant(hover_thrust_per_motor);

    // Pacote de medição para a correção (odometria)
    laser_uav_lib::MeasurementPackage measurements;
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.pose.pose.orientation.w = 1.0;
    // Preenche a covariância da medição com valores plausíveis
    for (int i = 0; i < 36; ++i)
    {
        odom_msg.pose.covariance[i] = (i % 7 == 0) ? 0.05 : 0.0;
    }
    measurements.px4_odometry = odom_msg;

    // --- Medição do Tempo da Função de PREDIÇÃO ---

    auto start_predict = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_iterations; ++i)
    {
        ekf->predict(u_hover, dt);
    }
    auto end_predict = std::chrono::high_resolution_clock::now();

    // Calcula a duração total e a média em microssegundos
    auto total_duration_predict_us = std::chrono::duration_cast<std::chrono::microseconds>(end_predict - start_predict);
    double avg_time_predict_us = static_cast<double>(total_duration_predict_us.count()) / num_iterations;

    // --- Medição do Tempo da Função de CORREÇÃO ---

    auto start_correct = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_iterations; ++i)
    {
        ekf->correct(measurements);
    }
    auto end_correct = std::chrono::high_resolution_clock::now();

    // Calcula a duração total e a média em microssegundos
    auto total_duration_correct_us = std::chrono::duration_cast<std::chrono::microseconds>(end_correct - start_correct);
    double avg_time_correct_us = static_cast<double>(total_duration_correct_us.count()) / num_iterations;

    // --- Impressão dos Resultados ---

    std::cout << "\n==================================================" << std::endl;
    std::cout << "          RESULTADOS DO BENCHMARK DE TEMPO" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "Número de iterações por função: " << num_iterations << std::endl;
    std::cout << std::fixed << std::setprecision(4); // Formata a saída para 4 casas decimais
    std::cout << "Tempo médio de PREDIÇÃO: " << avg_time_predict_us << " microssegundos (" << avg_time_predict_us / 1000.0 << " milissegundos)" << std::endl;
    std::cout << "Tempo médio de CORREÇÃO: " << avg_time_correct_us << " microssegundos (" << avg_time_correct_us / 1000.0 << " milissegundos)" << std::endl;
    std::cout << "==================================================\n"
              << std::endl;
}