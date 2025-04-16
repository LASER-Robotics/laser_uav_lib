#include <gtest/gtest.h>
#include <laser_uav_lib/filter/irr_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <random>

using namespace laser_uav_lib;

class IIRFilterTest : public ::testing::Test
{
protected:
    rclcpp::Logger logger = rclcpp::get_logger("IIRFilterTest");

    // Setup: Inicializa o ROS2, se necessário
    void SetUp() override
    {
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);
    }

    // Cleanup: Finaliza o ROS2, se necessário
    void TearDown() override
    {
        if (!rclcpp::ok())
            rclcpp::shutdown();
    }
};

// Teste para a resposta ao impulso do filtro
TEST_F(IIRFilterTest, ImpulseResponseTest)
{
    std::vector<double> a = {1.0, -0.5};
    std::vector<double> b = {0.5, 0.5};
    IIRFilter filter(a, b, logger);

    std::vector<double> output;
    output.push_back(filter.iterate(1.0)); // Impulso

    for (int i = 0; i < 10; ++i)
        output.push_back(filter.iterate(0.0)); // Zero após o impulso

    // Esperamos uma resposta decrescente
    EXPECT_GT(output[0], output[1]) << "Falha na resposta do impulso: " << output[0] << " não é maior que " << output[1];
    EXPECT_GT(output[1], output[2]) << "Falha na resposta do impulso: " << output[1] << " não é maior que " << output[2];

    // Log para acompanhar a saída
    for (size_t i = 0; i < output.size(); ++i)
    {
        RCLCPP_INFO(logger, "Saída do impulso %zu: %f", i, output[i]);
    }
}

// Teste para a resposta ao degrau do filtro
TEST_F(IIRFilterTest, StepResponseTest)
{
    std::vector<double> a = {1.0, -0.9};
    std::vector<double> b = {0.1};
    IIRFilter filter(a, b, logger);

    double y = 0;
    for (int i = 0; i < 100; ++i)
        y = filter.iterate(1.0); // Degrau de entrada

    // Espera que o filtro se estabilize próximo de 1.0
    EXPECT_NEAR(y, 1.0, 0.1) << "Resposta do degrau não estabilizou em 1.0. Valor final: " << y;

    RCLCPP_INFO(logger, "Resposta do degrau estabilizada em: %f", y);
}

// Teste para a entrada constante
TEST_F(IIRFilterTest, ConstantInputTest)
{
    std::vector<double> a = {1.0, -0.8};
    std::vector<double> b = {0.2};
    IIRFilter filter(a, b, logger);

    double y = 0;
    for (int i = 0; i < 50; ++i)
        y = filter.iterate(5.0); // Entrada constante

    // Saída deve estabilizar no valor da entrada
    EXPECT_NEAR(y, 5.0, 0.2) << "Saída não estabilizou no valor esperado. Saída final: " << y;

    RCLCPP_INFO(logger, "Saída com entrada constante estabilizada em: %f", y);
}

// Teste para o conteúdo do buffer do filtro
TEST_F(IIRFilterTest, BufferContentTest)
{
    std::vector<double> a = {1.0, 0.0};
    std::vector<double> b = {0.5, 0.5};
    IIRFilter filter(a, b, logger);

    filter.iterate(1.0); // Entrada de valor 1.0
    auto buffer = filter.getBuffer();

    ASSERT_EQ(buffer.size(), 2) << "Tamanho do buffer inesperado. Esperado: 2, Obtido: " << buffer.size();
    EXPECT_DOUBLE_EQ(buffer[0], 0.0) << "Valor incorreto no buffer. Esperado: 0.0, Obtido: " << buffer[0];
    EXPECT_DOUBLE_EQ(buffer[1], 1.0) << "Valor incorreto no buffer. Esperado: 1.0, Obtido: " << buffer[1];

    RCLCPP_INFO(logger, "Conteúdo do buffer: [ %f, %f ]", buffer[0], buffer[1]);
}

// Teste para a resposta do filtro a uma onda senoidal
TEST_F(IIRFilterTest, SineWaveResponse)
{
    std::vector<double> a = {1.0, -0.95};
    std::vector<double> b = {0.05};
    IIRFilter filter(a, b, logger);

    double fs = 1000.0; // Frequência de amostragem
    double freq = 10.0; // Frequência da onda senoidal

    double prev_output = 0.0;
    bool is_stable = true;

    for (int i = 0; i < 1000; ++i)
    {
        double t = i / fs;
        double input = std::sin(2 * M_PI * freq * t);
        double y = filter.iterate(input);

        // Verifica se a saída está estabilizada
        if (i > 100 && std::fabs(y - prev_output) > 0.01)
        {
            is_stable = false;
            break;
        }
        prev_output = y;
    }

    EXPECT_TRUE(is_stable) << "Resposta à onda senoidal não estabilizou após 100 amostras.";

    RCLCPP_INFO(logger, "Resposta à onda senoidal estabilizada.");
}

// Teste para a resposta do filtro ao ruído
TEST_F(IIRFilterTest, NoiseResponseTest)
{
    std::vector<double> a = {1.0, -0.8};
    std::vector<double> b = {0.3};
    IIRFilter filter(a, b, logger);

    // Gerar um vetor de ruído gaussiano
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dis(0.0, 1.0);

    double output = 0.0;
    for (int i = 0; i < 1000; ++i)
    {
        double noise_input = dis(gen); // Entrada de ruído
        output = filter.iterate(noise_input);
    }

    // Verifica se o filtro ainda está estabilizado (evita variações grandes)
    EXPECT_NEAR(output, 0.0, 1.0) << "A saída não se estabilizou corretamente após o ruído. Saída final: " << output;

    RCLCPP_INFO(logger, "Resposta ao ruído estabilizada em: %f", output);
}
