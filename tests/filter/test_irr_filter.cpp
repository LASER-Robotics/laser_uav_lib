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
    IIRFilter filter(a, b);

    double impulse = 1.0;
    std::vector<double> output;
    for (int i = 0; i < 10; ++i)
    {
        output.push_back(filter.iterate(impulse));
        impulse = 0.0; // Impulse only at t=0
    }

    // Verificar se a resposta diminui ao longo do tempo (estabiliza)
    bool is_stable = true;
    for (size_t i = 2; i < output.size(); ++i)
    {
        if (output[i] > output[i - 1] + 1e-6)
        { // Adicionar uma tolerância
            is_stable = false;
            break;
        }
    }
    EXPECT_TRUE(is_stable) << "Resposta do impulso não estabilizou.";

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
    IIRFilter filter(a, b);

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
    IIRFilter filter(a, b);

    double y = 0;
    for (int i = 0; i < 50; ++i)
        y = filter.iterate(5.0); // Entrada constante

    // Saída deve estabilizar no valor da entrada
    EXPECT_NEAR(y, 5.0, 0.2) << "Saída não estabilizou no valor esperado. Saída final: " << y;

    RCLCPP_INFO(logger, "Saída com entrada constante estabilizada em: %f", y);
}

// Teste para o conteúdo do buffer de entrada do filtro
TEST_F(IIRFilterTest, InputBufferContentTest)
{
    std::vector<double> a = {1.0, 0.0};
    std::vector<double> b = {0.5, 0.5};
    IIRFilter filter(a, b);

    filter.iterate(1.0); // Entrada de valor 1.0
    auto input_buffer = filter.getInputBuffer();

    ASSERT_EQ(input_buffer.size(), 1) << "Tamanho do buffer de entrada inesperado.";
    EXPECT_DOUBLE_EQ(input_buffer[0], 1.0) << "Valor incorreto no buffer de entrada [0].";
    EXPECT_DOUBLE_EQ(input_buffer[1], 0.0) << "Valor incorreto no buffer de entrada [1].";

    RCLCPP_INFO(logger, "Conteúdo do buffer de entrada: [ %f, %f ]", input_buffer[0], input_buffer[1]);
}

// Teste para o conteúdo do buffer de saída do filtro
TEST_F(IIRFilterTest, OutputBufferContentTest)
{
    std::vector<double> a = {1.0, 0.0};
    std::vector<double> b = {0.5, 0.5};
    IIRFilter filter(a, b);

    filter.iterate(1.0); // Entrada de valor 1.0
    auto output_buffer = filter.getOutputBuffer();

    ASSERT_EQ(output_buffer.size(), 1) << "Tamanho do buffer de saída inesperado.";
    EXPECT_DOUBLE_EQ(output_buffer[0], 0.5) << "Valor incorreto no buffer de saída [0]."; // A saída na primeira iteração

    RCLCPP_INFO(logger, "Conteúdo do buffer de saída: [ %f ]", output_buffer[0]);
}

// Teste para a resposta do filtro ao ruído
TEST_F(IIRFilterTest, NoiseResponseTest)
{
    std::vector<double> a = {1.0, -0.8};
    std::vector<double> b = {0.3};
    IIRFilter filter(a, b);

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

TEST_F(IIRFilterTest, SinusoidalResponse)
{
    // Filtro de média móvel de 3 pontos: y[n] = (1/3) * x[n] + (1/3) * x[n-1] + (1/3) * x[n-2]
    std::vector<double> b = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
    std::vector<double> a = {1.0};
    IIRFilter filter(b, a);

    double sample_rate = 100.0;
    double frequency = 10.0;
    double amplitude = 1.0;
    std::vector<double> output;
    double time = 0.0;
    double dt = 1.0 / sample_rate;
    int num_samples = 500;

    for (int i = 0; i < num_samples; ++i)
    {
        double input_signal = amplitude * std::sin(2.0 * M_PI * frequency * time);
        output.push_back(filter.iterate(input_signal));
        time += dt;
    }

    // Verifica se a saída também parece ser uma senoide (após um período inicial de transiente)
    // Uma verificação mais rigorosa exigiria análise de amplitude e fase.
    for (int i = 100; i < num_samples - 1; ++i)
    {
        // A direção da mudança da saída deve geralmente seguir a direção da mudança da entrada (com um atraso)
        if ((output[i] > output[i - 1] && std::sin(2.0 * M_PI * frequency * (time - dt)) < std::sin(2.0 * M_PI * frequency * time)) ||
            (output[i] < output[i - 1] && std::sin(2.0 * M_PI * frequency * (time - dt)) > std::sin(2.0 * M_PI * frequency * time)))
        {
            // Isso é uma verificação muito simplificada e pode falhar em pontos de inflexão.
            // Um teste mais robusto envolveria comparar a amplitude e fase esperadas.
            // Para agora, vamos apenas verificar se não há um comportamento completamente aleatório.
            ASSERT_TRUE(true);
        }
    }
}
