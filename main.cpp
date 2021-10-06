#include <QCoreApplication>

#include <thread>
#include <utils/timer/timer.h>
#include <clients/vision/visionclient.h>
#include <clients/referee/refereeclient.h>
#include <clients/actuator/actuatorclient.h>
#include <clients/replacer/replacerclient.h>

#include <math.h>
#include <iomanip>

#include "modulos/localizacao.cpp"
#include "modulos/colisao.cpp"
#include "modulos/movimento.cpp"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    // Starting timer
    Timer timer;

    // Creating client pointers
    VisionClient *visionClient = new VisionClient("224.0.0.1", 10002);
    RefereeClient *refereeClient = new RefereeClient("224.5.23.2", 10003);
    ReplacerClient *replacerClient = new ReplacerClient("224.5.23.2", 10004);
    ActuatorClient *actuatorClient = new ActuatorClient("127.0.0.1", 20011);

    // Setting our color as BLUE at left side
    VSSRef::Color ourColor = VSSRef::Color::YELLOW;
    bool ourSideIsLeft = false;
    bool game_on = false;

    // Desired frequency (in hz)
    float freq = 60.0;

    // Setting actuator and replacer teamColor
    actuatorClient->setTeamColor(ourColor);
    replacerClient->setTeamColor(ourColor);

    while (1)
    {
        // Start timer
        timer.start();

        // Running vision and referee clients
        visionClient->run();
        refereeClient->run();

        // Debugging vision
        fira_message::sim_to_ref::Environment lastEnv = visionClient->getLastEnvironment();
        if (lastEnv.has_frame())
        {
            // Taking last frame
            fira_message::Frame lastFrame = lastEnv.frame();

            // Debugging ball
            std::cout << "\n===== BALL =====\n";
            QString ballDebugStr = QString("Ball x: %1 y: %2")
                                       .arg(lastFrame.ball().x())
                                       .arg(lastFrame.ball().y());
            std::cout << ballDebugStr.toStdString() + '\n';

            // Debugging blue robots
            std::cout << "\n===== BLUE TEAM =====\n";
            for (int i = 0; i < lastFrame.robots_blue_size(); i++)
            {
                QString robotDebugStr = QString("Robot %1 -> x: %2 y: %3 ori: %4")
                                            .arg(lastFrame.robots_blue(i).robot_id())
                                            .arg(lastFrame.robots_blue(i).x())
                                            .arg(lastFrame.robots_blue(i).y())
                                            .arg(lastFrame.robots_blue(i).orientation());
                std::cout << robotDebugStr.toStdString() + '\n';
            }

            // Debugging yellow robots
            std::cout << "\n===== YELLOW TEAM =====\n";
            for (int i = 0; i < lastFrame.robots_yellow_size(); i++)
            {
                QString robotDebugStr = QString("Robot %1 -> x: %2 y: %3 ori: %4")
                                            .arg(lastFrame.robots_yellow(i).robot_id())
                                            .arg(lastFrame.robots_yellow(i).x())
                                            .arg(lastFrame.robots_yellow(i).y())
                                            .arg(lastFrame.robots_yellow(i).orientation());
                std::cout << robotDebugStr.toStdString() + '\n';
            }
        }

        fira_message::Frame lastFrame = lastEnv.frame();

        if (refereeClient->getLastFoul() == VSSRef::Foul::GAME_ON)
        {
            game_on = true;
        }
        else
        {
            game_on = false;
        }

        // Sending robot commands for robot 0, 1 and 2
        if (game_on)
        {

            double robot0_x = lastFrame.robots_yellow(0).x(); // Coordenada em x do robo 0
            double robot0_y = lastFrame.robots_yellow(0).y(); // Coordenada  em y do robo 0

            double robot1_x = lastFrame.robots_yellow(1).x(); // Coordenada em x do robo 1
            double robot1_y = lastFrame.robots_yellow(1).y(); // Coordenada  em y do robo 1

            double robot2_x = lastFrame.robots_yellow(2).x(); // Coordenada em x do robo 2
            double robot2_y = lastFrame.robots_yellow(2).y(); // Coordenada  em y do robo 2

            double robot0_angle = lastFrame.robots_yellow(0).orientation(); // Orientação do robo 0
            double robot1_angle = lastFrame.robots_yellow(1).orientation(); // Orientação do robo 1
            double robot2_angle = lastFrame.robots_yellow(2).orientation(); // Orientação do robo 2

            double target_x = lastFrame.ball().x(); // Coordenada em x da bola
            double target_y = lastFrame.ball().y(); // Coordenada  em y da bola

            bool beGolDireito = areaGolDireito(robot0_x, robot0_y);

            int robot_index = 1;
            int velocity = 10;

            // printf("Está no gol direito: %d\n", beGolDireito);

            /// Função de Posicionar o Robô

            // Robô 1
            double angle_target, angle_target2; // Angulo que devo ir

            double distance1_x = robot1_x - target_x; // Distancia em x até a bola
            double distance1_y = robot1_y - target_y; // Distancia em y até a bola

            double route1 = sqrt(pow(distance1_x, 2) + pow(distance1_y, 2)); // distancia efetiva até a bola

            double cos_x = distance1_x / route1; //Obtendo COSSENO
            double sen_x = distance1_y / route1; //Obtendo seno

            double route_angle_y = asin(sen_x); // Obtendo angulo
            double route_angle_x = acos(cos_x); // Obtendo angulo

            // Robô 2
            double distance2_x = robot2_x - target_x; // Distancia em x até a bola
            double distance2_y = robot2_y - target_y; // Distancia em y até a bola

            double route2 = sqrt(pow(distance2_x, 2) + pow(distance2_y, 2)); // distancia efetiva até a bola

            double cos2_x = distance2_x / route2; //Obtendo COSSENO
            double sen2_x = distance2_y / route2; //Obtendo seno

            double route2_angle_y = asin(sen2_x); // Obtendo angulo
            double route2_angle_x = acos(cos2_x); // Obtendo angulo

            double distance_route = route1 - route2;

            printf("DISTANCIA ROTA: %9.5f \n", distance_route);

            // if (route2 > route1)
            // {
            //     printf("ROBÔ 1 \n");
            //     pegarBola(1, robot1_x, robot1_y, robot1_angle, target_x, target_y, 20);
            // }
            // else
            // {
            //     printf("ROBÔ 2 \n");
            //     pegarBola(2, robot2_x, robot2_y, robot2_angle, target_x, target_y, 20);
            // }

            if (route2 > route1)
            {
                printf("ROBÔ 1 \n");

                // Robô 1
                if (distance1_y < -0.03)
                { // Se a bola estiver acima do robo
                    if (distance1_x < 0)
                    { // Se a bola estiver na direita do robo
                        angle_target = -route_angle_y;
                    }
                    else
                    { // Se a bola estiver na esquerda do robo
                        printf("Angulo SENO: %9.2f \n", route_angle_y);
                        angle_target = 3.14 + route_angle_y;
                    }
                }

                if (distance1_y > 0.03)
                { // Se a bola estiver em baixo do robo//mudar intervalo
                    if (distance1_x < 0)
                    { // Se a bola estiver na direita do robO
                        angle_target = -route_angle_y;
                    }
                    else
                    { // Se a bola estiver na esquerda do robo
                        angle_target = -3.14 + route_angle_y;
                    }
                }

                //Se o robo estiver na mesma linha da bola
                if (distance1_y >= -0.01 && distance1_y <= 0.01)
                {
                    printf("NA LINHA ");
                    if (distance1_x < 0)
                    { // Se a bola estiver na direita do robo
                        printf("NA DIREITA \n");
                        angle_target = 0.2;
                    }
                    else
                    { // Se a bola estiver na esquerda do robo
                        angle_target = 3.12;
                        printf("NA ESQUERDA \n");
                    }
                }

                double robot_angle = lastFrame.robots_yellow(robot_index).orientation();

                //////////// Função de Girar o Robô ////////
                double convert_angle, convert_robot_angle, caminho;

                printf("DISTANCIA Y: %9.5f \n", distance1_y);
                printf("DISTANCIA X: %9.5f \n", distance1_x);
                printf("ANGULO ALVO: %9.5f \n", angle_target);
                printf("ANGULO ROBO: %9.5f \n", robot_angle);

                // Convertendo angulos para positivo

                //Calculando distância do angulo horário

                if (angle_target > 0.2 && angle_target < 3.12)
                { // se o angulo for para cima
                    printf("ANGULO CIMA \n");

                    if (!(robot_angle > angle_target - 0.2 && robot_angle < angle_target + 0.2))
                    { //se estiver fora do angulo
                        caminho = robot_angle - angle_target;
                        if (caminho > 0)
                        {
                            //horario = positivo
                            actuatorClient->sendCommand(robot_index, 4, -4);
                            printf("HORÁRIO \n");
                        }
                        else
                        {
                            //anti
                            actuatorClient->sendCommand(robot_index, -4, 4);
                            printf("ANTIHORÁRIO \n");
                        }

                        robot_angle = lastFrame.robots_yellow(robot_index).orientation();
                    }
                    else
                    {
                        actuatorClient->sendCommand(robot_index, velocity, velocity);
                    }
                }
                else if (angle_target < -0.2 && angle_target > -3.12)
                { // se o angulo for para baixo
                    printf("ANGULO BAIXO \n");

                    if (!(robot_angle < angle_target + 0.2 && robot_angle > angle_target - 0.2))
                    { //se estiver fora do angulo
                        caminho = robot_angle - angle_target;
                        if (caminho > 0)
                        {
                            //horario = positivo
                            actuatorClient->sendCommand(robot_index, 4, -4);
                            printf("HORÁRIO \n");
                        }
                        else
                        {
                            //anti
                            actuatorClient->sendCommand(robot_index, -4, 4);
                            printf("ANTIHORÁRIO \n");
                        }
                        robot_angle = lastFrame.robots_yellow(robot_index).orientation();
                    }
                    else
                    {
                        actuatorClient->sendCommand(robot_index, velocity, velocity);
                    }
                }
                /////////////////////////////////////////////////////////////////
                else
                { // se estiver na linha horizontal

                    if ((robot_angle < angle_target * 0.90))
                    { //gira se n tiver na horizontal

                        caminho = robot_angle - angle_target;
                        if (caminho > 0)
                        {
                            //anti
                            actuatorClient->sendCommand(robot_index, 4, -4);
                        }
                        else
                        {
                            //horario = positivo
                            actuatorClient->sendCommand(robot_index, -4, 4);
                        }
                        robot_angle = lastFrame.robots_yellow(robot_index).orientation();
                    }
                    else
                    {
                        actuatorClient->sendCommand(robot_index, velocity, velocity);
                    }
                }
                printf("CAMINHO: %9.5f \n", caminho);
            }
            else
            {
                printf("ROBÔ 2 \n");

                // Robô 2
                robot_index = 2;

                if (distance2_y < -0.03)
                { // Se a bola estiver acima do robo
                    if (distance2_x < 0)
                    { // Se a bola estiver na direita do robo
                        angle_target2 = -route2_angle_y;
                    }
                    else
                    { // Se a bola estiver na esquerda do robo
                        printf("Angulo SENO: %9.2f \n", route2_angle_y);
                        angle_target2 = 3.14 + route2_angle_y;
                    }
                }

                if (distance2_y > 0.03)
                { // Se a bola estiver em baixo do robo//mudar intervalo
                    if (distance2_x < 0)
                    { // Se a bola estiver na direita do robO
                        angle_target2 = -route2_angle_y;
                    }
                    else
                    { // Se a bola estiver na esquerda do robo
                        angle_target2 = -3.14 + route2_angle_y;
                    }
                }

                //Se o robo estiver na mesma linha da bola
                if (distance2_y >= -0.01 && distance2_y <= 0.01)
                {
                    printf("NA LINHA ");
                    if (distance2_x < 0)
                    { // Se a bola estiver na direita do robo
                        printf("NA DIREITA \n");
                        angle_target2 = 0.2;
                    }
                    else
                    { // Se a bola estiver na esquerda do robo
                        angle_target2 = 3.12;
                        printf("NA ESQUERDA \n");
                    }
                }

                double robot2_angle = lastFrame.robots_yellow(robot_index).orientation();

                //////////// Função de Girar o Robô ////////
                double convert_angle, convert_robot_angle, caminho;

                printf("DISTANCIA Y: %9.5f \n", distance2_y);
                printf("DISTANCIA X: %9.5f \n", distance2_x);
                printf("ANGULO ALVO: %9.5f \n", angle_target2);
                printf("ANGULO ROBO: %9.5f \n", robot2_angle);

                // Convertendo angulos para positivo

                //Calculando distância do angulo horário

                if (angle_target2 > 0.2 && angle_target2 < 3.12)
                { // se o angulo for para cima
                    if (!(robot2_angle > angle_target2 - 0.2 && robot2_angle < angle_target2 + 0.2))
                    { //se estiver fora do angulo
                        caminho = robot2_angle - angle_target2;
                        if (caminho > 0)
                        {
                            //horario = positivo
                            actuatorClient->sendCommand(robot_index, 4, -4);
                            printf("HORÁRIO \n");
                        }
                        else
                        {
                            //anti
                            actuatorClient->sendCommand(robot_index, -4, 4);
                            printf("ANTIHORÁRIO \n");
                        }

                        robot2_angle = lastFrame.robots_yellow(robot_index).orientation();
                    }
                    else
                    {
                        actuatorClient->sendCommand(robot_index, velocity, velocity);
                    }
                }
                else if (angle_target2 < -0.2 && angle_target2 > -3.12)
                { // se o angulo for para baixo
                    if (!(robot2_angle < angle_target2 + 0.2 && robot2_angle > angle_target2 - 0.2))
                    { //se estiver fora do angulo
                        caminho = robot2_angle - angle_target2;
                        if (caminho > 0)
                        {
                            //anti
                            actuatorClient->sendCommand(robot_index, 4, -4);
                        }
                        else
                        {
                            //horario = positivo
                            actuatorClient->sendCommand(robot_index, -4, 4);
                        }
                        robot2_angle = lastFrame.robots_yellow(robot_index).orientation();
                    }
                    else
                    {
                        actuatorClient->sendCommand(robot_index, velocity, velocity);
                    }
                }
                /////////////////////////////////////////////////////////////////
                else
                { // se estiver na linha horizontal

                    if ((robot2_angle < angle_target2 * 0.90))
                    { //gira se n tiver na horizontal

                        caminho = robot2_angle - angle_target2;
                        if (caminho > 0)
                        {
                            //anti
                            actuatorClient->sendCommand(robot_index, 4, -4);
                        }
                        else
                        {
                            //horario = positivo
                            actuatorClient->sendCommand(robot_index, -4, 4);
                        }
                        robot2_angle = lastFrame.robots_yellow(robot_index).orientation();
                    }
                    else
                    {
                        actuatorClient->sendCommand(robot_index, velocity, velocity);
                    }
                }
            }
            /////////////////// Função de Colisão com Robôs do Time ////////////

            // double distance_robot1_x, distance_robot1_y;

            // distance_robot1_x = robot0_x - robot1_x; // Distancia em x até o robo 1
            // distance_robot1_y = robot0_y - robot1_y; // Distancia em y até o robo 1
            // // std::cout << std::fixed << std::showpoint;
            // // std::cout << std::setprecision(3);
            // // std::cout << distance_robot1_x << std::endl;

            // printf("DISTANCIA ROBO 1 X: %9.6f \n", distance_robot1_x);
            // printf("DISTANCIA ROBO 1 Y: %9.6f \n", distance_robot1_y);

            // if(robot0_y >= 0.58 * 0.98 && robot0_x <= 0.075) { //Lateral Q2
            // if( (distance_robot1_x >= 0.095 && distance_robot1_x <= 0.1) && (distance_robot1_y >= 0.025 && distance_robot1_y <= 0.03) ) {
            // if (distance_robot1_x <= 0.098 && distance_robot1_y <= 0.098)
            // {
            //     printf("OIII \n");
            //     actuatorClient->sendCommand(0, 0, 0);
            // }
            //}

            /////////////////// Função de Colisão com Lateral Superior e Inferior ////////////

            bool colidiu = cLateralSuperior(robot0_y);
            bool colidiuS = cSuperiorDireito(robot0_x, robot0_y);

            if (colidiuS)
            {
                printf("PAROU \n");
                actuatorClient->sendCommand(0, 0, 0);
            }
        }

        else
        {
            actuatorClient->sendCommand(0, 0, 0);
            actuatorClient->sendCommand(1, 0, 0);
            actuatorClient->sendCommand(2, 0, 0);
        }

        // If is kickoff, send this test frame!
        if (refereeClient->getLastFoul() == VSSRef::Foul::KICKOFF)
        {
            replacerClient->placeRobot(0, ourSideIsLeft ? -0.2 : 0.2, 0, 0);
            replacerClient->placeRobot(1, ourSideIsLeft ? -0.2 : 0.2, 0.2, 0);
            replacerClient->placeRobot(2, ourSideIsLeft ? -0.2 : 0.2, -0.2, 0);
            replacerClient->sendFrame();
        }

        // Stop timer
        timer.stop();

        // Sleep for remainingTime
        long remainingTime = (1000 / freq) - timer.getMiliSeconds();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    // Closing clients
    visionClient->close();
    refereeClient->close();
    replacerClient->close();
    actuatorClient->close();

    return a.exec();
}
