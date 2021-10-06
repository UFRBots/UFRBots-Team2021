/////////////////  Funções de Movimento   ////////////

#include <clients/vision/visionclient.h>
#include <clients/actuator/actuatorclient.h>

#include <math.h>
#include <iomanip>

// void girarDireita90()
// {
// }

void pegarBola(int robot_index, double robot_x, double robot_y, double robot_orientation, double target_x, double target_y, int velocity)
{
    ActuatorClient *actuatorClient = new ActuatorClient("127.0.0.1", 20011);

    VSSRef::Color ourColor = VSSRef::Color::YELLOW;

    actuatorClient->setTeamColor(ourColor);

    double route, distance_y, distance_x, angle_target, convert_angle, convert_robot_angle, caminho;

    double robot_angle = robot_orientation;

    distance_x = robot_x - target_x; // Distancia em x até a bola
    distance_y = robot_y - target_y; // Distancia em y até a bola

    route = sqrt(pow(distance_x, 2) + pow(distance_y, 2)); // distancia efetiva até a bola

    double cos_x = distance_x / route; //Obtendo cosseno
    double sen_x = distance_y / route; //Obtendo seno

    double route_angle_y = asin(sen_x); // Obtendo angulo
    double route_angle_x = acos(cos_x); // Obtendo angulo

    //////////// Função de Posicionar o Robô ////////

    if (distance_y < -0.03)
    { // Se a bola estiver acima do robo
        if (distance_x < 0)
        { // Se a bola estiver na direita do robo
            angle_target = -route_angle_y;
        }
        else
        { // Se a bola estiver na esquerda do robo
            printf("Angulo SENO: %9.2f \n", route_angle_y);
            angle_target = 3.14 + route_angle_y;
        }
    }

    if (distance_y > 0.03)
    { // Se a bola estiver em baixo do robo//mudar intervalo
        if (distance_x < 0)
        { // Se a bola estiver na direita do robO
            angle_target = -route_angle_y;
        }
        else
        { // Se a bola estiver na esquerda do robo
            angle_target = -3.14 + route_angle_y;
        }
    }

    //Se o robo estiver na mesma linha da bola
    if (distance_y >= -0.01 && distance_y <= 0.01)
    {
        printf("NA LINHA ");
        if (distance_x < 0)
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

    //////////// Função de Girar o Robô ////////

    // Convertendo angulos para positivo

    // Calculando distância do angulo horário

    if (angle_target > 0.2 && angle_target < 3.12)
    { // se o angulo for para cima
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

            robot_angle = robot_orientation;
        }
        else
        {
            actuatorClient->sendCommand(robot_index, velocity, velocity);
        }
    }
    else if (angle_target < -0.2 && angle_target > -3.12)
    { // se o angulo for para baixo
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
            robot_angle = robot_orientation;
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
            robot_angle = robot_orientation;
        }
        else
        {
            actuatorClient->sendCommand(robot_index, velocity, velocity);
        }
    }
}