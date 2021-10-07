#include <QCoreApplication>

#include <thread>
#include <utils/timer/timer.h>
#include <clients/vision/visionclient.h>
#include <clients/referee/refereeclient.h>
#include <clients/actuator/actuatorclient.h>
#include <clients/replacer/replacerclient.h>

#include "functions.cpp"


void PID(fira_message::Robot robot, Objective objective, int index, ActuatorClient &actuatorClient, bool Team_UFRBots, double acrescimo)
{
    double Kp = 20;
    double Kd = 2.5;
    static double lastError = 0;

    double rightMotorSpeed;
    double leftMotorSpeed;

    bool reversed = false;

    double angle_rob = robot.orientation();

    double angle_obj = atan2(objective.y() - robot.y(), objective.x() - robot.x());

    double error = smallestAngleDiff(angle_rob, angle_obj);

    if (fabs(error) > M_PI / 2.0 + M_PI / 20.0)
    {
        reversed = true;
        angle_rob = to180range(angle_rob + M_PI);
        // Calculates the error and reverses the front of the robot
        error = smallestAngleDiff(angle_rob, angle_obj);
    }

    double motorSpeed = (Kp * error) + (Kd * (error - lastError)); // + 0.2 * sumErr;
    lastError = error;

    double baseSpeed = (30 + acrescimo);

    // Normalize
    motorSpeed = motorSpeed > (30 + acrescimo) ? (30 + acrescimo) : motorSpeed;
    motorSpeed = motorSpeed < (-30 - acrescimo) ? (-30 - acrescimo)  : motorSpeed;

    if (motorSpeed > 0)
    {
        leftMotorSpeed = baseSpeed;
        rightMotorSpeed = baseSpeed - motorSpeed;
    }
    else
    {
        leftMotorSpeed = baseSpeed + motorSpeed;
        rightMotorSpeed = baseSpeed;
    }

    if (reversed)
    {
        if (motorSpeed > 0)
        {
            leftMotorSpeed = -baseSpeed + motorSpeed;
            rightMotorSpeed = -baseSpeed;
        }
        else
        {
            leftMotorSpeed = -baseSpeed;
            rightMotorSpeed = -baseSpeed - motorSpeed;
        }
    }
    //actuatorClient.sendCommand(leftMotorSpeed, rightMotorSpeed, Team_UFRBots, index);
    actuatorClient->sendCommand(index, leftMotorSpeed, rightMotorSpeed);
}

int estado = APROXIMA;
int estadoant;
double x, y, ang;
double x_in, y_in;

Objective defineObjective(fira_message::Robot robot, fira_message::Ball ball)
{
    double distancia = sqrt(pow(robot.x() - ball.x(), 2) + pow(robot.y() - ball.y(), 2));

    if (robot.x() < ball.x())
    { //se o robô está atrás da bola
        estado = APROXIMA;

        if ((robot.y() < ball.y() - 5 || robot.y() > ball.y() + 5) && robot.x() <= 22)
            //alinhando o robô com a bola!
            return Objective(ball.x() - 10, ball.y(), M_PI / 4.); // x,y,angle

        else if (distancia < 8 && 49 < robot.y() && 83 > robot.y())
        {
            //Se o robô 'está' com a bola e está indo em direção ao gol
            return Objective(ball.x(), ball.y(), M_PI / 4.);
        }
        else if (distancia < 8 && (43 > robot.y() || 83 < robot.y()))
        {
            //Se o robô 'está' com a bola e NÃO está indo em direção ao gol
            return Objective(ball.x(), 66, M_PI / 4.);
        }
        else
            return Objective(ball.x(), ball.y(), M_PI / 4.);
        //no mais, corre atrás da bola...
    }
    else
    {
        switch (estado)
        {
        
        case APROXIMA:
            //corre atras da bola ate chegar perto
            if (distancia < 10)
                estado = DECIDE_DESVIO;
            x = ball.x() + 10;
            y = ball.y();
            ang = 0; // x,y,angle
            break;

        case DECIDE_DESVIO:
            //desvia da bola, para voltar a ficar atrás dela
            if (robot.y() - 10 > 6)
                estado = SOBE;
            else
                estado = DESCE;

            break;

        case SOBE:
            estadoant = SOBE;
            estado = VOLTA;
            x = ball.x() + 10;
            y = ball.y() - 8;
            x_in = robot.x();
            y_in = robot.y();
            ang = M_PI / 2.0; // x,y,angle

            break;
        
        case DESCE:
            estadoant = DESCE;
            estado = VOLTA;
            x = ball.x() + 10;
            y = ball.y() + 8;
            x_in = robot.x();
            y_in = robot.y();
            ang = M_PI / 2.0; // x,y,angle

            break;
        
        case VOLTA:
            if (robot.x() <= x_in - 16 || robot.x() <= 12)
            {
                //se ele andou 16 ou tá no limite do mapa
                //fazendo_manobra = 0;
                estado = APROXIMA;
            }
            if (estadoant == DESCE)
            {
                x = ball.x() - 16;
                y = ball.y() + 8;
                ang = M_PI; // x,y,angle */
            }
            else
            {
                x = ball.x() - 16;
                y = ball.y() - 8;
                ang = M_PI; // x,y,angle */
            }
            break;
        }

        return Objective(x, y, ang);
    }
}



int main(int argc, char *argv[]) {
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
    bool penalty = false;

    // Desired frequency (in hz)
    float freq = 60.0;

    // Setting actuator and replacer teamColor
    actuatorClient->setTeamColor(ourColor);
    replacerClient->setTeamColor(ourColor);

    double width, length;
    width = 1.3 / 2.0;
    length = 1.7 / 2.0;

    while(1) {
        // Start timer
        timer.start();

        // Running vision and referee clients
        visionClient->run();
        refereeClient->run();

        // Debugging vision
        fira_message::sim_to_ref::Environment lastEnv = visionClient->getLastEnvironment();
        if(lastEnv.has_frame()) {
            // Taking last frame
            fira_message::Frame lastFrame = lastEnv.frame();

            // Informacoes da bola
            fira_message::Ball ball = detection.ball();
            ball.set_x((length + ball.x()) * 100);
            ball.set_y((width + ball.y()) * 100);

            if (ourColor == VSSRef::Color::BLUE)
            {
                for (int i = 0; i < 3; i++)
                {
                    fira_message::Robot robot = detection.robots_blue(i); // Detecção do robo
                    robot.set_x((length + robot.x()) * 100); // Convertendo para centimetros
                    robot.set_y((width + robot.y()) * 100);
                    robot.set_orientation(to180range(robot.orientation()));
                }

            }
            else
            {
                for (int i = 0; i < 3; i++)
                {
                    fira_message::Robot robot = detection.robots_yellow(i); // Detecção do robo
                    robot.set_x((length + robot.x()) * 100); // Convertendo para centimetros
                    robot.set_y((width + robot.y()) * 100);
                    robot.set_orientation(to180range(robot.orientation()));
                }
            }
            
            
            

        //     // Debugging ball
        //     std::cout << "\n===== BALL =====\n";
        //     QString ballDebugStr = QString("Ball x: %1 y: %2")
        //                         .arg(lastFrame.ball().x())
        //                         .arg(lastFrame.ball().y());
        //     std::cout << ballDebugStr.toStdString() + '\n';

        //     // Debugging blue robots
        //     std::cout << "\n===== BLUE TEAM =====\n";
        //     for(int i = 0; i < lastFrame.robots_blue_size(); i++) {
        //         QString robotDebugStr = QString("Robot %1 -> x: %2 y: %3 ori: %4")
        //                 .arg(lastFrame.robots_blue(i).robot_id())
        //                 .arg(lastFrame.robots_blue(i).x())
        //                 .arg(lastFrame.robots_blue(i).y())
        //                 .arg(lastFrame.robots_blue(i).orientation());
        //         std::cout << robotDebugStr.toStdString() + '\n';
        //     }

        //     // Debugging yellow robots
        //     std::cout << "\n===== YELLOW TEAM =====\n";
        //     for(int i = 0; i < lastFrame.robots_yellow_size(); i++) {
        //         QString robotDebugStr = QString("Robot %1 -> x: %2 y: %3 ori: %4")
        //                 .arg(lastFrame.robots_yellow(i).robot_id())
        //                 .arg(lastFrame.robots_yellow(i).x())
        //                 .arg(lastFrame.robots_yellow(i).y())
        //                 .arg(lastFrame.robots_yellow(i).orientation());
        //         std::cout << robotDebugStr.toStdString() + '\n';
        //     }
        }

        VSSRef::Quadrant quadrante = refereeClient->getLastFoulQuadrant();
        VSSRef::Foul tipo_falta = refereeClient->getLastFoul();
        VSSRef::Color time_falta = refereeClient->getLastFoulColor();

        // Posicionando robôs

        // If is kickoff, send this test frame!
        if(tipo_falta == VSSRef::Foul::KICKOFF) {
            replacerClient->placeRobot(0, ourSideIsLeft ? -0.675 : 0.675, 0, 0);
            replacerClient->placeRobot(1, ourSideIsLeft ? -0.2 : 0.2, 0.03, 0);
            replacerClient->placeRobot(2, ourSideIsLeft ? -0.375 : 0.375, -0.03, 0);
            replacerClient->sendFrame();
        }


        if (quadrante == VSSRef::Quadrant::QUADRANT_1)
        {
            replacerClient->placeRobot(0, ourSideIsLeft ? -0.675 : 0.675, 0.15, 0);
            replacerClient->placeRobot(1, ourSideIsLeft ? 0.175 : 0.575 , 0.4, 0);
            replacerClient->placeRobot(2, ourSideIsLeft ? 0.1 : 0.3, -0.1, 0);
            replacerClient->sendFrame();
        }

        if (quadrante == VSSRef::Quadrant::QUADRANT_2)
        {
            replacerClient->placeRobot(0, ourSideIsLeft ? -0.675 : 0.675, 0.15, 0);
            replacerClient->placeRobot(1, ourSideIsLeft ? -0.575 : -0.175, 0.4, 0);
            replacerClient->placeRobot(2, ourSideIsLeft ? -0.3 : -0.1, -0.1, 0);
            replacerClient->sendFrame();
        }
        
        if (quadrante == VSSRef::Quadrant::QUADRANT_3)
        {
            replacerClient->placeRobot(0, ourSideIsLeft ? -0.675 : 0.675, -0.15, 0);
            replacerClient->placeRobot(1, ourSideIsLeft ? -0.3 : -0.1, 0.1, 0);
            replacerClient->placeRobot(2, ourSideIsLeft ? -0.575 : -0.175, -0.4, 0);
            replacerClient->sendFrame();
        }

         if (quadrante == VSSRef::Quadrant::QUADRANT_4)
        {
            replacerClient->placeRobot(0, ourSideIsLeft ? -0.675 : 0.675, -0.15, 0);
            replacerClient->placeRobot(1, ourSideIsLeft ? 0.1 : 0.3, 0.1, 0);
            replacerClient->placeRobot(2, ourSideIsLeft ? 0.175 : 0.575, -0.4, 0);
            replacerClient->sendFrame();
        }
    
        //Fim de posicionamento



        // Ação dos robôs

        if (refereeClient->getLastFoul() == VSSRef::Foul::GAME_ON)
        {
            game_on = true;
        }
        else{
            game_on = false;
        }
        
        // Sending robot commands for robot 0, 1 and 2
        // if(game_on) {
        //     actuatorClient->sendCommand(0, -10, -10);
        //     actuatorClient->sendCommand(1, 0, 0);
        //     actuatorClient->sendCommand(2, 0, 0);
        // }
        // else{
        //     actuatorClient->sendCommand(0, 0, 0);
        //     actuatorClient->sendCommand(1, 0, 0);
        //     actuatorClient->sendCommand(2, 0, 0);
        // }



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
