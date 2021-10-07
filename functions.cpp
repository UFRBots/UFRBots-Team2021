#include <stdio.h>
#include <iostream>
#include <QtNetwork>

enum 
{
    APROXIMA = 0,
    DECIDE_DESVIO,
    SOBE,
    DESCE,
    VOLTA
};

class Objective 
{
    double m_x;
    double m_y;
    double m_angle;

    public:
        Objective(double t_x, double t_y, double t_angle) : m_x(t_x), m_y(t_y), m_angle(t_angle){};

        void setY(double value)
        {
            m_y = value;
        }

        void setAngle(double value)
        {
            m_angle = value;
        }

        void setX(double value)
        {
            m_x = value;
        }
        double x()
        {
            return m_x;
        }
        double y()
        {
            return m_y;
        }
        double angle()
        {
            return m_angle;
        }
};

double to180range(double angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle < -M_PI)
    {
        angle = angle + 2 * M_PI;
    }
    else if (angle > M_PI)
    {
        angle = angle - 2 * M_PI;
    }
    return angle;
}

double smallestAngleDiff(double target, double source)
{
    double a;
    a = fmod(target + 2 * M_PI, 2 * M_PI) - fmod(source + 2 * M_PI, 2 * M_PI);

    if (a > M_PI)
    {
        a = a - 2 * M_PI;
    }
    else if (a < -M_PI)
    {
        a = a + 2 * M_PI;
    }
    return a;
}