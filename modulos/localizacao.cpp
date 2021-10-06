/////////////////  Funções de Posição no Campo   ////////////


bool quadrante1(double robot_x, double robot_y)
{
    bool position = false;

    if(robot_x >= 0.04 && robot_y >= 0.026)
    {
        position = true;
    }

    return position ;
}
bool quadrante2(double robot_x, double robot_y)
{
    bool position = false;

    if(robot_x <= -0.04 && robot_y >= 0.026)
    {
        position = true;
    }

    return position ;
}
bool quadrante3(double robot_x, double robot_y)
{
    bool position = false;

    if(robot_x <= -0.04 && robot_y <= -0.026)
    {
        position = true;
    }

    return position ;
}
bool quadrante4(double robot_x, double robot_y)
{
    bool position = false;

    if(robot_x >= 0.04 && robot_y <= -0.026)
    {
        position = true;
    }

    return position ;
}

bool areaGolDireito(double robot_x, double robot_y)
{
    bool position = false;

    if(robot_x >= 0.637 && ((robot_y <= 0.305 && robot_y >= -0.305) || robot_y == 0.075) )
    {
        position = true;
    }

    return position ;
}
bool areaGolEsquerdo(double robot_x, double robot_y)
{
    bool position = false;

    if(robot_x <= -0.637 && ((robot_y <= 0.305 && robot_y >= -0.305) || robot_y == 0.075) )
    {
        position = true;
    }

    return position ;
}
bool linhaDireita(double robot_x, double robot_y)
{
    bool position = false;

    if(robot_x > 0 && (robot_y <= 0.025 && robot_y >= -0.025) )
    {
        position = true;
    }

    return position ;
}
bool linhaEsquerda(double robot_x, double robot_y)
{
    bool position = false;

    if(robot_x < 0 && (robot_y <= 0.025 && robot_y >= -0.025) )
    {
        position = true;
    }

    return position ;
}