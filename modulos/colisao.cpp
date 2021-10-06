/////////////////// Funções de Colisão no Campo  ////////////


bool cLateralSuperior(double robot_y)
{
    bool lateral_superior = false;

    if (robot_y >= 0.609) //Lateral superior do campo
    {
        lateral_superior = true;
    }

    return lateral_superior;
}
bool cLateralInferior(double robot_y)
{
    bool lateral_inferior = false;

    if (robot_y <= -0.609) //Lateral inferior do campo
    {
        lateral_inferior = true;
    }

    return lateral_inferior;
}

bool cLateralDireita(double robot_x)
{
    bool lateral_direita = false;

    if (robot_x >= 0.71) //Lateral direita do campo
    {
        lateral_direita = true;
    }

    return lateral_direita;
}
bool cLateralEsquerda(double robot_x)
{
    bool lateral_esquerda = false;

    if (robot_x <= -0.71) //Lateral esquerda do campo
    {
        lateral_esquerda = true;
    }

    return lateral_esquerda;
}


bool cSuperiorDireito(double robot_x, double robot_y)
{
    bool superior_direito = false;

    if (robot_x >= 0.71 && robot_y >= 0.609) //Canto superior direito do campo
    {
        superior_direito = true;
    }

    return superior_direito;
}
bool cSuperiorEsquerdo(double robot_x, double robot_y)
{
    bool superior_esquerdo = false;

    if (robot_x <= -0.71 && robot_y >= 0.609) //Canto superior esquerdo do campo
    {
        superior_esquerdo = true;
    }

    return superior_esquerdo;
}
bool cInferiorDireito(double robot_x, double robot_y)
{
    bool inferior_direito = false;

    if (robot_x >= 0.71 && robot_y <= -0.609) //Canto inferior direito do campo
    {
        inferior_direito = true;
    }

    return inferior_direito;
}
bool cInferiorEsquerdo(double robot_x, double robot_y)
{
    bool inferior_esquerdo = false;

    if (robot_x <= -0.71 && robot_y <= -0.609) //Canto inferior esquerdo do campo
    {
        inferior_esquerdo = true;
    }

    return inferior_esquerdo;
}


bool cGolDireito(double robot_x)
{
    bool gol_direito = false;

    if (robot_x >= 0.81) //Gol direito 
    {
        gol_direito = true;
    }

    return gol_direito;
}
bool cGolEsquerdo(double robot_x)
{
    bool gol_esquerdo = false;

    if (robot_x <= -0.81) //Gol esquerdo 
    {
        gol_esquerdo = true;
    }

    return gol_esquerdo;
}
