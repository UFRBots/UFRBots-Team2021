#ifndef COLISAO_H
#define COLISAO_H


bool cLateralSuperior(double robot_y);
bool cLateralInferior(double robot_y);
bool cLateralDireita(double robot_x);
bool cLateralEsquerda(double robot_x);

bool cSuperiorDireito(double robot_x, double robot_y);
bool cSuperiorEsquerdo(double robot_x, double robot_y);
bool cInferiorDireito(double robot_x, double robot_y);
bool cInferiorEsquerdo(double robot_x, double robot_y);

bool cGolDireito(double robot_x);
bool cGolEsquerdo(double robot_x);


#endif