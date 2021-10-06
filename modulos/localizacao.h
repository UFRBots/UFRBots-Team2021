#ifndef LOCALIZACAO_H
#define LOCALIZACAO_H


bool quadrante1(double robot_x, double robot_y);
bool quadrante2(double robot_x, double robot_y);
bool quadrante3(double robot_x, double robot_y);
bool quadrante4(double robot_x, double robot_y);

bool areaGolDireito(double robot_x, double robot_y);
bool areaGolEsquerdo(double robot_x, double robot_y);

bool linhaDireita(double robot_x, double robot_y);
bool linhaEsquerda(double robot_x, double robot_y);


#endif