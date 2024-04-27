#pragma once
#ifndef CONTROLHELPER_H
#define CONTROLHELPER_H
#include "head.h"
#include "udpsocket.h"
/***********motor control helper*************/
enum enum_LEGSTATUS{detach=0, swingUp, swingDown, attach, recover, stance}; // Status belongs to phase. Swing phase: detach, swingUp, swingDown, attach; || Stance phase: recover, stance
enum enum_LEGNAME{LF,RF,LH,RH};
enum enum_CONTROLMODE{ADMITTANCE,IMPEDANCE};
void string2float(std::string add, float* dest);
void printSvStatus(unsigned char svStatus);
vector<float> motorMapping(Matrix<float,4,3> jointCmdPos);
Matrix<float,4,3> inverseMotorMapping(vector<float> motorPos);

/*************command match help**************/
int match(char *P,char *T);
int *buildNext(char *P);
bool commandJudge(char *P,char* T);
#endif