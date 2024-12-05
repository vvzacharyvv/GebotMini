#pragma once
#ifndef head_H
#define head_H

//#define VMCCONTROL
#define ForceLPF  0.3
/* Admittance control */
// #define StepHeight_F  17.0*0.001  //swingUp
// #define StepHeight_H  16.0*0.001  //swingUp
// #define Press  16.0*0.001       //attach press 
// #define CompDisA1 -4*0.001     // Compensation of Distance in AttitudeCorrection() with Amble gait
// #define CompDisA2 26*0.001  
// #define CompDisA3 30*0.001  
// #define CompDisALL 16*0.001     // Compensation of Distance  AttitudeCorrection() All stace phase
/* Position control */
#define StepHeight_F  25.0/1000   //swingUp
#define StepHeight_H  25.0/1000   //swingUp
#define Press  26.0/1000.0  //19.0/1000        //attach press 7
#define CompDisA1 -8.0/1000.0   // Compensation of Distance in AttitudeCorrection() with Amble gait
#define CompDisA2 18.0/1000.0 
#define CompDisA3 18.0/1000.0   
#define CompDisALL 8.0 /1000.0     // Compensation of Distance  AttitudeCorrection() All stace phase
#define OFFSET {-0.1,0.0,-0,-0.1}
#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1
//  1:  Motor angle
//  2:  Foot end position
#define INIMODE 2
#define MORTOR_ANGLE_AMP 40*3.14/180.0
#define loopRateCommandUpdate 100.0   //hz
#define loopRateStateUpdateSend 20.0   //hz
#define loopRateImpCtller 100.0   //hz
#define loopRateDataSave 100.0 //hz
#define loopRateSVRead   100.0//hz
#define VELX 3.0/1000    // mm  step length = VELX * timeForStancePhase        
#define TimePeriod 0.05
#define TimeForGaitPeriod 8.0
#define PI 3.1415926
#define THREHOLDLF 1000//800
#define THREHOLDRF 1000//800
#define THREHOLDLH 1000//800
#define THREHOLDRH 1000//800
#define ATTACHDIS_MAX 20.0/1000
#define ATTACH_TIMES  4
#define PrePsotiveFactor 0.04
#define OMEGA PI
#define Y0 15.0/1000



#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <time.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <map>
#include <string>
#include <string.h>
#include "i2c.h"
#include "dynamixel.h"
#include "api.h"
#include <stdio.h>
#include <wiringPi.h> 
#include<fstream>
#include<sstream>
#include<bitset>
#include <mutex>
#include <condition_variable>
#include"vibration.h"
#include <qpOASES.hpp>
#ifdef  VMCCONTROL
  
#endif
using namespace Eigen;
using namespace std;
#endif
