#pragma once
#ifndef RBOBOTCONTROL_H
#define RBOBOTCONTROL_H
#include "gebot.h"

class CRobotControl : public CGebot{
public:
    CRobotControl(float length,float width,float height,float mass,enum_CONTROLMODE mode):CGebot(length,width,height,mass){
            m_eControlMode=mode;
            Init();
    }
//     CRobotControl(enum_CONTROLMODE mode):CGebot(){
//         m_eControlMode=mode;
//         Init();
// }
    Matrix<float, 4, 3> mfTargetPos; // LF RF LH RH ; x y z  in CoM cordinate 
    Matrix<float, 4, 3> mfTargetVel;
    Matrix<float, 4, 3> mfTargetAcc; // Force in target position
    Matrix<float, 4, 3> mfTargetForce;
    Matrix<float, 4, 3> mfXcDotDot;
    Matrix<float, 4, 3> mfXcDot;
    Matrix<float, 4, 3> mfXc;
    Matrix<float, 3, 4> mfForce, mfLastForce;              // force feedback   x y z ; LF RF LH RH
    Matrix<float, 4, 4> mfTargetTor;
    Matrix<float, 4, 3> mfKswing, mfKstance, mfKdetach, mfKattach;                     //LF RF LH RH
    Matrix<float, 4, 3> mfBwing, mfBstance, mfBdetach, mfBattach;
    Matrix<float, 4, 3> mfMswing, mfMstance, mfMdetach, mfMattach;
    float fCtlRate;
    enum_CONTROLMODE m_eControlMode;  
    void Init();
    void UpdateFtsPresForce(vector<float> present_torque);
    void UpdateTargTor(Matrix<float, 3, 4> force);
    void ParaDeliver();
    void Control();
    void ChangePara(Matrix<float, 4, 3> mK, Matrix<float, 4, 3> mB, Matrix<float, 4, 3> mM,int mode);
   
    //vmc
     // ASM: 0, -c, b, c, 0, -a, -b, a, 0
    vector<Matrix<float,3,3> >  mfFtsPosASM;
    Matrix<float,Dynamic,Dynamic> mfVmcA; //6*3c
    Matrix<float,Dynamic,Dynamic> mfVmcW; //3c*3c
    Matrix<float,Dynamic,Dynamic> mfVmcH; //3c*3c
    Matrix<float,3,1> vfGravity;
    Matrix<float,6,1> vfVmcb61;
    Matrix<float,6,6> mfVmcS66;
    Matrix<float,Dynamic,1> vfVmcg3c1;
    Matrix<float,3,3> mfVmcKpcom;
    Matrix<float,3,3> mfVmcKdcom;
    Matrix<float,3,3> mfVmcKpbase;
    Matrix<float,3,3> mfVmcKdbase;
    Matrix<float,3,1> vfVmcAccDCom;
    Matrix<float,3,1> vfVmcAngelAccDBase;
    Matrix<float,3,1> vfVmcOmegaDBase;
    Matrix<float,3,1> vfVmcOmegaBase;
    void CalVmcCom();
    void UpdateImuData();


};
#endif