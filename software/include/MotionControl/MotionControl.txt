#ifndef robotMotionControl_H
#define robotMotionControl_H
#include "leg.h"
using namespace Eigen;
using namespace std;
class MotionControl{
public:
        float timeForGaitPeriod;  // The time of the whole period
        float timePeriod;  // The time of one period
        float timePresent;  
        Vector<float, 4> timeForSwing;   // The swing time for legs
        Matrix<float, 4, 2> timeForStancePhase;  // startTime, endTime: LF, RF, LH, RH
        Vector<float, 6> targetCoMVelocity;  // X, Y , Z ,yaw in world cordinate
        Vector<float, 6> presentCoMVelocity;  // X, Y , Z ,yaw in world cordinate
        Matrix<float, 4, 3> targetCoMPosition;  // X, Y , alpha in world cordinate
        Matrix<float, 4, 3> targetCoMPosture;
        float yawVelocity;   // yaw velocity from imu
        enum _stepFlag{stance=0, swing, detach, attach}stepFlag;  //  0-stance, 1-swing, 2-detach, 3-attach: LF, RF, LH, RH>
        Vector<float, 4> timePresentForSwing;
        float L1, L2, L3;  // The length of L
        float width, length,height,mass;
        Matrix<float, 4, 2> shoulderPos;  // X-Y: LF, RF, LH, RH
        Matrix<float, 4, 3> stancePhaseStartPos;
        Matrix<float, 4, 3> stancePhaseEndPos;
        Matrix<float, 4, 3> initFootPos;    // initial position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
        Matrix<float, 4, 3> legCmdPos;  // command position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
        Matrix<float, 4, 3> legPresPos;  // present position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
        Matrix<float, 4, 3> legPos_last;
        Matrix<float, 4, 3> legPresVel;  // present velocity of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
        Matrix<float, 4, 3> legLastVel;   //last velocity of foot to shoulder,for filter
        Matrix<float, 4, 3> jointCmdPos;  // command joint angle 0-11
        Matrix<float, 4, 3>  jointPresPos;  // present motor 0-11
        Matrix<float, 4, 3>  jointPresVel;  // present motor 0-11
        Matrix<float, 4, 3>  jointCmdVel; 
        std::vector<Matrix<float, 3, 3>> jacobian_vector; 
        float motorTorque[1];
        float motorInitPos[12];   // init motor angle of quadruped state
        float pid_motortorque[12];
        float jacobian_motortorque[12]; //motor torque in VMC
        float motorCmdTorque[12];
        bool initFlag;

        void setInitPos(Matrix<float, 4, 3> initPosition);
        void setCoMVel(Vector<float, 6> tCV);   
        void setPhase(float tP, float tFGP, Matrix<float, 4, 2> tFSP);
        void nextStep();
        void updatejointPresPos(vector<float> jointPos);
        void updatejointPresVel(vector<float> jointVel);
        void updateJacobians();
        void forwardKinematics(int mode);
        void inverseKinematics(Matrix<float, 4, 3> cmdpos);   // standing state
        void updateFtsPresVel(); 
        //robot control
        uint8_t svStatus=0b00000000;
        API api;
        void air_control();
        void pumpNegtive(int legNum);
        void pumpPositive(int legNum);
        void pumpAllNegtive();
        void pumpAllClose();
        MotionControl();




};

class IMPControl : public MotionControl
{
    public:
        Matrix<float, 4, 3> target_pos; // LF RF LH RH ; x y z  in CoM cordinate 
        Matrix<float, 4, 3> target_vel;
        Matrix<float, 4, 3> target_acc; // Force in target position
        Matrix<float, 4, 3> target_force;
        // Vector<float, 3> targetCoMVelocity;  // X, Y , alpha in world cordinate
        // Vector<float, 3> presentCoMVelocity;  // X, Y , alpha in world cordinate
        // Matrix<float, 4, 3> targetCoMPosition;  // X, Y , alpha in world cordinate
        Matrix<float, 4, 3> xc_dotdot;
        Matrix<float, 4, 3> xc_dot;
        Matrix<float, 4, 3> xc;
        Matrix<float, 3, 4> force, force_last;              // force feedback   x y z ; LF RF LH RH
        Matrix<float, 3, 4> target_torque;
        Matrix<float, 4, 3> K_swing, K_stance, K_detach, K_attach;                     //LF RF LH RH
        Matrix<float, 4, 3> B_swing, B_stance, B_detach, B_attach;
        Matrix<float, 4, 3> M_swing, M_stance, M_detach, M_attach;
        float impCtlRate;
                //vmc
        float Ixx,Iyy,Izz;
        Matrix<float,3,3> tempASM;  // ASM: 0, -c, b, c, 0, -a, -b, a, 0
        vector<Matrix<float,3,3> > ftsPosASM;
        Matrix<float,3,3> tempEye33;
        Matrix<float,3,3> tempZero33;
        Matrix<float,Dynamic,Dynamic> A; //6*3c
        Matrix<float,Dynamic,Dynamic> W; //3c*3c
        Matrix<float,Dynamic,Dynamic> H; //3c*3c
        Matrix<float,3,1> gravity;
        Matrix<float,6,1> b61;
        Matrix<float,6,6> S66;
        Matrix<float,6,1> g61;
        Matrix<float,3,3> Kpcom;
        Matrix<float,3,3> Kdcom;
        Matrix<float,3,3> Kpbase;
        Matrix<float,3,3> Kdbase;
        Matrix<float,3,1> acc_DCOM;
        Matrix<float,3,1> angelAcc_DBase;
        Matrix<float,3,1> omegaDBase;
        Matrix<float,3,1> omegaBase;
        void calVmcCom();
        void updateImuData();
        void updateFtsPresForce(vector<float> torque);
        void updateTargTor(Matrix<float, 3, 4> force);
        void impParaDeliver();
        void impCtller(int mode);
        void impChangePara(Matrix<float, 4, 3> mK, Matrix<float, 4, 3> mB, Matrix<float, 4, 3> mM, int mode);
        IMPControl();
};

void string2float(string add, float* dest);

#endif