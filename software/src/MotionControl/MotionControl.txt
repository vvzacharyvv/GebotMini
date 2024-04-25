#include "MotionControl.hpp"

#define LF_PIN	1
#define RF_PIN	24
#define LH_PIN	28
#define RH_PIN	29

#define ForceLPF  0.9
#define StepHeight  (30.0/1000.0)
#define TimeHeight (2.0/4.0)  // time for trajectory within vertical part
#define swingVelFactor 3      

/**
 * @brief 
 * Open the file to read float data to dest.    
 * In the file, ',' must be used after every data, including the last data.  
 * @param add The address of the file to read, like "../include/init_Motor_angle.csv"
 * @param dest Floating pointer to store datas.
 */
void string2float(string add, float* dest)
{
    char data_char[8000],*char_float;
    const char *a=",";  //Separate datas
    int i=0;
    std::ifstream inidate;
    inidata.open(add);
    if (inidata)    cout<<"file open Successful"<<endl;
    else    cout<<"file open FAIL"<<endl;
    inidata.read(data_char,8000);
    char_float=strtok(data_char, a);
    while(char_float!=NULL)
    {        
        dest[i++] = stof(char_float);
        //cout<<'|'<<dest[i-1]<<endl;
        char_float=strtok(NULL, a);
    }
    inidata.close();
}

MotionControl::MotionControl()
{
    Matrix<float, 3, 3> temp;
    temp << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    for(uint8_t i=0; i<4; i++)
        jacobian_vector.push_back(temp);

    initFlag = false;
    timePresent = 0.0;
    timePresentForSwing << 0.0, 0.0, 0.0, 0.0;
    targetCoMVelocity << 0.0, 0.0, 0.0,0.0,0.0,0.0;
    L1 = 33.5 / 1000;
    L2 = 47.5 / 1000;
    L3 = 23.1 / 1000;
    width = 132.0 / 1000;
    length = 172.0 / 1000; 
    height=30.0/1000; 
    mass=500.0/1000;
    shoulderPos << width/2, length/2, width/2, -length/2, -width/2, length/2, -width/2, -length/2;  // X-Y: LF, RF, LH, RH
}
/**
 * @brief set phases for gait
 * 
 * @param tP The time of one period
 * @param tFGP The time of the whole period
 * @param tFSP The time of stance phase on start and end, in order LF, RF, LH, RH
 */
void MotionControl::setPhase(float tP, float tFGP, Matrix<float, 4, 2> tFSP)
{
    timePeriod = tP;
    timeForGaitPeriod = tFGP;
    timeForStancePhase = tFSP;
    timePresent = 0.0;
    timePresentForSwing << 0.0, 0.0, 0.0, 0.0;
    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
        if(timeForStancePhase(legNum,0) < timeForStancePhase(legNum,1))
            timeForSwing(legNum) = (timeForGaitPeriod - (timeForStancePhase(legNum,1) - timeForStancePhase(legNum,0) - timePeriod));
        else
            timeForSwing(legNum) = timeForStancePhase(legNum,0) - timeForStancePhase(legNum,1) - timePeriod;
    }
}
/**
 * @brief set initial position of feet in shoulder coordinate
 * 
 * @param initPosition foot position in shoulder coordinate.
 * @note The lenth of legs, whitch is L1, L2, L3 in constructor of MotionControl.
 */

void MotionControl::setInitPos(Matrix<float, 4, 3> initPosition)
{
    stancePhaseStartPos = initPosition;
    stancePhaseEndPos = initPosition;
    legPresPos = initPosition;
    legCmdPos = initPosition;
    initFootPos = initPosition;
    targetCoMPosition.setZero();
}
/**
 * @brief 
 * 
 * @param tCV 
 * set  Vel of X,Y,alpha in world cordinate
 */
void MotionControl::setCoMVel(Vector<float, 6> tCV)
{
    targetCoMVelocity = tCV;
}
/**
 * @brief 
 * 
 * @param jointPos 
 * put (vector)jointPos[12] into (Matrix)jointPresPos(4,3)
 */
void MotionControl::updatejointPresPos(vector<float> jointPos)
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
            jointPresPos(i,j) = jointPos[i*3 + j];
    }
}
/**
 * @brief 
 * 
 * @param jointVel 
 * put (vector)jointVel[12] into (Matrix)jointPresVel(4,3)
 */
void MotionControl::updatejointPresVel(vector<float> jointVel)
{
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<3; j++)
            jointPresVel(i,j) = jointVel[i*3 + j];
    }
}
/**
 * @brief 
 * Calculcate jacobian_vector with jointPresPos
 */
void MotionControl::updateJacobians()
{
    
       Matrix<float, 3, 3>  CHANGE; CHANGE << 0, 0, 1,
        1, 0, 0,
        0, 1, 0;
    
    for (uint8_t legNum = 0; legNum < 4; legNum++)  // LF RF LH RH
    {
        float th3, th1, th2;
        float factor_y, factor_z, factor_By, factor_Cy, factor_Bz, factor_Cz, factor_Ax, factor_Bx;
      /*  th1 = motorPos(legNum, 1);
        th2 = motorPos(legNum, 0);
        th3 = motorPos(legNum, 2);*/
        th1 = jointPresPos(legNum, 0);
        th2 = jointPresPos(legNum, 1);
        th3 = jointPresPos(legNum, 2);
        switch (legNum)
        {
        case 0:
            factor_By = -1;
            factor_Cy = 1;
            factor_Bz = 1;
            factor_Cz = 1;
            factor_Ax = -1;
            factor_Bx = -1;
            factor_y = 1;
            factor_z = 1;
            break;
        case 1:
            factor_By = 1;
            factor_Cy = -1;
            factor_Bz = -1;
            factor_Cz = -1;
            factor_Ax = 1;
            factor_Bx = -1;
            factor_y = -1;
            factor_z = -1;
            break;
        case 2:
            factor_By = 1;
            factor_Cy = 1;
            factor_Bz = -1;
            factor_Cz = 1;
            factor_Ax = -1;
            factor_Bx = 1;
            factor_y = 1;
            factor_z = 1;
            break;
        case 3:
            factor_By = -1;
            factor_Cy = -1;
            factor_Bz = 1;
            factor_Cz = -1;
            factor_Ax = 1;
            factor_Bx = 1;
            factor_y = -1;
            factor_z = -1;
            break;
        default:
            break;
        }
 /*       jacobian_vector[legNum](0, 0) = factor_y* ( - sin(th1) * cos(th2) * L1 - factor_By * sin(th1) * sin(th2 + th3) * L2 + factor_Cz * cos(th1) * L3);
        jacobian_vector[legNum](0, 1) = factor_y * (-cos(th1) * sin(th2) * L1 + factor_By*cos(th1) * cos(th2 + th3) * L2);
        jacobian_vector[legNum](0, 2) = factor_y * (factor_By*cos(th1) * cos(th2 + th3) * L2);
        jacobian_vector[legNum](1, 0) = factor_z * (cos(th1) * cos(th2) * L1 +factor_Bz* cos(th1) * sin(th2 + th3) * L2 -factor_Cz* sin(th1) * L3);
        jacobian_vector[legNum](1, 1) = factor_z * (-sin(th1) * sin(th2) * L1 +factor_Bz* sin(th1) * cos(th2 + th3) * L2);
        jacobian_vector[legNum](1, 2) = factor_z * (factor_Bz*sin(th1) * cos(th2 + th3) * L2);
        jacobian_vector[legNum](2, 0) = 0;
        jacobian_vector[legNum](2, 1) = factor_Ax*cos(th2) * L1 - fa0ctor_Bx*sin(th2 + th3) * L2;
        jacobian_vector[legNum](2, 2) = -factor_Bx*sin(th2 + th3) * L2;*/
 
        jacobian_vector[legNum](0, 0) = factor_y * (-sin(th1) * cos(th2) * L1 + factor_By * sin(th1) * sin(th2 + th3) * L2 + factor_Cy* cos(th1) * L3);
        jacobian_vector[legNum](0, 1) = factor_y * (-cos(th1) * sin(th2) * L1 - factor_By * cos(th1) * cos(th2 + th3) * L2);
        jacobian_vector[legNum](0, 2) = factor_y * (-factor_By * cos(th1) * cos(th2 + th3) * L2);
        jacobian_vector[legNum](1, 0) = factor_z * (cos(th1) * cos(th2) * L1 + factor_Bz * cos(th1) * sin(th2 + th3) * L2 + factor_Cz * sin(th1) * L3);
        jacobian_vector[legNum](1, 1) = factor_z * (-sin(th1) * sin(th2) * L1 + factor_Bz * sin(th1) * cos(th2 + th3) * L2);
        jacobian_vector[legNum](1, 2) = factor_z * (factor_Bz * sin(th1) * cos(th2 + th3) * L2);
        jacobian_vector[legNum](2, 0) = 0;
        jacobian_vector[legNum](2, 1) = factor_Ax * cos(th2) * L1 + factor_Bx * sin(th2 + th3) * L2;
        jacobian_vector[legNum](2, 2) = factor_Bx * sin(th2 + th3) * L2;
        jacobian_vector[legNum] = CHANGE * jacobian_vector[legNum];
    }
}
/**
 * @brief 
 * update Vel of feet in shoulder coordinate
 */
void MotionControl::updateFtsPresVel()
{
    legLastVel=legPresVel;
    Matrix <float, 3, 1> temp_vel;
    for(int i=0; i<4; i++)
    {
        temp_vel = jacobian_vector[i] * jointPresVel.row(i).transpose();
        legPresVel.row(i) = temp_vel.transpose();
    }
}
/**
 * @brief forwardKinematics
 * 
 * @param mode 
 * if mode=0    calculcate foot position(legCmdPos) with jointCmdPos(target),
 * if mode=1    calculcate foot position(legPresPos) with jointPresPos(present) and update legPos_last
 */
void MotionControl::forwardKinematics(int mode)
{
    for(uint8_t legNum=0; legNum<4; legNum++)  // LF RF LH RH
    {
        float x, y, z;//replace ftsPos(legNum,0),ftsPos(legNum,1),ftsPos(legNum,2);
        float factor_Ay, factor_By, factor_Cy, factor_Az, factor_Bz, factor_Cz, factor_Ax, factor_Bx;//The sign factors before the formulas L1, L2, and L3, where A represents the sign factors before L1, B represents the sign factors before L2, C represents the sign factors before L3
        float factor_y, factor_z;//the sign factors of whole formulas of y,z.
        float th1, th2, th3;
        if(mode==0)
        {
            th1 =  jointCmdPos(legNum,0);
            th2 =  jointCmdPos(legNum,1);
            th3 =  jointCmdPos(legNum,2);    
        }
        else if(mode==1)
        {
            th1 =  jointPresPos(legNum,0);
            th2 =  jointPresPos(legNum,1);
            th3 =  jointPresPos(legNum,2);     
        }
        switch (legNum)
        {
        case 0:
            factor_Ay = 1;
            factor_By = 1;
            factor_Cy = 1;
            factor_Az = 1;
            factor_Bz = 1;
            factor_Cz = -1;
            factor_Ax = -1;
            factor_Bx = 1;
            factor_y = 1;
            factor_z = 1;
            break;
        case 1:
            factor_Ay = 1;
            factor_By = -1;
            factor_Cy = -1;
            factor_Az = 1;
            factor_Bz = -1;
            factor_Cz = 1;
            factor_Ax = 1;
            factor_Bx = 1;
            factor_y = -1;
            factor_z = -1;
            break;
        case 2:
            factor_Ay = 1;
            factor_By = -1;
            factor_Cy = 1;
            factor_Az = 1;
            factor_Bz = -1;
            factor_Cz = -1;
            factor_Ax = -1;
            factor_Bx = -1;
            factor_y = 1;
            factor_z = 1;
            break;
        case 3:
            factor_Ay = 1;
            factor_By = 1;
            factor_Cy = -1;
            factor_Az = 1;
            factor_Bz = 1;
            factor_Cz = 1;
            factor_Ax = 1;
            factor_Bx = -1;
            factor_y = -1;
            factor_z = -1;
            break;
        default:
            break;
        }
        y = factor_y * (factor_Ay * cos(th1) * cos(th2) * L1 + factor_By * cos(th1) * sin(th2 + th3) * L2 + factor_Cy*sin(th1) * L3);
        z = factor_z * (factor_Az * sin(th1) * cos(th2) * L1 + factor_Bz * sin(th1) * sin(th2 + th3) * L2 + factor_Cz * cos(th1) * L3);
        x = factor_Ax * sin(th2) * L1 + factor_Bx * cos(th2 + th3) * L2;
        if(mode==0){
        legCmdPos(legNum, 0) = x;
        legCmdPos(legNum, 1) = y;
        legCmdPos(legNum, 2) = z;}
        else if(mode == 1)
        {
             legPos_last = legPresPos;
        legPresPos(legNum, 0) = x;
        legPresPos(legNum, 1) = y;
        legPresPos(legNum, 2) = z;
        }
    }

}
/**
 * @brief inverse Kinematics
 * 
 * @param cmdpos 
 * Calculcate joint angles (jointCmdPos) for motors with foot position(cmdpos) in shoulder coordinate
 */
void MotionControl::inverseKinematics(Matrix<float, 4, 3> cmdpos)
{
     float factor_x, factor_y, factor_z, factor_1, factor_2, factor_0;
    float x, y, z;
    for (int legNum = 0; legNum < 4; legNum++)
    {
        x = cmdpos(legNum, 0);
        y = cmdpos(legNum, 1);
        z = cmdpos(legNum, 2);
        switch (legNum)
        {
        case 0:
            factor_x = -1;
            factor_y = 1;
            factor_z = 1;
            factor_1 = 1;
            factor_2 = -1;
            factor_0 = 1;
            break;
        case 1:
            factor_x = 1;
            factor_y = -1;
            factor_z = -1;
            factor_1 = -1;
            factor_2 = 1;
            factor_0 = -1;
            break;
        case 2:
            factor_x = -1;
            factor_y = 1;
            factor_z = 1;
            factor_1 = 1;
            factor_2 = 1;
            factor_0 = -1;
            break;
        case 3:
            factor_x = 1;
            factor_y = -1;
            factor_z = -1;
            factor_1 = -1;
            factor_2 = -1;
            factor_0 = 1;
            break;

        }
        // jointCmdPos(legNum, 1) = atan2(factor_z * z, factor_y * y) + factor_1 * atan2(L3, sqrt(z * z + y * y - L3 * L3));
        // jointCmdPos(legNum, 2) = factor_2 * asin((L1 * L1 + L2 * L2 + L3 * L3 - x * x - y * y - z * z) / (2 * L1 * L2));
        // jointCmdPos(legNum, 0) = atan2(factor_x * x, sqrt((L1 + factor_0 * sin(jointCmdPos(legNum, 2)) * L2) * (L1 + factor_0 * sin(jointCmdPos(legNum, 2)) * L2) + (cos(jointCmdPos(legNum, 2)) * L2) * (cos(jointCmdPos(legNum, 2)) * L2) - x * x)) + factor_0 * atan2(cos(jointCmdPos(legNum, 2)) * L2, L1 + factor_0 * L2 * sin(jointCmdPos(legNum, 2)));
         jointCmdPos(legNum, 0) = atan2(factor_z * z, factor_y * y) + factor_1 * atan2(L3, sqrt(z * z + y * y - L3 * L3));
         jointCmdPos(legNum, 1) = atan2(factor_x * x, sqrt((L1 + factor_0 * sin(jointCmdPos(legNum, 2)) * L2) * (L1 + factor_0 * sin(jointCmdPos(legNum, 2)) * L2) + (cos(jointCmdPos(legNum, 2)) * L2) * (cos(jointCmdPos(legNum, 2)) * L2) - x * x)) + factor_0 * atan2(cos(jointCmdPos(legNum, 2)) * L2, L1 + factor_0 * L2 * sin(jointCmdPos(legNum, 2)));
         jointCmdPos(legNum, 2) = factor_2 * asin((L1 * L1 + L2 * L2 + L3 * L3 - x * x - y * y - z * z) / (2 * L1 * L2));
    }
}


void MotionControl::nextStep()
{

    if (abs(timePresent - timeForGaitPeriod ) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        timePresent = 0.0;
        // legCmdPos = initFootPos;
    }

    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
        if(timeForStancePhase(legNum,0) < timeForStancePhase(legNum,1))
        {
            if(timePresent > timeForStancePhase(legNum,0) - timePeriod/2 && timePresent < timeForStancePhase(legNum,1) - timePeriod/2 )
                // check timePresent is in stance phase or swing phase, -timePeriod/2 is make sure the equation is suitable
                stepFlag[legNum] = stance;
            else    //swing phase 
                stepFlag[legNum] = swing;            
        }
        else
        {
            if(timePresent > timeForStancePhase(legNum,0) - timePeriod/2 || timePresent < timeForStancePhase(legNum,1) - timePeriod/2 )
                // check timePresent is in stance phase or swing phase, -timePeriod/2 is make sure the equation is suitable
                stepFlag[legNum] = stance;
            else    //swing phase 
                stepFlag[legNum] = swing;
        }
        if(stepFlag[legNum] == stance ) //stance phase
        {     
            for(uint8_t pos=0; pos<3; pos++)
            {
                targetCoMPosition(legNum, pos) += targetCoMVelocity(pos) * timePeriod;
                targetCoMPosture(legNum,pos) +=targetCoMVelocity(pos+3)*timePeriod;
            }
            if(abs(timePresent - timeForStancePhase(legNum,0)) < 1e-4)  // if on the start pos 
            {
                stancePhaseStartPos(legNum) = legCmdPos(legNum);
                for(uint8_t pos=0; pos<3; pos++)
                    targetCoMPosition(legNum, pos) = 0.0;
            }
            Matrix<float, 3, 3> trans;
            trans<<cos(targetCoMPosture(legNum,2)), -sin(targetCoMPosture(legNum,2)), targetCoMPosition(legNum,0),
                sin(targetCoMPosture(legNum,2)), cos(targetCoMPosture(legNum,2)), targetCoMPosition(legNum,1),
                0, 0, 1;
            Matrix<float, 3, 1> oneShoulderPos_3x1;
            oneShoulderPos_3x1<<shoulderPos(legNum,0), shoulderPos(legNum,1), 1;
            oneShoulderPos_3x1 = trans * oneShoulderPos_3x1;

            if(abs(timePresent - timeForStancePhase(legNum,0)) < 1e-4)  // if on the start pos 
            {
                stancePhaseStartPos(legNum) = legCmdPos(legNum);
                // shoulderPos(legNum, 0) = oneShoulderPos_3x1(0);
                // shoulderPos(legNum, 1) = oneShoulderPos_3x1(1);
            }
            if(abs(timePresent - timeForStancePhase(legNum,1)) < timePeriod + 1e-4)  // if on the end pos   
                stancePhaseEndPos(legNum) = legCmdPos(legNum);

            legCmdPos(legNum, 0) = stancePhaseStartPos(legNum, 0) + (shoulderPos(legNum, 0) - oneShoulderPos_3x1(0));
            legCmdPos(legNum, 1) = stancePhaseStartPos(legNum, 1) + (shoulderPos(legNum, 1) - oneShoulderPos_3x1(1));
        }
        else    //swing phase 
        {
            Matrix<float, 1, 3> swingPhaseVelocity = -(stancePhaseEndPos.row(legNum) - stancePhaseStartPos.row(legNum)) / timeForSwing(legNum) ;
            float x, xh, m, n, k;
            //cout<<"legNum_"<<(int)legNum<<":"<<swingPhaseVelocity.array()<<"  ";
            if( ( timePresentForSwing(legNum) - timeForSwing(legNum) * TimeHeight ) < 1e-4 && timePresentForSwing(legNum) > -1e-4 && swingPhaseVelocity( 0, 0) != 0)
            {            
                for(uint8_t pos=0; pos<2; pos++)
                    legCmdPos(legNum, pos) += swingPhaseVelocity(pos) * timePeriod * swingVelFactor;     // for vertical down
                x = legCmdPos(legNum, 0) - stancePhaseEndPos(legNum, 0);
                xh = -(stancePhaseEndPos(legNum, 0) - stancePhaseStartPos(legNum, 0)) * TimeHeight * swingVelFactor;

                /* z = -( x - m )^2 + n + z0    */
                // m = (StepHeight + xh * xh) /2 /xh;
                // n = m * m;
                // legCmdPos(legNum, 2) = -(x - m) * (x - m) + n + stancePhaseEndPos(legNum, 2);

                /* z = -k * ( x - xh )^2 + H + z0 */
                k = StepHeight / xh / xh;
                legCmdPos(legNum, 2) = -k * (x - xh) * (x - xh) + StepHeight + stancePhaseEndPos(legNum, 2);
            }
            else if( timePresentForSwing(legNum) - timeForSwing(legNum) < 1e-4 && swingPhaseVelocity( 0, 0) != 0)
            {
                for(uint8_t pos=0; pos<2; pos++)
                    legCmdPos(legNum, pos) += swingPhaseVelocity(pos) * timePeriod * (1 - swingVelFactor * TimeHeight) / (1 - TimeHeight); // targetCoMVelocity
                legCmdPos(legNum, 2) -= StepHeight / timeForSwing(legNum) / (1 - TimeHeight) * timePeriod;
            }  

            if(swingPhaseVelocity( 0, 0) == 0)      //first step
            {
                if( ( timePresentForSwing(legNum) - timeForSwing(legNum)/2 ) < 1e-4 && timePresentForSwing(legNum) > -1e-4)
                    legCmdPos(legNum, 2) += StepHeight / timeForSwing(legNum) * 2 * timePeriod;
                if( ( timePresentForSwing(legNum) - timeForSwing(legNum)/2 ) > -1e-4)
                    legCmdPos(legNum, 2) -=  StepHeight / timeForSwing(legNum) * 2 * timePeriod;
            }
        }
        //cout<<"legNum_"<<(int)legNum<<":"<<stepFlag[legNum]<<"  ";
    }

    air_control();
    for(uint8_t legNum=0; legNum<4; legNum++)
    {
        if(stepFlag[legNum] != stance) timePresentForSwing(legNum) += timePeriod;
        else timePresentForSwing(legNum) = 0;   //stance phase
    }
    timePresent += timePeriod;
}
//robot's air control & imu update
void MotionControl::pumpAllNegtive(uint8_t legNum)
{
    svStatus=0b00000000;
    api.setSV(svStatus);
}
void MotionControl::pumpAllPositve(uint8_t legNum)
{
    svStatus=0b11111111;
    api.setSV(svStatus);
}
void MotionControl::pumpPositive(uint8_t legNum)
{
    svStatus=svStatus|(0b00000011<<((3-legNum)<<1));
    api.setSV(svStatus);
}
void MotionControl::pumpNegtive(uint8_t legNum)
{   
    svStatus=svStatus&!(0b00000011<<((3-legNum)<<1))
    api.setSV(svStatus);
}
/**
 * @brief control SV to ensure that robot has a suitable positive and negative pressure state to adhere to the wall
 * 
 */
void MotionControl::air_control()
{
    for(int legNum=0;legNum<4;legNum++)
    {
        if(stepFlag[legNum]==attach)
        {
            pumpNegative(legNum);
        }
        if(stepFlag[legNum]==detach)
        {
            pumpPositive(legNum);
        }

    }

}




/////////////////////////////////////////////////////////////////////////////
//                                          IMPControl
//////////////////////////////////////////////////////////////////////////////

IMPControl::IMPControl()
{
    float impdata[200];
    // string2float("../include/imp_parameter.csv",impdata);   // 0-stance, 1-swing, 2-detach, 3-attach
    string2float("../include/adm_parameter.csv",impdata);   // adm
    //Map<Matrix<float, 4, 3, RowMajor>> mapK(impdata), mapB(impdata + 12 * 1), mapM(impdata + 12 * 2);
    for(int i=0; i<4; i++)
    {
        Map<Matrix<float, 4, 3, RowMajor>> mapK(impdata + 36 * i), mapB(impdata + 12 + 36 * i), mapM(impdata  + 24 + 36 * i);
        impChangePara(mapK,mapB,mapM, i);
    } 

    xc_dotdot.setZero();
    xc_dot.setZero();
    xc.setZero();
    target_pos.setZero();
    target_vel.setZero();
    target_acc.setZero();
    target_force.setZero();
    force_last.setZero();
    impCtlRate = 100;
    // cal inertial
    Ixx=((length*length)+(height*height))*mass/12;
}

/**
 * @brief 
 * Calculcate feedback force in LPF with jacobian for impCtller.
 * @note Feedback force is driving force. Use the counter force to represent the driving force. driving force = - counter force
 * @param torque update present force with present_torque
 */
void IMPControl::updateFtsPresForce(vector<float> torque)
{
    Matrix<float, 3, 4> temp;
    if(force(2,3) - force_last(2,3) > 0.3 || force(2,3) - force_last(2,3) < -0.3)
        temp.setZero();
    for(int i=0; i<3; i++)
        for(int j=0;j<4;j++)
            temp(i ,j ) = torque[i+j*3];
    for (int i=0; i<4; i++)
        force.col(i) = ForceLPF * force_last.col(i) + (1-ForceLPF) * jacobian_vector[i].transpose().inverse() * temp.col(i);
    force_last = force;
}
/**
 * @brief 
 * Calculcate target torque with force and jacobin.
 * @param force 
 */
void IMPControl::updateTargTor(Matrix<float, 3, 4> force)
{
    for (int i=0; i<4; i++)
        target_torque.col(i) = jacobian_vector[i] * force.col(i);
}
/**
 * @brief 0
 * Deliver parameters in nextstep() to impCtller.
 */
void IMPControl::impParaDeliver()
{
    #ifdef  VMCCONTROL
    calVmcCom();
    #else
    target_pos = legCmdPos;
    for(uint8_t legNum=0; legNum<4; legNum++)
    {   
        if(stepFlag[legNum] == swing) //swing
        {
            if( ( timePresentForSwing(legNum) - (timeForSwing(legNum) - timePeriod *8) ) > -1e-4 )
            {
                stepFlag[legNum] = attach;   //attach
                target_force.row(legNum) << 0, 0, -1.6;  // x, y, z
            }
            else if( ( timePresentForSwing(legNum) - timePeriod *8 ) < 1e-4 && timePresentForSwing(legNum) > 1e-4)
            {
                stepFlag[legNum] = detach;   //detach
                target_force.row(legNum) << 0, 0, 1.5;
            }
            else    //swing
            {
                stepFlag[legNum] = swing;
                target_force.row(legNum) << 0, 0, 0;
            }
        }
        else        //stance
        {
            stepFlag[legNum] = stance;
           // target_force.row(legNum) << -0.6, 0, -1.6;    
        }
    }
    #endif

    
    // target_force<< 
    // 0, 0, -1.0,
    // 0, 0, -1.0,
    // 0, 0, -1.0,
    // 0, 0, -1.0;
}
/**
 * @brief 
 * Calculcate legCmdPos with target_acc, target_force, target_vel, target_pos and force,
 * or calculcate target force with target_acc, target_force, target_vel, target_pos and legPresPos.
 * 
 * @param mode 0 for Impedance control;    1 for Admittance control
 */
void IMPControl::impCtller(int mode)
{
    if(mode == 0)  // Impedance control
    {
        xc_dot = (legPresPos - legPos_last) * impCtlRate;
        xc_dotdot = (legPresVel - xc_dot) * impCtlRate;
        for(uint8_t legNum=0; legNum<4; legNum++)
        {
            force.transpose().row(legNum) = target_force.row(legNum)  
            + K_stance.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
            + B_stance.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) ;
            // + M_stance.row(legNum).cwiseProduct(target_acc.row(legNum) - xc_dotdot.row(legNum));
            cout<<"legNum_"<<(int)legNum<<":"<<stepFlag[legNum]<<endl;
            cout<<"K__stance_"<<(int)legNum<<"  "<<K_stance.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))<<endl;
            cout<<"B__stance_"<<(int)legNum<<"  "<<B_stance.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))<<endl;
            // cout<<"force"<<force.transpose().row(legNum)<<endl;
        }
        updateTargTor(force);
        // cout<<"target_pos:"<<endl<<target_pos<<endl;
        // cout<<"legPresPos:"<<endl<<legPresPos<<endl;
        cout<<"force:"<<endl<<force<<endl;
        cout<<"target_torque:"<<endl<<target_torque<<endl<<endl;
    }
    else if(mode == 1)  // Admittance control    xc_dotdot = 0 + M/-1*( - footForce[i][0] + refForce[i] - B * (pstFootVel[i][0] - 0) - K * (pstFootPos[i][0] - targetFootPos(i,0)));
    {
        for(uint8_t legNum=0; legNum<4; legNum++)
        {   
            switch(stepFlag[legNum])
            {
                case 0: //stance
                    xc_dotdot.row(legNum) =  target_acc.row(legNum) 
                    + K_stance.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
                    + B_stance.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) 
                    + M_stance.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );
                    // cout<<"K__stance_"<<(int)legNum<<"  "<<K_stance.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))<<endl;
                    // cout<<"B__stance_"<<(int)legNum<<"  "<<B_stance.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))<<endl;
                    // cout<<"M__stance_"<<(int)legNum<<"  "<<M_stance.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )<<endl;
                    break;

                case 1: //swing
                    xc_dotdot.row(legNum) =  target_acc.row(legNum) 
                    + K_swing.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
                    + B_swing.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) 
                    + M_swing.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );

                    break;

                case 2: //detach
                    xc_dotdot.row(legNum) =  target_acc.row(legNum) 
                    + K_detach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
                    + B_detach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))   
                    + M_detach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );
                    cout<<"K__detach_"<<(int)legNum<<"  "<<K_detach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))<<endl;
                    cout<<"B__detach_"<<(int)legNum<<"  "<<B_detach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))<<endl;
                    cout<<"M__detach_"<<(int)legNum<<"  "<<M_detach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )<<endl;
                    break;

                case 3: //attach
                    xc_dotdot.row(legNum) =  target_acc.row(legNum) 
                    + K_attach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
                    + B_attach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) 
                    + M_attach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );
                    cout<<"K__attach_"<<(int)legNum<<"  "<<K_attach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))<<endl;
                    cout<<"B__attach_"<<(int)legNum<<"  "<<B_attach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum))<<endl;
                    cout<<"M__attach_"<<(int)legNum<<"  "<<M_attach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) )<<endl<<endl;                 
                    break;
            }
            // cout<<stepFlag[legNum]<<endl;
             xc_dotdot.row(legNum) =  target_acc.row(legNum) 
            + K_attach.row(legNum).cwiseProduct(target_pos.row(legNum) - legPresPos.row(legNum))
            + B_attach.row(legNum).cwiseProduct(target_vel.row(legNum) - legPresVel.row(legNum)) 
            + M_attach.row(legNum).cwiseInverse().cwiseProduct( target_force.row(legNum) - force.transpose().row(legNum) );
        }
        xc_dot =  legPresVel + xc_dotdot * (1/impCtlRate);
        xc =  legPresPos + (xc_dot * (1/impCtlRate));
        
        // cout<<"force_\n"<<force.transpose()<<endl;
    }

}

/**
 * @brief 
 * Change parameters ofimp
 * @param mK K of impCtller
 * @param mB B of impCtller
 * @param mM M of impCtller
 * @param mode 
 * 0-stance, 1-swing, 2-detach, 3-attach
 */
void IMPControl::impChangePara(Matrix<float, 4, 3> mK, Matrix<float, 4, 3> mB, Matrix<float, 4, 3> mM, int mode)
{
    switch(mode) 
    {
        case 0:
            K_stance = mK; B_stance = mB; M_stance = mM;
            break;
        case 1:
            K_swing = mK; B_swing = mB; M_swing = mM;
            break;
        case 2:
            K_detach = mK; B_detach = mB; M_detach = mM;
            break;
        case 3:
            K_attach = mK; B_attach = mB; M_attach = mM;   
            break; 
    }

}
/**
 * @brief Update imu information
 * 
 */
  void IMPControl::updateImuData()
  {
    Matrix<float,3,1> preOmegaBase;preOmegaBase<<0.0,0.0,0.0;
    Matrix<float,3,1> tempg;tempg<<0,0,9.8;
    omegaBase=preOmegaBase;
    gravity=tempg;
  }
/**
 * @brief 
 * Calculcate  the target_force with qp.
 * 
 */
void IMPControl::calVmcCom()
{
    bool isOk=1;
    int stanceCount=0;
    int stanceUsefulCount=0;
    std::vector<int> stanceLegNum;
    Matrix<float,3,1> stanceAccumlate;
    stanceAccumlate.setZero();
    for(int legNum=0;legNum<4;legNum++)
    {
        if(stepFlag[legNum]==stance)
        { 
        stanceLegNum.push_back(legNum);
        Matrix<float,3,1> temp_vel = jacobian_vector[legNum] * jointPresVel.row(legNum).transpose();
        legPresVel.row(legNum) = temp_vel.transpose();
            for(int i=0;i<3;i++)
            {
                if((legPresVel.row(legNum)[i]-legLastVel.row(legNum)[i])/legLastVel.row(legNum)[i]>0.3||(legPresVel.row(legNum)[i]-legLastVel.row(legNum)[i])/legLastVel.row(legNum)[i]<-0.3)
                isOk=0;
            
                if(i==2)
                    {
                        if(isOk)
                        {
                        stanceAccumlate+=legPresVel.row(legNum).transpose();
                        stanceUsefulCount++;
                        }
                        isOk=1;
                    }
                    
            }
        }

    }
    Kdcom<<200,0,0,0,200,0,0,0,0;
    Kdbase<<200,0,0,0,200,0,0,0,200;
    Matrix<float,3,1> targetComVelxyz;
    targetComVelxyz=targetCoMVelocity.block(0,0,3,1);
    presentCoMVelocity=-stanceAccumlate/stanceUsefulCount;
    acc_DCOM=Kdcom*(targetComVelxyz-presentCoMVelocity);
   
    vector<int>::size_type tempSize=stanceLegNum.size();
    int stanceCount=(int)tempSize;
    omegaBase=targetCoMVelocity.block(3,0,3,1);
    updateImuData();
    angelAcc_DBase=Kdbase(omegaDBase-omegaBase);//cal angelAcc


    b61<<mass*(acc_DCOM+gravity),Ixx*angelAcc_DBase[0],Iyy*angelAcc_DBase[1],Izz*angelAcc_DBase[2];
    int k=3*stanceCount;
    tempASM.setZero();
    S66.setIdentity(6,6);
    S66=S66*100;
    W.setIdentity(k,k);
    for(int i=0; i<4; i++)
    {
        ftsPosASM.push_back(tempASM);
    }
    for(int i=0; i<4; i++)
    {
    tempASM << 0, -legCmdPos(i, 2), legCmdPos(i, 1), legCmdPos(i, 2), 0, -legCmdPos(i, 0), -legCmdPos(i, 1), legCmdPos(i, 0), 0;
    ftsPosASM[i] = tempASM;
    }
    A.resize(6,k);
    for(int i=0;i<stanceCount;i++)
    {
        A.block<3,3>(0,i*3)<<MatrixXf::Identity(3, 3);
        A.block<3,3>(3,i*3)<<ftsPosASM[stanceLegNum[i]];
    }
    H.resize(k,k);
    H=2*A.transpose()*S66*A+2*W;
    g61=-2*A.transpose()*S66*b61;

    //qp
    qpOASES::Options options;
    options.initialStatusBounds = qpOASES::ST_INACTIVE;
	options.numRefinementSteps = 3;
	options.enableCholeskyRefactorisation = 1;
    qpOASES::QProblemB qpPrograme(k);
    qpPrograme.setOptions(options);
    qpOASES::real_t qp_H[k*k]={};
    qpOASES::real_t qp_g[6]={};
    qpOASES::real_t lb[6]={-200,-200,-200,-200,-200,-200};
    qpOASES::real_t ub[6]={200,200,200,200,200,200};
        for(int i=0; i<k; i++)
    {
        qp_g[i] = g61(i,0);
        for(int j=0; j<k; j++)
        {
        qp_H[i*6+j] = H(i,j);
        }
    }
    qpOASES::int_t nWSR = 10;
    qpPrograme.init(qp_H,qp_g,lb,ub,nWSR,0);
    qpOASES::real_t xOpt[k];
	qpPrograme.getPrimalSolution( xOpt );
    for(int i=0;i<3*stanceCount;i=i+3)
    {
        target_force.row(stanceLegNum[i/3])<<xOpt[i],xOpt[i+1];xOpt[i+2];
    }
}