#include "gebot.h"



CGebot::CGebot(float length,float width,float height,float mass)
{
    dxlMotors.init("/dev/ttyAMA0", 1000000, ID, 2);  // CAN NOT 4M.   ttyUSB0 ttyAMA0      
    m_glLeg[0] = new CLeg(LF,65.5,70.0,21.0);  // mm
    m_glLeg[1] = new CLeg(RF,65.5,70.0,21.0);
    m_glLeg[2] = new CLeg(LH,65.5,84.0,21.0);
    m_glLeg[3] = new CLeg(RH,65.5,84.0,21.0); //24
    m_fLength=length/1000.0;
    m_fWidth=width/1000.0;
    m_fHeight=height/1000.0;
    m_fMass=mass/1000.0; //g->kg
    m_threhold[0]=THREHOLDLF;
    m_threhold[1]=THREHOLDRF;
    m_threhold[2]=THREHOLDLH;
    m_threhold[3]=THREHOLDRH;
    mfShoulderPos<<m_fWidth/2, m_fLength/2, 0,1,m_fWidth/2, -m_fLength/2,0,1, -m_fWidth/2, m_fLength/2,0,1,-m_fWidth/2, -m_fLength/2,0,1;  // X-Y: LF, RF, LH, RH
    for (size_t i = 0; i < 4; i++)
    {
        touchTrigger[i]=false;
        probeTrigger[i]=false;
        attchTimes[i]=ATTACH_TIMES;
    }
    
    fSwingPhaseStatusPart[0]=0.4; //detach
    fSwingPhaseStatusPart[1]=0.3; //swingUp
    fSwingPhaseStatusPart[2]=0.3; //swingDown
    // fSwingPhaseStatusPart[3]=0.3; //attach
    fStancePhaseStatusPart[0]=0.1;//recover  
    fStancePhaseStatusPart[1]=0.9;//stance

    BSwingPhaseStartFlag = true;
    autoControlFlag=true;
    BSwingPhaseEndFlag = 0;     //
    mfCompensation.setZero();
    mfJointCompDis.setZero();

    for(int i=0;i<16;++i)
        vLastSetPos.push_back(0);
    fTimePresent=0.0;
    mfTimePresentForSwing.setZero();
    vfTargetCoMVelocity.setZero();
    dxlMotors.setOperatingMode(3);  //3 position control; 0 current control
    usleep(500);
    dxlMotors.torqueEnable();
     dxlMotors.getPosition();
    api.setPump(1, LOW);//LF
    api.setPump(24, LOW);//RF
    api.setPump(28, LOW);//LH
    api.setPump(29, LOW);//RH
    // api.setPump(1, HIGH);//LF
    // api.setPump(24, HIGH);//RF
    // api.setPump(28, HIGH);//LH
    // api.setPump(29, HIGH);//RH
    usleep(1e6);
}


/**
 * @brief set phases for gait
 * 
 * @param tP The time of one period
 * @param tFGP The time of the whole period
 * @param tFSP The time of stance phase on start and end, in order LF, RF, LH, RH
 */
void CGebot::SetPhase(float tP, float tFGP, Matrix<float,4,2> tFSP)
{
    float fSwPSFactor[4], fStPSFactor[2];
    fTimePeriod = tP;
    fTimeForGaitPeriod = tFGP;
    mfTimeForSwingPhase = tFSP;
    fTimePresent = 0.0;
    // mfTimeForStancePhase<< TimeForGaitPeriod/4.0 *3,          TimeForGaitPeriod/4.0 *2,   // tripod
    //                     TimeForGaitPeriod/4.0,             TimeForGaitPeriod,
    //                     TimeForGaitPeriod - TimePeriod,    TimeForGaitPeriod/4.0 *3,
    //                     TimeForGaitPeriod/4.0 *2,          TimeForGaitPeriod/4.0;
    mfTimeForStancePhase<<mfTimeForSwingPhase(0,1), mfTimeForSwingPhase(0,0),
                        mfTimeForSwingPhase(1,1), mfTimeForSwingPhase(1,0),
                        mfTimeForSwingPhase(2,1), mfTimeForSwingPhase(2,0),
                        mfTimeForSwingPhase(3,1), mfTimeForSwingPhase(3,0);
    mfTimePresentForSwing.setZero();

    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
        fTimeForSwing[legNum] = mfTimeForSwingPhase(legNum,1)-mfTimeForSwingPhase(legNum,0);
        iStatusCounterBuffer[legNum][int(detach)] = floor(fTimeForSwing[legNum] / fTimePeriod * fSwingPhaseStatusPart[0]);
        iStatusCounterBuffer[legNum][int(swingUp)] = floor(fTimeForSwing[legNum] / fTimePeriod * fSwingPhaseStatusPart[1]);
        iStatusCounterBuffer[legNum][int(swingDown)] = floor(fTimeForSwing[legNum] / fTimePeriod * fSwingPhaseStatusPart[2]);
        // iStatusCounterBuffer[legNum][int(attach)] = floor(fTimeForSwing[legNum] / fTimePeriod * fSwingPhaseStatusPart[3]);
        iStatusCounterBuffer[legNum][int(recover)] = floor( (fTimeForGaitPeriod - fTimeForSwing[legNum]) / fTimePeriod * fStancePhaseStatusPart[0]);
        iStatusCounterBuffer[legNum][int(stance)] = floor( (fTimeForGaitPeriod - fTimeForSwing[legNum]) / fTimePeriod * fStancePhaseStatusPart[1]);
    }
    
    fSwPSFactor[0]=fSwingPhaseStatusPart[0];
    fSwPSFactor[1]=fSwPSFactor[0]+fSwingPhaseStatusPart[1];
    fSwPSFactor[2]=fSwPSFactor[1]+fSwingPhaseStatusPart[2];
    // fSwPSFactor[3]=fSwPSFactor[2]+fSwingPhaseStatusPart[3];
    fStPSFactor[0]=fStancePhaseStatusPart[0];
    fStPSFactor[1]=fStPSFactor[0]+fStancePhaseStatusPart[1];
    for(uint8_t legNum=0; legNum<4; legNum++) 
    {
        if(fTimePresent - mfTimeForSwingPhase(legNum,0) >=  0 &&  fTimePresent - mfTimeForSwingPhase(legNum,0) <  fTimeForSwing[legNum] * fSwPSFactor[0] )
		{
            iStatusCounter[legNum] = floor((fTimeForSwing[legNum] * fSwPSFactor[0] - (fTimePresent-mfTimeForSwingPhase(legNum,0)) ) / fTimePeriod);
            m_glLeg[legNum]->ChangeStatus(detach);
        }
        else if(fTimePresent - mfTimeForSwingPhase(legNum,0) >=  fTimeForSwing[legNum] * fSwPSFactor[0] &&  fTimePresent - mfTimeForSwingPhase(legNum,0) <  fTimeForSwing[legNum] * fSwPSFactor[1] )
		{
            iStatusCounter[legNum] = floor((fTimeForSwing[legNum] * fSwPSFactor[1] - (fTimePresent-mfTimeForSwingPhase(legNum,0)) ) / fTimePeriod);
            m_glLeg[legNum]->ChangeStatus(swingUp);
        }
        else if(fTimePresent - mfTimeForSwingPhase(legNum,0) >=  fTimeForSwing[legNum] * fSwPSFactor[1] &&  fTimePresent - mfTimeForSwingPhase(legNum,0) <  fTimeForSwing[legNum] * fSwPSFactor[2] )
		{
            iStatusCounter[legNum] = floor((fTimeForSwing[legNum] * fSwPSFactor[2] - (fTimePresent-mfTimeForSwingPhase(legNum,0)) ) / fTimePeriod);
            m_glLeg[legNum]->ChangeStatus(swingDown);
        }
        // else if(fTimePresent - mfTimeForSwingPhase(legNum,0) >=  fTimeForSwing[legNum] * fSwPSFactor[2] &&  fTimePresent - mfTimeForSwingPhase(legNum,0) <  fTimeForSwing[legNum] * fSwPSFactor[3] )
		// {
        //     iStatusCounter[legNum] = floor((fTimeForSwing[legNum] * fSwPSFactor[3] - (fTimePresent-mfTimeForSwingPhase(legNum,0)) ) / fTimePeriod);
        //     m_glLeg[legNum]->ChangeStatus(attach);
        // }
        else //stance phase
		{
            if( fTimePresent + fTimeForGaitPeriod - mfTimeForStancePhase(legNum,0) < (fTimeForGaitPeriod - fTimeForSwing[legNum]) * fStPSFactor[0] )
            {
                iStatusCounter[legNum] = floor(((fTimeForGaitPeriod - fTimeForSwing[legNum]) * fStPSFactor[0] - (fTimePresent + fTimeForGaitPeriod - mfTimeForStancePhase(legNum,0)))/ fTimePeriod);
                m_glLeg[legNum]->ChangeStatus(recover);
            }
            else 
            {
                iStatusCounter[legNum] = floor(((fTimeForGaitPeriod - fTimeForSwing[legNum]) * fStPSFactor[1] - (fTimePresent + fTimeForGaitPeriod - mfTimeForStancePhase(legNum,0))) / fTimePeriod);
                m_glLeg[legNum]->ChangeStatus(stance);
            }
        }
    }  
}

/**
 * @brief Update the status of leg, record the start Pos and the end Pos of stance phase.
 * 
 * @param legNum the leg number
 */
void CGebot::UpdateLegStatus(int legNum)
{
     if(m_glLeg[legNum]->GetLegStatus()==attach)
    {
        attchTimes[legNum]--;
        if(attchTimes[legNum]<0){
            autoControlFlag=0;
            return;
        }
    }
    else
     iStatusCounter[legNum]--;
   
    // BSwingPhaseStartFlag = 0;
    // BSwingPhaseEndFlag = 0;
    bool ts=m_glLeg[legNum]->getTouchStatus() ;
   
    if(probeTrigger[legNum]== true){
        if(ts == true){
              probeTrigger[legNum]=false;
              touchTrigger[legNum]=true;
        }
    }
    if(iStatusCounter[legNum] <= 0 || touchTrigger[legNum] == true )
        switch(m_glLeg[legNum]->GetLegStatus())
        {
        case detach:
            m_glLeg[legNum]->ChangeStatus(swingUp);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(swingUp)];
            break;
        case swingUp:
            m_glLeg[legNum]->ChangeStatus(swingDown);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(swingDown)];
            break;
        case swingDown:
            if(ts != true){
                m_glLeg[legNum]->ChangeStatus(attach);
                iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(recover)];
                probeTrigger[legNum]=true;
            }
            else{
            m_glLeg[legNum]->ChangeStatus(recover);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(recover)];
            mfStancePhaseStartPos(legNum) = mfLegCmdPos(legNum);
            for(int pos=0; pos<3; pos++)
                    targetCoMPosition(legNum, pos) = 0.0;
            BSwingPhaseEndFlag = true;
            }
            break;
        case attach:
            m_glLeg[legNum]->ChangeStatus(recover);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(recover)];
            mfStancePhaseStartPos(legNum) = mfLegCmdPos(legNum);
            for(int pos=0; pos<3; pos++)
                    targetCoMPosition(legNum, pos) = 0.0;
            BSwingPhaseEndFlag = true;
            touchTrigger[legNum]=false;
            attchTimes[legNum]=ATTACH_TIMES;
            break;
        case recover:
            m_glLeg[legNum]->ChangeStatus(stance);
            attchDis[legNum]=0;
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(stance)];
            break;
        case stance:
            m_glLeg[legNum]->ChangeStatus(detach);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(detach)];
            mfStancePhaseEndPos(legNum) = mfLegCmdPos(legNum);
            mfSwingVelocity = -(mfStancePhaseEndPos.row(legNum) - mfStancePhaseStartPos.row(legNum)) / (iStatusCounterBuffer[legNum][(int)detach] + iStatusCounterBuffer[legNum][(int)swingUp] + iStatusCounterBuffer[legNum][(int)swingDown]) ;
            BSwingPhaseStartFlag = true;
            break;
        }
}

/**
 * @brief set initial position of feet in shoulder coordinate
 * 
 * @param initPosition foot position in shoulder coordinate.
 * @note The lenth of legs, whitch is L1, L2, L3 in constructor of MotionControl.
 */

void CGebot::SetInitPos(Matrix<float, 4, 3> initPosition)
{
    mfStancePhaseStartPos = initPosition;
    mfStancePhaseEndPos = initPosition;
    mfLegPresPos = initPosition;
    mfLegCmdPos = initPosition;
    mfInitFootPos = initPosition;
    targetCoMPosition.setZero();
}
/**
 * @brief init inertia of robot 
 * @note ,Ic 0,1 rep mass of lf 1 and 23,then rf 2 3,lh 4 5, rh 6 7.at last init body Ic;
 *         same to Pc;
 * 
 */
void CGebot::InertiaInit()
{
    float  I[256];
    string2float("../include/inertiaInit.csv", I);
    for(size_t i=0;i<96;i=i+12)
    {   
        Pc[i/12]<<I[i],I[i+1],I[i+2];
        Ic[i/12]<<I[i+3],I[i+4],I[i+5],
               I[i+6],I[i+7],I[i+8],
               I[i+9],I[i+10],I[i+11]; 
    }
    Pc_body<<-1.96,-0.22,10.32;
    Ic_body<<421534.86,-488.38,-36537.05,
             -488.38,1054973.20,-1750.03,
            -36537.05,-1750.03,1211574.07;
    //g*mm^2 2 kg*m^2;
    for (size_t i=0;i<8;i++)
    {
        Pc[i]=Pc[i]/1e3;
        //   cout<<"Pc:"<<endl;
        // cout<<Pc[i]<<endl;
    }
    for(size_t i=0;i<8;i++)
    {
        Ic[i]=Ic[i]/1e9;
        // cout<<"Ic:"<<endl;
        // cout<<Ic[i]<<endl;
    }
    Pc_body=Pc_body/1e3;
    Ic_body=Ic_body/1e9;
}

/**
 * @brief 
 * 
 * @param tCV 
 * set  Vel of X,Y,alpha in world cordinate
 */
void CGebot::SetCoMVel(Matrix<float, 6,1> tCV)
{
    vfTargetCoMVelocity = tCV;
}


/**
 * @brief 
 * 
 * @param jointPos 
 * put (vector)jointPos[12] into (Matrix)jointPresPos(4,3)
 */
void CGebot::UpdatejointPresPosAndVel()
{
    mfJointPresPos=inverseMotorMapping(dxlMotors.present_position);
    // cout<<"mfJointPresPos: " <<mfJointPresPos<<endl;
    // cout<<"mfJointLastPos: " <<mfJointLastPos<<endl;
    for(int legNum=0;legNum<4;legNum++)
    {   
        Matrix<float,3,1> temp=mfJointPresPos.block(legNum,0,1,3).transpose();
        m_glLeg[legNum]->SetJointPos(temp);
    }

    mfJointPresVel=(mfJointPresPos-mfJointLastPos)*loopRateCommandUpdate;
    mfJointLastPos=mfJointPresPos;
  
}

/**
 * @brief 
 * 
 * @param jointVel 
 * put (vector)jointVel[12] into (Matrix)jointPresVel(4,3)
 */
void CGebot::UpdatejointPresVel()
{
    mfJointPresVel=inverseMotorMapping(dxlMotors.present_velocity);    
    //cout<<"dxlMotors.present_velocity:  ";
    for(size_t i=0;i<dxlMotors.present_velocity.size();i++)   
    {
       // cout<<dxlMotors.present_velocity[i]<<" ";
    }
   // cout<<endl;
    //cout<<"mfJointPresVel:\n"<<mfJointPresVel<<endl;
}

void CGebot::UpdateJacobians()
{
    for(int legNum=0;legNum<4;legNum++)
        m_glLeg[legNum]->UpdateJacobian();
}

/**
 * @brief 
 * update Vel of feet in shoulder coordinate
 */
void CGebot::UpdateFtsPresVel()
{
    mfLegLastVel=mfLegPresVel;
    Matrix <float, 3, 1> temp_vel;
    for(int i=0; i<4; i++)
    {
        temp_vel = m_glLeg[i]->GetJacobian() * mfJointPresVel.row(i).transpose();
        mfLegPresVel.row(i) = temp_vel.transpose();
    }
    
}
void CGebot::NextStep()
{
    if (abs(fTimePresent - fTimeForGaitPeriod ) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        fTimePresent = 0.0;
        // legCmdPos = initFootPos;
    }

    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
        UpdateLegStatus(legNum);
        enum_LEGSTATUS ls=m_glLeg[legNum]->GetLegStatus(); //get present status
        if(legNum<2)
            fStepHeight = StepHeight_F;
        else
            fStepHeight = StepHeight_H;
        
        // cout<<"leg_"<<(int)legNum<<"_status: "<<(int)ls<<endl;
        if( ls == stance ) //stance phase
        {     
            for(uint8_t pos=0; pos<3; pos++)
            {
                targetCoMPosition(legNum, pos) += vfTargetCoMVelocity(pos) * fTimePeriod;
                targetCoMPosture(legNum,pos) +=vfTargetCoMVelocity(pos+3)*fTimePeriod;
            }
            Matrix<float, 4, 4> trans;
            trans<<cos(targetCoMPosture(legNum,2)), -sin(targetCoMPosture(legNum,2)),0, targetCoMPosition(legNum,0),
                sin(targetCoMPosture(legNum,2)), cos(targetCoMPosture(legNum,2)),0, targetCoMPosition(legNum,1),
                0,0,1,targetCoMPosition(legNum,2),
                0, 0, 0, 1;
            Matrix<float, 4, 1> oneShoulderPos_4x1;
            oneShoulderPos_4x1<<mfShoulderPos(legNum,0), mfShoulderPos(legNum,1),0,1;
            oneShoulderPos_4x1 = trans * oneShoulderPos_4x1;
            for (size_t i = 0; i < 3; i++)
            {
                mfLegCmdPos(legNum, i) = mfStancePhaseStartPos(legNum, i) + (mfShoulderPos(legNum, i) - oneShoulderPos_4x1(i));
                //  cout<<"mfStancePhaseStartPos(legNum, i): \n"<<mfStancePhaseStartPos(legNum, i)<<endl;
                //  cout<<"oneShoulderPos_3x1("<<i<<")"<<oneShoulderPos_4x1(i)<<endl;
                //  cout<<"mfShoulderPos(legNum,"<<i<<")"<<mfShoulderPos(legNum, i)<<endl;
            }
        }
        else if( ls == detach || ls == swingUp )   //swing phase 
        {
            // cout<<"swing-"<<(unsigned)legNum<<endl;
            float x, xh, m, n, k;
            if(mfSwingVelocity( 0, 0) == 0)      //first step
            {
                mfLegCmdPos(legNum, 2) += fStepHeight / (iStatusCounterBuffer[legNum][(int)detach] + iStatusCounterBuffer[legNum][(int)swingUp]);
                
            }
            else 
            {            
                for(uint8_t pos=0; pos<2; pos++)
                    mfLegCmdPos(legNum, pos) += mfSwingVelocity(pos);     
                x = mfLegCmdPos(legNum, 0) - mfStancePhaseEndPos(legNum, 0);
                xh = -(mfStancePhaseEndPos(legNum, 0) - mfStancePhaseStartPos(legNum, 0)) * (iStatusCounterBuffer[legNum][(int)detach] + iStatusCounterBuffer[legNum][(int)swingUp])  / (iStatusCounterBuffer[legNum][(int)detach] + iStatusCounterBuffer[legNum][(int)swingUp] + iStatusCounterBuffer[legNum][(int)swingDown]);

                /* z = -( x - m )^2 + n + z0    */
                // m = (StepHeight + xh * xh) /2 /xh;
                // n = m * m;
                // legCmdPos(legNum, 2) = -(x - m) * (x - m) + n + stancePhaseEndPos(legNum, 2);

                /* z = -k * ( x - xh )^2 + H + z0 */
                k = fStepHeight / xh / xh;
                mfLegCmdPos(legNum, 2) = -k * (x - xh) * (x - xh) + fStepHeight + mfStancePhaseEndPos(legNum, 2);
               
            }
        }
        else if( ls == swingDown )    //swing phase
        {
            if(mfSwingVelocity( 0, 0) != 0)     
            {
                for(uint8_t pos=0; pos<2; pos++)
                    mfLegCmdPos(legNum, pos) += mfSwingVelocity(pos) ;
            }   
            
            mfLegCmdPos(legNum, 2) -= fStepHeight / iStatusCounterBuffer[legNum][(int)ls];
        }
        else if( ls == attach )    //swing phase
        {
            mfLegCmdPos(legNum, 2) -=  ATTACHDIS_MAX/ATTACH_TIMES;
            attchDis[legNum]+=ATTACHDIS_MAX/ATTACH_TIMES;
        }
        else if( ls == recover )   //stance phase
        {   // mfLegCmdPos(legNum, 2)  is not related to mfStancePhaseEndPos(legNum, 2) and mfStartPhaseEndPos(legNum, 2)
            mfLegCmdPos(legNum, 2) +=  attchDis[legNum] / iStatusCounterBuffer[legNum][(int)ls];
        }
        //cout<<"legNum_"<<(int)legNum<<":"<<stepFlag[legNum]<<"  ";
    }
   
    for(uint8_t legNum=0; legNum<4; legNum++)
    {
        if(m_glLeg[legNum]->GetLegStatus()!= stance) mfTimePresentForSwing(legNum) += fTimePeriod;
        else mfTimePresentForSwing(legNum) = 0;   //stance phase
    }
    fTimePresent += fTimePeriod;
}

/**
 * @brief forwardKinematics
 * 
 * @param mode 
 * if mode=0    calculcate foot position(legCmdPos) with jointCmdPos(target),
 * if mode=1    calculcate foot position(legPresPos) with jointPresPos(present) and update legPos_last
 */
void CGebot::ForwardKinematics(int mode)
{
    if(mode==0)
        for(int legNum=0;legNum<4;legNum++)
            mfLegCmdPos.row(legNum)=m_glLeg[legNum]->ForwardKinematic().transpose();    // need SetJointPos(jointCmdPos)
    else{
        mfLegLastPos=mfLegPresPos;
        for(int legNum=0;legNum<4;legNum++)
            mfLegPresPos.row(legNum)=m_glLeg[legNum]->ForwardKinematic().transpose();  // need SetJointPos(jointPresPos)
    }
}

void CGebot::InverseKinematics(Matrix<float, 4, 3> cmdpos)
{
    for(int legNum=0;legNum<4;legNum++)
        mfJointCmdPos.row(legNum)=m_glLeg[legNum]->InverseKinematic(cmdpos.block(legNum,0,1,3)).transpose();
}

//motor control;
 void CGebot::SetPos(Matrix<float,4,3> jointCmdPos)
{
    vector<float> setPos;
    setPos=motorMapping(jointCmdPos);
    for(int i =0;i<4;i++)
    setPos.emplace_back(jointCmdPos(i,1));
    for(int i=0; i<4; i++)  
            for(int j=0;j<3;j++)
            {
                if( isnanf(setPos[i*3+j]))         
                {
                    setPos[i*3+j] = vLastSetPos[i*3+j];   // last
                    cout<<"-------------motor_angle_"<<i*3+j<<" NAN-----------"<<endl;
                    // cout<<"target_pos: \n"<<imp.target_pos<<"; \nxc: \n"<<imp.xc<<endl;
                    exit(0);
                }
                else
                {
                    if(setPos[i*3+j] - vLastSetPos[i*3+j] > MORTOR_ANGLE_AMP)
                    {
                        vLastSetPos[i*3+j] += MORTOR_ANGLE_AMP;
                        cout<<"-------------motor_angle_"<<i*3+j<<" +MAX-----------"<<endl;
                    }
                    else if(setPos[i*3+j] - vLastSetPos[i*3+j] < -MORTOR_ANGLE_AMP)
                    {
                        vLastSetPos[i*3+j] -= MORTOR_ANGLE_AMP;
                        cout<<"-------------motor_angle_"<<i*3+j<<" -MAX-----------"<<endl;
                    }
                    else {
                            vLastSetPos[i*3+j] = setPos[i*3+j];   // now
                            for(int i=12;i<16;i++)
                            vLastSetPos[i*3+j] = setPos[i*3+j]; 
                    }
                        
                }
                //cout<<"motor_angle_"<<i*3+j<<": "<<SetPos[i*3+j]<<"  ";
            }   
    dxlMotors.setPosition(vLastSetPos); 
 
}




//robot's air control 
//RF-RH-LH-LF
void CGebot::PumpAllNegtive()
{
    svStatus=0b01010101;// 01-N 10-P;
    api.setSV(svStatus);
}
void CGebot::PumpAllPositve()
{
    svStatus=0b10101010;
    api.setSV(svStatus);
}
void CGebot::PumpPositive(int legNum)
{
    if(legNum==0) legNum=3;
    else if(legNum==1) legNum=0;
    else if(legNum==2) legNum=2;
    else if(legNum==3) legNum=1;
    svStatus|=1<<((3-legNum)*2+1);
    //cout<<"svStatus1="<<bitset<8>(svStatus)<<"\n";
    svStatus&=~(1<<((3-legNum)*2));
    //cout<<"svStatus2="<<bitset<8>(svStatus)<<"\n";
    api.setSV(svStatus);
}
void CGebot::PumpNegtive(int legNum)
{   
    if(legNum==0) legNum=3;
    else if(legNum==1) legNum=0;
    else if(legNum==2) legNum=2;
    else if(legNum==3) legNum=1;
    svStatus|=1<<((3-legNum)*2);
   //  cout<<"svStatus1="<<bitset<8>(svStatus)<<"\n";
    svStatus&=~(1<<((3-legNum)*2+1));
    //cout<<"svStatus2="<<bitset<8>(svStatus)<<"\n";
    api.setSV(svStatus);
}
 /**
 * @brief control SV to ensure that robot has a suitable positive and negative pressure state to adhere to the wall
 * 
 */
void CGebot::AirControl()
{
    for(int legNum=0;legNum<4;legNum++)
    {
        if(m_glLeg[legNum]->GetLegStatus()==swingDown)  //attach
        {
            PumpNegtive(legNum);
        }
        else if(m_glLeg[legNum]->GetLegStatus()==stance)// Apply negative pressure in advance to solve gas path delay.
        {
            if(iStatusCounter[legNum] <= ceil(iStatusCounterBuffer[legNum][int(stance)] * PrePsotiveFactor) )   
            {
                PumpPositive(legNum);
                BSwingPhaseStartFlag = true;
            }    
        }
        else if(m_glLeg[legNum]->GetLegStatus()==detach)
            PumpPositive(legNum);
        //cout<<"svStatus:"<<std::setw(2)<<svStatus<<endl;
    }
}
/**
 * @brief Correct body tilt.
 * Notice: Applicable to amble gait in 180 degree
 */
void CGebot::AttitudeCorrection180()
{
    int legNum;
    if(BSwingPhaseStartFlag == true )
    {
        BSwingPhaseStartFlag = 0;
        for(legNum=0;legNum<4;legNum++)
        {
            switch(m_glLeg[legNum]->GetLegStatus()) // select swing leg
            {
            case detach:
            case swingUp:
            case swingDown:
            case attach:
                switch (legNum) //the number of swing leg is only 1 in amble gait   
                {
                case 0:
                    mfCompensation(0,2) = 0;
                    mfCompensation(1,2) = CompDisA3;
                    mfCompensation(2,2) = CompDisA2;
                    mfCompensation(3,2) = CompDisA1;
                    break;
                case 1:
                    mfCompensation(0,2) = CompDisA3;
                    mfCompensation(1,2) = 0;
                    mfCompensation(2,2) = CompDisA1;
                    mfCompensation(3,2) = CompDisA2;
                    break;
                case 2:
                    mfCompensation(0,2) = CompDisA2;
                    mfCompensation(1,2) = CompDisA1;
                    mfCompensation(2,2) = 0;
                    mfCompensation(3,2) = CompDisA3;
                    break;
                case 3:
                    mfCompensation(0,2) = CompDisA1;
                    mfCompensation(1,2) = CompDisA2;
                    mfCompensation(2,2) = CompDisA3;
                    mfCompensation(3,2) = 0;
                    break;
                }
                break;
             case stance:
                if(iStatusCounter[legNum] <= ceil(iStatusCounterBuffer[legNum][int(stance)] * PrePsotiveFactor) )   
                    switch (legNum) //the number of swing leg is only 1 in amble gait   
                    {
                    case 0:
                        mfCompensation(0,2) = 0;
                        mfCompensation(1,2) = CompDisA3;
                        mfCompensation(2,2) = CompDisA2;
                        mfCompensation(3,2) = CompDisA1;
                        break;
                    case 1:
                        mfCompensation(0,2) = CompDisA3;
                        mfCompensation(1,2) = 0;
                        mfCompensation(2,2) = CompDisA1;
                        mfCompensation(3,2) = CompDisA2;
                        break;
                    case 2:
                        mfCompensation(0,2) = CompDisA2;
                        mfCompensation(1,2) = CompDisA1;
                        mfCompensation(2,2) = 0;
                        mfCompensation(3,2) = CompDisA3;
                        break;
                    case 3:
                        mfCompensation(0,2) = CompDisA1;
                        mfCompensation(1,2) = CompDisA2;
                        mfCompensation(2,2) = CompDisA3;
                        mfCompensation(3,2) = 0;
                        break;
                    }
                break;
            }
        }
    }
    if(BSwingPhaseEndFlag == true )
    {
        BSwingPhaseEndFlag = 0;
        mfCompensation(0,2) = CompDisALL;
        mfCompensation(1,2) = CompDisALL;
        mfCompensation(2,2) = CompDisALL;
        mfCompensation(3,2) = CompDisALL;
    }

}

void CGebot::AttitudeCorrection90()
{
    int legNum;
    // float attX=-4, attZ=6, attALLx=-3;           //p
    float attX=-4*2.5/1000, attZ=6*3/1000, attALLx=-3*2.5/1000;  //adm
    if(BSwingPhaseStartFlag == true )
    {
        BSwingPhaseStartFlag = 0;
        for(legNum=0;legNum<4;legNum++)
        {
            switch(m_glLeg[legNum]->GetLegStatus()) // select swing leg
            {
            case detach:
            case swingUp:
            case swingDown:
            case attach:
                switch (legNum) //the number of swing leg is only 1 in amble gait   
                {
                case 0:
                    mfCompensation<< 0, 0, 0, 
                                    attX, 0, attZ, 
                                    attX, 0, 0, 
                                    attX, 0, 0;
                    break;
                case 1:
                    mfCompensation<< attX, 0, attZ, 
                                     0, 0, 0, 
                                    attX, 0, 0, 
                                    attX, 0, 0;
                    break;
                case 2:
                    mfCompensation<<attX, 0, 0, 
                                    attX, 0, 0, 
                                     0, 0, 0, 
                                    attX, 0, 0;
                    break;
                case 3:
                    mfCompensation<<attX, 0, 0, 
                                    attX, 0, 0, 
                                    attX, 0, 0, 
                                     0, 0, 0;
                    break;
                }
                break;
             case stance:
                if(iStatusCounter[legNum] <= ceil(iStatusCounterBuffer[legNum][int(stance)] * PrePsotiveFactor) )   
                    switch (legNum) //the number of swing leg is only 1 in amble gait   
                    {
                    case 0:
                        mfCompensation<< 0, 0, 0, 
                                        attX, 0, attZ, 
                                        attX, 0, 0, 
                                        attX, 0, 0;
                        break;
                    case 1:
                        mfCompensation<< attX, 0, attZ, 
                                         0, 0, 0, 
                                        attX, 0, 0, 
                                        attX, 0, 0;
                        break;
                    case 2:
                        mfCompensation<<attX, 0, 0, 
                                        attX, 0, 0, 
                                         0, 0, 0, 
                                        attX, 0, 0;
                        break;
                    case 3:
                        mfCompensation<<attX, 0, 0, 
                                        attX, 0, 0, 
                                        attX, 0, 0, 
                                         0, 0, 0;
                        break;
                    }
                break;
            }
        }
    }
    if(BSwingPhaseEndFlag == true )
    {
        BSwingPhaseEndFlag = 0;
        mfCompensation<<attALLx, 0, attZ/2.0, 
                        attALLx, 0, attZ/2.0, 
                        attALLx, 0, 0, 
                        attALLx, 0, 0;
    }

}
void CGebot::UpdateTouchStatus(vector<int> values,vector<int> prevalues,vector<int> preprevalues){
    for(int i=0;i<4;i++){
    if(preprevalues[i]>m_threhold[i])
        if(prevalues[i]>m_threhold[i])
            m_glLeg[i]->setTouchStatus(values[i]>m_threhold[i]);
    }
}
