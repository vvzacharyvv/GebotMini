#include "robotcontrol.h"
using namespace Eigen;
void CRobotControl::Init(){
    float impdata[200];
    string2float2("../include/adm_parameter.csv",impdata);   // 0-swing,1-attach,2-stance,3-detach
    for(int i=0; i<4; i++)
    {
    Map<Matrix<float, 4, 3, RowMajor>> mapK(impdata + 36 * i), mapB(impdata + 12 + 36 * i), mapM(impdata  + 24 + 36 * i);
    ChangePara(mapK,mapB,mapM, i);
    } 
    mfXcDotDot.setZero();
    mfXcDot.setZero();
    mfXc.setZero();
    mfTargetPos.setZero();
    mfTargetVel.setZero();
    mfTargetAcc.setZero();
    mfTargetForce.setZero();
    mfLastForce.setZero();
    fCtlRate = 100;
    // cal inertial
    m_fIxx=((m_fLength*m_fLength)+(m_fHeight*m_fHeight))*m_fMass/12;
}
/**
 * @brief Update imu data.
 *  api.fAcc, fGyro, fAngle, Pitch Roll Yaw  in the coordinate system of the robot (Y X Z)
 */
void CRobotControl::UpdateImuData()
{
    api.updateIMU();
     vfVmcOmegaBase<<api.fGyro[0],api.fGyro[1],api.fGyro[2];
     Matrix<float,3,3> rotateX,rotateY,rotateZ,rotateR;
     for (size_t i = 0; i < 3; i++)
     {
        api.fAngle[i]=api.fAngle[i]/180*PI;
     }
     
     Matrix<float,3,1> gravity;
      gravity<<0,0,9.8;
    rotateX<<1,0,0,
            0,cos(api.fAngle[0]),-sin(api.fAngle[0]),
            0,sin(api.fAngle[0]),cos(api.fAngle[0]);
    rotateY<<cos(api.fAngle[1]),0,sin(api.fAngle[1]),
            0,1,0,
            -sin(api.fAngle[1]),0,cos(api.fAngle[1]);
    rotateZ<<cos(api.fAngle[2]),-sin(api.fAngle[2]),0,
            sin(api.fAngle[2]),cos(api.fAngle[2]),0,
            0,0,1;
    rotateR=rotateZ*rotateY*rotateX;
    vfGravity=rotateR.inverse()*gravity;
    // something wrong with the rotate Matrix,maybe imu use ypr, swap x and y to get right gravity;
    float temp;
    temp=vfGravity[0];
    vfGravity[0]=vfGravity[1];
    vfGravity[1]=temp;
    
    // cout<<"vfGravity: "<<vfGravity<<endl;
    //cout<<"api.fAngle ";
    // for (size_t i = 0; i < 3; i++)
    // {
    //     cout<<api.fAngle[i]<<" ";
    // }
    //cout<<endl;
    
}

void CRobotControl::UpdateFtsPresForce(vector<float> present_torque)
{
    
    Matrix<float, 4, 4> temp;
    if(mfForce(2,3) - mfLastForce(2,3) > 0.3 || mfForce(2,3) - mfLastForce(2,3) < -0.3)
        temp.setZero();
    temp=inverseMotorMapping(present_torque);
    // for(int i=0; i<4; i++)
    //     for(int j=0;j<4;j++)
    //         temp(i ,j ) = present_torque[i+j*4];
    // cout<<"temp: "<<temp<<endl;
    // cout<<"pinv= "<<pinv(m_glLeg[1]->GetJacobian().transpose(),8e-6)<<" "<<endl;

    for (int i=0; i<4; i++)
    {
        mfForce.col(i) = ForceLPF * mfLastForce.col(i) + (1-ForceLPF) *pinv(m_glLeg[i]->GetJacobian().transpose(),8e-6) * temp.row(i).transpose();
        // cout<<m_glLeg[i]->GetJacobian().transpose().inverse() <<" ";
    }  
    static int t=0;
    t++;
    if(t%16==0){
        cout<<"RF= "<<mfForce.col(1).transpose()<<endl;
        cout<<"LF= "<<mfForce.col(0).transpose()<<endl;
        t=1;
    }
        
    mfLastForce = mfForce;
}

void CRobotControl::UpdateTargTor(Matrix<float, 3, 4> force)
{
    for (int legNum=0; legNum<4; legNum++)
        mfTargetTor.col(legNum) = m_glLeg[legNum]->GetJacobian().transpose() * force.col(legNum); 
}

/**
 * @brief Transfer data to Force Control, including TargetForce, TargetPos.
 *  
 */
void CRobotControl::ParaDeliver()
{
    mfTargetPos = mfLegCmdPos+ mfCompensation;
    #ifdef  VMCCONTROL
    CalVmcCom();
        for(uint8_t legNum=0; legNum<4; legNum++) //{detach=0, swingUp=1, swingDown=2, attach=3, recover=4, stance=5}; 
    {   
        switch(m_glLeg[legNum]->GetLegStatus())
        {
            case 1: mfTargetForce.row(legNum) << 0, 0, 0;        //swing
            break;
            case 2: mfTargetForce.row(legNum) << 0, 0, 1.5;      //detach
            break;
            case 3: mfTargetForce.row(legNum) << 0, 0, -1.6;     //attach
            break;
            default:
            break;
        }
    }
    #else
    
    for(uint8_t legNum=0; legNum<4; legNum++) //{detach=0, swingUp=1, swingDown=2, attach=3, recover=4, stance=5}; 
    {   
        float k=6.5,d=1;
        float force = k*(mfLegCmdPos(legNum,2)-mfLegPresPos(legNum,2))+d*(mfLegPresPos(legNum,2) - mfLegLastPos(legNum,2));
        switch(m_glLeg[legNum]->GetLegStatus())
        {
            case 0: mfTargetForce.row(legNum) << 0, 0, 0 ; //-2.0, 0, -1.6;//-0.6, 0, -1.6;   //stance
            break;
            case 1: mfTargetForce.row(legNum) << 0, 0, 0;        //swing
            break;
            case 2: mfTargetForce.row(legNum) << 0, 0, 1.5;      //detach
            break;
            case 3: mfTargetForce.row(legNum) << 0, 0, -2.0;     //attach
            break;
        }
    }
    #endif
}

/**
 * @brief 
 * The parameters in .csv must conform to the order of 'mode'.
 * @param mK 
 * @param mB 
 * @param mM 
 * @param mode the order of parameter: 0-stance, 1-swing, 2-detach, 3-attach
 */
void CRobotControl::ChangePara(Matrix<float, 4, 3> mK, Matrix<float, 4, 3> mB, Matrix<float, 4, 3> mM,int mode)
{
        switch(mode) 
    {
        case 0:
            mfKstance = mK; mfBstance = mB; mfMstance = mM;
            break;
        case 1:
            mfKswing = mK; mfBwing = mB; mfMswing = mM;
            break;
        case 2:
            mfKdetach = mK; mfBdetach = mB; mfMdetach = mM;
            break;
        case 3:
            mfKattach = mK; mfBattach = mB; mfMattach = mM;   
            break; 
    }
}

void CRobotControl::VibrationControl_quad(float k, float c,float m)
{
      c=2*m*sqrt(k/m); // xi =1
      cout<<"c"<<c<<endl;
      float a=0.1,b=1;
      mfTargetForce.col(2)<<0.02,0.02,0.02,0.02;
      mfForce(2,2)/=10;
      cout<<"mfForce: "<<mfForce<<endl;
      mfXcDotDot.col(2)=mfTargetAcc.col(2)+1*a/m*(b*k/4*(-mfLegPresPos.col(2)+mfTargetPos.col(2))+c/4*(-mfLegPresVel.col(2)+mfTargetVel.col(2))
                        +( mfTargetForce.col(2) - mfForce.transpose().col(2)));
      cout<<"mfTargetForce.col(2) - mfForce.transpose().col(2):"<<mfTargetForce.col(2) - mfForce.transpose().col(2)<<endl;
      cout<<"c/4*(-mfLegPresVel.col(2)+mfTargetVel.col(2))"<<c/4*(-mfLegPresVel.col(2)+mfTargetVel.col(2))<<endl;
      cout<<"-mfLegPresPos.col(2)+mfTargetPos.col(2)"<<-mfLegPresPos.col(2)+mfTargetPos.col(2)<<endl;
      mfXcDot =  mfLegPresVel + mfXcDotDot * (1/fCtlRate);
      for(int i=0;i<4;i++)
      mfLegCmdPos(i,2) = mfLegPresPos(i,2);
      mfXc = mfLegCmdPos + (mfXcDot * (1/fCtlRate));             
}

void CRobotControl::Control()
{
    mfTargetForce.col(2)= mfTargetForce.col(2)/10;
    if(m_eControlMode == IMPEDANCE)  // Impedance control
    {
        mfXcDot = (mfLegPresPos - mfLegLastPos) * fCtlRate;
        mfXcDotDot = (mfLegPresVel - mfXcDot) * fCtlRate;
        for(uint8_t legNum=0; legNum<4; legNum++)
        {
            mfForce.transpose().row(legNum) = mfTargetForce.row(legNum)  
            + mfKstance.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))
            + mfBstance.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum)) ;
            // + mfMstance.row(legNum).cwiseProduct(mfTargetAcc.row(legNum) - mfXcDotDot.row(legNum));
            // cout<<"legNum_"<<(int)legNum<<":"<<m_glLeg[legNum]->GetLegStatus()<<endl;
            // cout<<"K__stance_"<<(int)legNum<<"  "<<mfKstance.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))<<endl;
            // cout<<"B__stance_"<<(int)legNum<<"  "<<mfBstance.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum))<<endl;
            // cout<<"mfForce"<<mfForce.transpose().row(legNum)<<endl;
        }
        UpdateTargTor(mfForce);
        // cout<<"mfTargetPos:"<<endl<<mfTargetPos<<endl;
        // cout<<"mfLegPresPos:"<<endl<<mfLegPresPos<<endl;
        // cout<<"mfForce:"<<endl<<mfForce<<endl;
        // cout<<"mfTargetTor:"<<endl<<mfTargetTor<<endl<<endl;
    }
    else if(m_eControlMode == ADMITTANCE)  // Admittance control    mfXcDotDot = 0 + M/-1*( - footForce[i][0] + refForce[i] - B * (pstFootVel[i][0] - 0) - K * (pstFootPos[i][0] - targetFootPos(i,0)));
    {
        for(uint8_t legNum=0; legNum<4; legNum++)
        {   
            switch(m_glLeg[legNum]->GetLegStatus())
            {
                case stance:  //stance
                    
                case recover: // or recover
                    mfXcDotDot.row(legNum) =  mfTargetAcc.row(legNum) 
                    + mfKstance.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))
                    + mfBstance.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum)) 
                    + mfMstance.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) );
                    // cout<<"K__stance_"<<(int)legNum<<"  "<<mfKstance.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))<<endl;
                    // cout<<"B__stance_"<<(int)legNum<<"  "<<mfBstance.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum))<<endl;
                    // cout<<"M__stance_"<<(int)legNum<<"  "<<mfMstance.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) )<<endl;
                    break;
                case swingUp:
                case swingDown: //swing
                    mfXcDotDot.row(legNum) =  mfTargetAcc.row(legNum) 
                    + mfKswing.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))
                    + mfBwing.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum)) 
                    + mfMswing.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) );

                    break;

                case detach: //detach
                    mfXcDotDot.row(legNum) =  mfTargetAcc.row(legNum) 
                    + mfKdetach.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))
                    + mfBdetach.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum))   
                    + mfMdetach.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) );
                    // cout<<"K__detach_"<<(int)legNum<<"  "<<mfKdetach.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))<<endl;
                    // cout<<"B__detach_"<<(int)legNum<<"  "<<mfBdetach.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum))<<endl;
                    // cout<<"M__detach_"<<(int)legNum<<"  "<<mfMdetach.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) )<<endl;
                    break;

                case attach: //attach
                    mfXcDotDot.row(legNum) =  mfTargetAcc.row(legNum) 
                    + mfKattach.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))
                    + mfBattach.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum)) 
                    + mfMattach.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) );
                    // cout<<"K__attach_"<<(int)legNum<<"  "<<mfKattach.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))<<endl;
                    // cout<<"B__attach_"<<(int)legNum<<"  "<<mfBattach.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum))<<endl;
                    // cout<<"M__attach_"<<(int)legNum<<"  "<<mfMattach.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) )<<endl<<endl;                 
                    break;
            }
            // cout<<stepFlag[legNum]<<endl;
            //  mfXcDotDot.row(legNum) =  mfTargetAcc.row(legNum) 
            // + mfKattach.row(legNum).cwiseProduct(mfTargetPos.row(legNum) - mfLegPresPos.row(legNum))
            // + mfBattach.row(legNum).cwiseProduct(mfTargetVel.row(legNum) - mfLegPresVel.row(legNum)) 
            // + mfMattach.row(legNum).cwiseInverse().cwiseProduct( mfTargetForce.row(legNum) - mfForce.transpose().row(legNum) );
        }
        mfXcDot =  mfLegPresVel + mfXcDotDot * (1/fCtlRate);
        mfXc =  mfLegPresPos + (mfXcDot * (1/fCtlRate));
        
        // cout<<"mfForce_\n"<<mfForce.transpose()<<endl;
    }

}
void CRobotControl::CalTargetForce(){
    int stanceCount=0;
    std::vector<int> stanceLegNum;
    for(int legNum=0;legNum<4;legNum++)
    {
        if(m_glLeg[legNum]->GetLegStatus()==stance||m_glLeg[legNum]->GetLegStatus()==recover)
        { 
        stanceLegNum.push_back(legNum);
        }

    }
    stanceCount=(int)stanceLegNum.size();
    for(int i=0;i<stanceCount;i++)
        mfTargetForce.row(stanceLegNum[i])<<mfOutForce.row(stanceLegNum[i]);
    cout<<"mfTargetForce"<<mfTargetForce<<endl;
}
void CRobotControl::CalGravity(){
    Matrix<float,3,3> mfTempASM; 
    bool isOk=1;
    int stanceCount=0;
    int stanceUsefulCount=0;
    std::vector<int> stanceLegNum;
    Matrix<float,3,1> stanceAccumlate;
    stanceAccumlate.setZero();
    for(int legNum=0;legNum<4;legNum++)
    {
        if(m_glLeg[legNum]->GetLegStatus()==stance||m_glLeg[legNum]->GetLegStatus()==recover)
        { 
        stanceLegNum.push_back(legNum);
        }

    }
    stanceCount=(int)stanceLegNum.size();
    vfGravity<<0,0,-9.8;
    vfVmcb61<<m_fMass*(vfGravity),0,0,0;
    //cout<<"vfVmcb61:"<<vfVmcb61<<endl;

    int k=3*stanceCount;
    mfTempASM.setZero();
    mfVmcS66.setIdentity(6,6);
    mfVmcS66=mfVmcS66*100;
    mfVmcW.setIdentity(k,k);
    for(int i=0; i<4; i++)
    {
        mfFtsPosASM.push_back(mfTempASM);
    }
    for(int i=0; i<4; i++)
    {
    mfTempASM << 0, -mfLegCmdPos(i, 2), mfLegCmdPos(i, 1), mfLegCmdPos(i, 2), 0, -mfLegCmdPos(i, 0), -mfLegCmdPos(i, 1), mfLegCmdPos(i, 0), 0;
    mfFtsPosASM[i] = mfTempASM;
    }
    mfVmcA.resize(6,k);
    for(int i=0;i<stanceCount;i++)
    {
        mfVmcA.block<3,3>(0,i*3)<<MatrixXf::Identity(3, 3);
        mfVmcA.block<3,3>(3,i*3)<<mfFtsPosASM[stanceLegNum[i]];
    }
    mfVmcH.resize(k,k);
    vfVmcg3c1.resize(k,1);
    mfVmcH=2*mfVmcA.transpose()*mfVmcS66*mfVmcA+2*mfVmcW;
    vfVmcg3c1=-2*mfVmcA.transpose()*mfVmcS66*vfVmcb61;
    // cout<<"mfVmcA:"<<mfVmcA<<endl;
    // cout<<"mfVmcS66:"<<mfVmcS66<<endl;
    // cout<<"vfVmcb61:"<<vfVmcb61<<endl;
    // cout<<"k="<<k<<endl;
    //qp
    qpOASES::Options options;
    options.initialStatusBounds = qpOASES::ST_INACTIVE;
	options.numRefinementSteps = 3;
	options.enableCholeskyRefactorisation = 1;

    qpOASES::QProblemB qpPrograme(k);
    qpPrograme.setOptions(options);
    qpOASES::real_t qp_H[k*k]={};
    qpOASES::real_t qp_g[k]={};
    // qpOASES::real_t lb[6]={-200,-200,-200,-200,-200,-200};
    // qpOASES::real_t ub[6]={200,200,200,200,200,200};
    qpOASES::real_t lb[k]={};
    qpOASES::real_t ub[k]={};
    
        for(int i=0; i<k; i++)
    {
        qp_g[i] = vfVmcg3c1(i,0);
        lb[i]=-200;
        ub[i]=200;
        for(int j=0; j<k; j++)
        {
        qp_H[i*k+j] = mfVmcH(i,j);
        }
    }
    qpOASES::int_t nWSR = 10;
//     cout<<"H:";
//    for(int i=0; i<k; i++)W
//    {
//     for(int j=0; j<k; j++)
//     {
//     cout<<qp_H[k*i+j]<<" ";
//     }
//     cout<<endl;
//    }
//    cout<<"g:";
//    for(int i=0; i<k; i++)
//    {
//          cout<<qp_g[i]<<" ";
//    }
//     cout<<endl;
//     cout<<"ub";
//    for(int i=0; i<k; i++)
//    {
//          cout<<ub[i]<<" ";
//    }
//     cout<<"lb";
//      cout<<endl;
//    for(int i=0; i<k; i++)
//    {
//          cout<<lb[i]<<" ";
//    }
//     cout<<endl;
    qpPrograme.init(qp_H,qp_g,lb,ub,nWSR,0);
    qpOASES::real_t xOpt[k];
  
	qpPrograme.getPrimalSolution( xOpt );
    cout<<"xopt:"<<endl;  
    for(int i=0;i<3*stanceCount;i+=3)
    {
       mfGravityForce.row(stanceLegNum[i/3])<<xOpt[i],xOpt[i+1],xOpt[i+2];
       cout<<"legNum:"<<stanceLegNum[i/3]<<" "<<xOpt[i]<<" "<<xOpt[i+1]<<" "<<xOpt[i+2]<<endl;
    }
    cout<<endl;

  

}
void CRobotControl::CalSpringForce(){
        float K[4]={1,1,1,1};
        float D[4]={1,1,1,1};
        int stanceCount=0;
        std::vector<int> stanceLegNum;
        for(int legNum=0;legNum<4;legNum++)
        {
            if(m_glLeg[legNum]->GetLegStatus()==stance||m_glLeg[legNum]->GetLegStatus()==recover)
            { 
            stanceLegNum.push_back(legNum);
            }

        }
        stanceCount=(int)stanceLegNum.size();
        Matrix<float, 1,3> TCV;
        TCV<<VELX,0,0;
        for(int i=0;i<stanceCount;i++){//
            Matrix<float, 1,3> force=K[stanceLegNum[i]]*(mfLegCmdPos.row(stanceLegNum[i])-mfLegPresPos.row(stanceLegNum[i]))+D[stanceLegNum[i]]*(TCV-mfLegPresVel.row(stanceLegNum[i]));
            mfSpringForce.row(stanceLegNum[i])<<force;
        cout<<"mfSpringForce"<<mfSpringForce<<endl;
        }
   }
    void CRobotControl::CaloutForce(){
        mfOutForce=mfGravityForce+mfSpringForce;

    }
#ifdef  VMCCONTROL
void CRobotControl::CalVmcCom()
{
    Matrix<float,3,3> mfTempASM; 
    bool isOk=1;
    int stanceCount=0;
    int stanceUsefulCount=0;
    std::vector<int> stanceLegNum;
    Matrix<float,3,1> stanceAccumlate;
    stanceAccumlate.setZero();
    for(int legNum=0;legNum<4;legNum++)
    {
        if(m_glLeg[legNum]->GetLegStatus()==stance)
        { 
        stanceLegNum.push_back(legNum);
            for(int i=0;i<3;i++)
            {
                //cout<<"legPre"<<legNum<<" 's "<<i<<": "<<mfLegPresVel.row(legNum)[i]<<"  "<<"legLast"<<legNum<<" 's "<<i<<": "<<mfLegLastVel.row(legNum)[i]<<"  ";
                //cout<<endl;
                if((mfLegPresVel.row(legNum)[i]-mfLegLastVel.row(legNum)[i])/mfLegLastVel.row(legNum)[i]>0.3||(mfLegPresVel.row(legNum)[i]-mfLegLastVel.row(legNum)[i])/mfLegLastVel.row(legNum)[i]<-0.3)
                isOk=0;
                if(i==2)
                    {
                        if(isOk)
                        {
                        stanceAccumlate+=mfLegPresVel.row(legNum).transpose();
                        stanceUsefulCount++;
                        }
                        isOk=1;
                    }
                    
            }
        }

    }
    mfVmcKdcom<<20,0,0,0,0.2,0,0,0,0;
    mfVmcKdbase<<200,0,0,0,200,0,0,0,200;
    Matrix<float,3,1> targetComVelxyz;
    targetComVelxyz=vfTargetCoMVelocity.block(0,0,3,1);
    if(stanceUsefulCount==0)
    {
        vfPresentCoMVelocity.block(0,0,3,1)= vfLastCoMVelocity.block(0,0,3,1);
    }
    else  
    {
    vfPresentCoMVelocity.block(0,0,3,1)=-stanceAccumlate/stanceUsefulCount;
    vfLastCoMVelocity.block(0,0,3,1)= vfPresentCoMVelocity.block(0,0,3,1);
    }
    //cout<<" vfPresentCoMVelocity: "<< vfPresentCoMVelocity.block(0,0,3,1)<<endl;
    // cout<<"mfLegPresVel:"<<mfLegPresVel<<endl;
    //  cout<<"mfLegLastVel:"<<mfLegLastVel<<endl;
    //   cout<<"stanceAccumlate:"<<stanceAccumlate<<endl;
    //   cout<<"stanceUsefulCount:"<<stanceUsefulCount<<endl;
    vfVmcAccDCom=mfVmcKdcom*(targetComVelxyz-vfPresentCoMVelocity.block(0,0,3,1));

    // cout<<"mfVmcKdcom:"<<mfVmcKdcom<<endl;
    // cout<<"targetComVelxyz:"<<targetComVelxyz<<endl;
    //  cout<<"vfPresentCoMVelocity:"<<vfPresentCoMVelocity<<endl;

    vector<int>::size_type tempSize=stanceLegNum.size();
    stanceCount=(int)tempSize;
    vfVmcOmegaBase=vfTargetCoMVelocity.block(3,0,3,1);
   // UpdateImuData();
    vfVmcAngelAccDBase=mfVmcKdbase*(vfVmcOmegaDBase-vfVmcOmegaBase);//cal angelAcc


    vfVmcb61<<m_fMass*(vfVmcAccDCom+vfGravity),m_fIxx*vfVmcAngelAccDBase[0],m_fIyy*vfVmcAngelAccDBase[1],m_fIzz*vfVmcAngelAccDBase[2];
    // cout<<"m_fMass:"<<m_fMass<<endl;
    //cout<<"vfVmcAccDCom:"<<vfVmcAccDCom<<endl;
    //cout<<"vfGravity:"<<vfGravity<<endl;

    int k=3*stanceCount;
    mfTempASM.setZero();
    mfVmcS66.setIdentity(6,6);
    mfVmcS66=mfVmcS66*100;
    mfVmcW.setIdentity(k,k);
    for(int i=0; i<4; i++)
    {
        mfFtsPosASM.push_back(mfTempASM);
    }
    for(int i=0; i<4; i++)
    {
    mfTempASM << 0, -mfLegCmdPos(i, 2), mfLegCmdPos(i, 1), mfLegCmdPos(i, 2), 0, -mfLegCmdPos(i, 0), -mfLegCmdPos(i, 1), mfLegCmdPos(i, 0), 0;
    mfFtsPosASM[i] = mfTempASM;
    }
    mfVmcA.resize(6,k);
    for(int i=0;i<stanceCount;i++)
    {
        mfVmcA.block<3,3>(0,i*3)<<MatrixXf::Identity(3, 3);
        mfVmcA.block<3,3>(3,i*3)<<mfFtsPosASM[stanceLegNum[i]];
    }
    mfVmcH.resize(k,k);
    vfVmcg3c1.resize(k,1);
    mfVmcH=2*mfVmcA.transpose()*mfVmcS66*mfVmcA+2*mfVmcW;
    vfVmcg3c1=-2*mfVmcA.transpose()*mfVmcS66*vfVmcb61;
    // cout<<"mfVmcA:"<<mfVmcA<<endl;
    // cout<<"mfVmcS66:"<<mfVmcS66<<endl;
    // cout<<"vfVmcb61:"<<vfVmcb61<<endl;
    // cout<<"k="<<k<<endl;
    //qp
    qpOASES::Options options;
    options.initialStatusBounds = qpOASES::ST_INACTIVE;
	options.numRefinementSteps = 3;
	options.enableCholeskyRefactorisation = 1;

    qpOASES::QProblemB qpPrograme(k);
    qpPrograme.setOptions(options);
    qpOASES::real_t qp_H[k*k]={};
    qpOASES::real_t qp_g[k]={};
    // qpOASES::real_t lb[6]={-200,-200,-200,-200,-200,-200};
    // qpOASES::real_t ub[6]={200,200,200,200,200,200};
    qpOASES::real_t lb[k]={};
    qpOASES::real_t ub[k]={};
    
        for(int i=0; i<k; i++)
    {
        qp_g[i] = vfVmcg3c1(i,0);
        lb[i]=-200;
        ub[i]=200;
        for(int j=0; j<k; j++)
        {
        qp_H[i*k+j] = mfVmcH(i,j);
        }
    }
    qpOASES::int_t nWSR = 10;
//     cout<<"H:";
//    for(int i=0; i<k; i++)W
//    {
//     for(int j=0; j<k; j++)
//     {
//     cout<<qp_H[k*i+j]<<" ";
//     }
//     cout<<endl;
//    }
//    cout<<"g:";
//    for(int i=0; i<k; i++)
//    {
//          cout<<qp_g[i]<<" ";
//    }
//     cout<<endl;
//     cout<<"ub";
//    for(int i=0; i<k; i++)
//    {
//          cout<<ub[i]<<" ";
//    }
//     cout<<"lb";
//      cout<<endl;
//    for(int i=0; i<k; i++)
//    {
//          cout<<lb[i]<<" ";
//    }
//     cout<<endl;
    qpPrograme.init(qp_H,qp_g,lb,ub,nWSR,0);
    qpOASES::real_t xOpt[k];
  
	qpPrograme.getPrimalSolution( xOpt );

    for(int i=0;i<3*stanceCount;i+=3)
    {
        mfTargetForce.row(stanceLegNum[i/3])<<xOpt[i],xOpt[i+1],xOpt[i+2];
    } 
    // cout<<"xopt:"<<endl;  
    // for(int i=0;i<3*stanceCount;i+=3)
    // {
    //    cout<<xOpt[i]<<" "<<xOpt[i+1]<<" "<<xOpt[i+2]<<endl;
    // }
    // cout<<endl;

    // cout<<"stanceLegNum: ";
    //  for (size_t i = 0; i < stanceLegNum.size(); i++)
    //  {
    //     cout<<stanceLegNum[i]<<" ";
    //  }
    //  cout<<endl;
     
   // cout<<"mfTargetForce:\n"<<mfTargetForce<<endl;
}
#endif
