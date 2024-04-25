#include"robotcontrol.h"

#define CHECK_RET(q) if((q)==false){return 0;}
CRobotControl rbt(110.0,60.0,20.0,800.0,ADMITTANCE);
bool runFlag=0;
float torque[12];

void *udpConnect(void *data)
{

	string ip = "127.0.0.1";
	uint16_t port = 8888;

	CUdpSocket srv_sock;
	//创建套接字
	CHECK_RET(srv_sock.Socket());
	//绑定地址信息
	CHECK_RET(srv_sock.Bind(ip, port));
	while(1)
	{
		//接收数据
		string buf;
		string peer_ip;
		uint16_t peer_port;
		CHECK_RET(srv_sock.Recv(&buf, &peer_ip, &peer_port));
		cout << "UpperComputer["<<peer_ip<<":"<<peer_port<<"] Command: " << buf << endl;
        //buf match command 
        int ret=commandJudge((char*)string("start").c_str(),(char *)buf.c_str());
        if(ret) {runFlag=1; goto END;}
        ret=commandJudge((char*)string("stop").c_str(),(char *)buf.c_str());
        if(ret) {runFlag=0; goto END;}
        // int ret=match((char*)string("start").c_str(),(char*)string("startsada").c_str());
        // cout<<(char*)string("start").c_str()<<endl;
        // cout<<ret<<endl;
		//发送数据
        END:
		buf.clear();
		// cout << "server say: ";
		// cin >> buf;
		// CHECK_RET(srv_sock.Send(buf, peer_ip, peer_port));
	}
	//关闭套接字
	srv_sock.Close();
	return 0;
}

void *dataSave(void *data)
{
    struct timeval startTime,endTime;
    double timeUse;
    ofstream data_IMU, data_Force, data_Torque;
    string add="../include/data_IMU.csv";
    float fAngleZero[3], fDataForce[12], fDataTorque[12];
    int status[4];

    //data_IMU.open(add, ios::app); // All outputs are attached to the end of the file.
    data_IMU.open(add);   // cover the old file
    if (data_IMU)    cout<<add<<" file open Successful"<<endl;
    else    cout<<add<<" file open FAIL"<<endl;
    data_IMU<<"Angle_pitch_roll_yaw:"<<endl;
    usleep(1e3);

    add="../include/data_Force.csv";
    data_Force.open(add);   // cover the old file
    if (data_Force)    cout<<add<<" file open Successful"<<endl;
    else    cout<<add<<" file open FAIL"<<endl;
    data_Force<<"Force_x0y0z0_x1y1z1_..._status0123:"<<endl;
    usleep(1e3);

    // add="../include/data_Torque.csv";
    // data_Torque.open(add);   // cover the old file
    // if (data_Torque)    cout<<add<<" file open Successful"<<endl;
    // else    cout<<add<<" file open FAIL"<<endl;
    // data_Torque<<"Torque_0-12:"<<endl;
    // usleep(1e3);

    while(rbt.bInitFlag == 0) //wait for initial
        usleep(1e2);

     rbt.UpdateImuData();
    for (int i = 0; i < 3; i++)
        fAngleZero[i] = rbt.api.fAngle[i];

    // WitCaliRefAngle();                               //  归零失败
    // u16 xx = rbt.api.fAngle[0] * 32768.0f / 180.0f;  
    // WitWriteReg(XREFROLL, xx); //sReg[Roll]          //  归零失败

	while(1)
	{
        if(runFlag)
        {
            gettimeofday(&startTime,NULL);
            //record data       Prevent simultaneous access to the same memory!
             rbt.UpdateImuData();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 4; j++)
                    fDataForce[i*4+j]=rbt.mfForce(i, j);  
            // for (int i = 0; i < 12; i++)
            //     fDataTorque[i]=rbt.dxlMotors.present_torque[i];
            // for (size_t i = 0; i < 4; i++)
            //     status[i]=rbt.m_glLeg[i]->GetLegStatus();
            //write data
            for (int i = 0; i < 3; i++)
            {
                data_IMU<<rbt.api.fAngle[i]-fAngleZero[i]<<",";  
                // cout<<"angle_"<<i<<": "<<rbt.api.fAngle[i]-fAngleZero[i]<<endl;
            }
            data_IMU<<endl;
            // for (int i = 0; i < 3; i++)
            //     for (int j = 0; j < 4; j++)
            //         data_Force<<rbt.mfForce(i, j)<<",";  
            // data_Force<<endl;
            // for (int i = 0; i < 12; i++)
            //     data_Torque<<rbt.dxlMotors.present_torque[i]<<",";
            //  data_Torque<<endl;

            for (size_t i = 0; i < 12; i++)
            {
                data_Force<<fDataForce[i]<<",";
                // data_Torque<<fDataTorque[i]<<",";
                // data_Torque<<torque[i]<<",";
            }
            // for (size_t i = 0; i < 4; i++)
            //      data_Force<<status[i]<<",";

            data_Force<<endl;
            // data_Torque<<endl;
                // for (size_t i = 0; i < 12; i++)
                // {
                //     cout<<torque[i]<<",";
                // }
                // cout<<endl<<endl;

            gettimeofday(&endTime,NULL);
            timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
            if(timeUse < 1e5)
                usleep(1.0/loopRateDataSave*1e6 - (double)(timeUse) - 10); 
            else
                cout<<"dataSave: "<<timeUse<<endl;
        }
	}
     data_IMU.close();data_IMU.clear();
    data_Force.close();data_Force.clear();
    // data_Torque.close();data_Torque.clear();
}


void *robotStateUpdateSend(void *data)
{
    Matrix<float,4,2> TimeForSwingPhase;
    Matrix<float, 4, 3> InitPos;
    Matrix<float, 6,1> TCV;
    TCV << VELX, 0, 0,0,0,0 ;// X, Y , alpha 
    
    
    //motors initial

#if(INIMODE==1)
    vector<float> init_Motor_angle(12);
    float float_init_Motor_angle[12];
    string2float("../include/init_Motor_angle.csv", float_init_Motor_angle);//Motor angle     d
    //cout<<"____________"<<endl;
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
            float_init_Motor_angle[i*3+j] = float_init_Motor_angle[i*3+j] * 3.1416/180; //to rad
            init_Motor_angle[i*3+j] = float_init_Motor_angle[i*3+j];      //vector
            rbt.mfJointCmdPos(i,j) = float_init_Motor_angle[i*3+j];            //rbt.forwardKinematics
            //cout<<init_Motor_angle[i*3+j]<<endl;
        }
    rbt.forwardKinematics(0);
    rbt.setInitPos(rbt.mfLegCmdPos);        //legCmdPos
    cout<<"legCmdPos:\n"<<rbt.legCmdPos<<endl ;

    motors.setPosition(init_Motor_angle);
#endif    
 
    //      rbt initial
    // TimeForStancePhase<< 0,                       TimeForGaitPeriod/2.0,     // diagonal
    //                      TimeForGaitPeriod/2.0,   TimeForGaitPeriod, 
    //                      TimeForGaitPeriod/2.0,   TimeForGaitPeriod, 
    //                      0,                       TimeForGaitPeriod/2.0;
    // TimeForSwingPhase<< TimeForGaitPeriod/4.0 *2,          TimeForGaitPeriod/4.0 *3,   // tripod
    //                      0,             TimeForGaitPeriod/4.0,
    //                      TimeForGaitPeriod/4.0 *3,    TimeForGaitPeriod,
    //                      TimeForGaitPeriod/4.0  ,          TimeForGaitPeriod/4.0 *2;
    TimeForSwingPhase<< 8*TimeForGaitPeriod/16, 	11*TimeForGaitPeriod/16,		
                        0,		 		 					3*TimeForGaitPeriod/16,		
                        12*TimeForGaitPeriod/16, 	15*TimeForGaitPeriod/16,		
                        4*TimeForGaitPeriod/16, 	7*TimeForGaitPeriod/16;
    rbt.SetPhase(TimePeriod, TimeForGaitPeriod, TimeForSwingPhase);

#if(INIMODE==2)
//    float  float_initPos[12]={    60, 60, -30,
//                                  60,-60, -30,
//                                 -60, 60, -30,
//                                 -60,-60, -30};
// 60, 60, -30,
// 60,-60, -30,
// -60, 60, -30,
// -60,-60, -30,

//  80,  50, -22,
//  80, -50, -22,
// -40,  50, -22,
// -40, -50, -23,

//  80,  55, -16,
//  80, -55, -16,
// -40,  55, -16,
// -40, -69, -18,
   float  float_initPos[12];
   string2float("../include/initPos.csv", float_initPos);//Foot end position
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
            InitPos(i, j) = float_initPos[i*3+j]/1000;
            //cout<<InitPos(i, j)<<endl;
        }
    rbt.SetInitPos(InitPos);
#endif
  
    rbt.InertiaInit();
    rbt.SetCoMVel(TCV);
    rbt.InverseKinematics(rbt.mfLegCmdPos);
    rbt.mfTargetPos = rbt.mfLegCmdPos;
  
#if(INIMODE==2)  
    rbt.SetPos(rbt.mfJointCmdPos);
    
#endif
    usleep(1e5);
    for (size_t i = 0; i < 4; i++)
        rbt.PumpNegtive(i);
    usleep(1e6);
    rbt.bInitFlag = 1;

    Matrix<float, 4, 3> mfJointTemp; 
    while(1)
    {
        if(runFlag)
        {
            struct timeval startTime,endTime;
            double timeUse;
            gettimeofday(&startTime,NULL);
            //If stay static, annotate below one line.
             rbt.NextStep();
            rbt.AirControl();
            // rbt.AttitudeCorrection90();
            
            rbt.ParaDeliver();
            
            // rbt.UpdateImuData();     // segmentation fault
            //cout<<"LegCmdPos:\n"<<rbt.mfLegCmdPos<<endl;    
            // cout<<"TargetPos:\n"<<rbt.mfTargetPos<<endl<<endl; 
            // cout<<"Compensation:\n"<<rbt.mfCompensation<<endl<<endl; 

            gettimeofday(&endTime,NULL);
            timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
            if(timeUse < 1e4)
                usleep(1.0/loopRateStateUpdateSend*1e6 - (double)(timeUse) - 10); 
            else
            cout<<"TimeRobotStateUpdateSend: "<<timeUse<<endl;
        }
    }
 
}

void *runImpCtller(void *data)
{
    struct timeval startTime,endTime;
    double timeUse;
    int run_times=0;    // for debugging

    while(rbt.bInitFlag == 0) //wait for initial
        usleep(1e2);

    rbt.dxlMotors.torqueEnable();
    while (1)
    {
        if(runFlag)
        {
            gettimeofday(&startTime,NULL);
            /* get motors data  */
            rbt.dxlMotors.getTorque();
            rbt.dxlMotors.getPosition();
            rbt.dxlMotors.getVelocity();

            /* update the data IMP need */
            rbt.UpdatejointPresPosAndVel();         
            //rbt.UpdatejointPresVel(); //useless,wrong data
            rbt.ForwardKinematics(1);
            rbt.UpdateJacobians();
            rbt.UpdateFtsPresVel();
            rbt.UpdateFtsPresForce();  
            // for (size_t i = 0; i < 12; i++)
            // {
            //     torque[i] = rbt.dxlMotors.present_torque[i];
            // }            


            /*      Admittance control     */ 
            rbt.Control();   
            rbt.InverseKinematics(rbt.mfXc);    // Admittance control

            /*      Postion control with Comp      */
            // rbt.InverseKinematics(rbt.mfTargetPos); //    Postion control

            /*      Postion control      */
            // rbt.InverseKinematics(rbt.mfLegCmdPos); 


            // cout<<"mfJointCmdPos:"<<rbt.mfJointCmdPos;
            // cout<<"mfLegCmdPos: \n"<<rbt.mfLegCmdPos<<endl;
            // cout<<"target_pos: \n"<<rbt.mfTargetPos<<endl;
            // cout<<"legPresPos: \n"<<rbt.mfLegPresPos<<"; \nxc: \n"<<rbt.xc<<endl;
            //cout<<"force:"<<endl<<rbt.mfForce.transpose()<<endl;
            // cout<<"xc_dotdot: \n"<<rbt.mfXcDotDot<<"; \nxc_dot: \n"<<rbt.mfXcDot<<"; \nxc: \n"<<rbt.mfXc<<endl;
            // cout<<endl;

            /*      Set joint angle      */
           rbt.SetPos(rbt.mfJointCmdPos);

            /*      Impedance control      */
            // for(int i=0; i<4; i++)  
            //     for(int j=0;j<3;j++)
            //         SetTorque[i*3+j] = rbt.target_torque(j,i);
            // motors.setTorque(SetTorque); 

            gettimeofday(&endTime,NULL);
            timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
            if(timeUse < 1e4)
                usleep(1.0/loopRateImpCtller*1e6 - (double)(timeUse) - 10); 
            // else
            //     cout<<"timeImpCtller: "<<timeUse<<endl;
        }
    }
  
}

int main(int argc, char ** argv)
{   
    
    
    pthread_t th1, th2, th3, th4;
	int ret;
    ret = pthread_create(&th1,NULL,udpConnect,NULL);
    if(ret != 0)
	{
		printf("create pthread1 error!\n");
		exit(1);
	}
    ret = pthread_create(&th2,NULL,robotStateUpdateSend,NULL);
    if(ret != 0)
	{
		printf("create pthread2 error!\n");
		exit(1);
	}
    ret = pthread_create(&th3,NULL,runImpCtller,NULL);
    if(ret != 0)
	{
		printf("create pthread3 error!\n");
		exit(1);
	}
    ret = pthread_create(&th4,NULL,dataSave,NULL);
    if(ret != 0)
	{
		printf("create pthread4 error!\n");
		exit(1);
	}
    
    // pthread_join(th1, NULL);
    pthread_join(th2, NULL);
    pthread_join(th3, NULL);
    // pthread_join(th4, NULL);
    while(1);

    
    
    return 0;
}