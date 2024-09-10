#include"robotcontrol.h"
#include "ADS1x15.h"
#include <iostream>
#include <string>
#include <cstdio>
#include <atomic>
#include <boost/lockfree/spsc_queue.hpp>



#define CHECK_RET(q) if((q)==false){return 0;}
CRobotControl rbt(110.0,60.0,20.0,800.0,ADMITTANCE);
DxlAPI motors("/dev/ttyAMA0", 1000000, rbt.ID, 2);
std::atomic<bool> runFlag(true);
std::atomic<float> program_run_time(0.0);
boost::lockfree::spsc_queue<vector<float>, boost::lockfree::capacity<1024>> ringBuffer_torque;
boost::lockfree::spsc_queue<Matrix<float,3,4>, boost::lockfree::capacity<1024>> ringBuffer_force;

void *dataSave(void *data)
{
    struct timeval startTime={0,0},endTime={0,0};
    double timeUse=0.0;;
    ofstream data_IMU, data_Force, data_Torque;
    string add="../include/data_IMU.csv";
    float fAngleZero[3], fDataForce[12];//fDataTorque[16];
    Matrix<float,3,4> mDataForce;
    std::vector<float> fDataTorque;
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

    add="../include/data_Torque.csv";
    data_Torque.open(add);   // cover the old file
    if (data_Torque)    cout<<add<<" file open Successful"<<endl;
    else    cout<<add<<" file open FAIL"<<endl;
    data_Torque<<"Torque_0-12:"<<endl;
    usleep(1e3);

    while(rbt.bInitFlag == 0) //wait for initial
        usleep(1e2);
     rbt.UpdateImuData();
    for (int i = 0; i < 3; i++)
        fAngleZero[i] = rbt.api.fAngle[i];
    WitCaliRefAngle();                               //  归零失败
    u16 xx = rbt.api.fAngle[0] * 32768.0f / 180.0f;  
    WitWriteReg(XREFROLL, xx); //sReg[Roll]          //  归零失败
	while(1)
	{
        if(runFlag)
        {
            gettimeofday(&startTime,NULL);
            //record data       Prevent simultaneous access to the same memory!
         //    rbt.UpdateImuData();
        //       for(int i=0; i<4;i++)
        // {
        //     cout<<" "<<rbt.m_glLeg[i]->getTouchStatus()<<" ";
        // }
        // cout<<endl;
           
           
         
            // for (int i = 0; i < 16; i++)
            //     fDataTorque[i]=motors.present_torque[i];
            ringBuffer_torque.pop(fDataTorque);
            ringBuffer_force.pop(mDataForce);
            for (int i = 0; i < 3; i++)
             for (int j = 0; j < 4; j++)
              fDataForce[i*4+j]=mDataForce(i, j);  
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
                //data_Torque<<fDataTorque[i]<<",";
                // data_Torque<<torque[i]<<",";
            }
            for(auto a:fDataTorque)
             data_Torque<<a<<",";
            // for (size_t i = 0; i < 4; i++)
            //      data_Force<<status[i]<<",";

            data_Force<<endl;
            data_Torque<<endl;
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
    // data_IMU.close();data_IMU.clear();
    data_Force.close();data_Force.clear();
    data_Torque.close();data_Torque.clear();
}


void *robotStateUpdateSend(void *data)
{
    Matrix<float,4,2> TimeForSwingPhase;
    Matrix<float, 4, 3> InitPos;
    Matrix<float, 6,1> TCV;
    TCV << VELX, 0, 0,0,0,0 ;// X, Y , alpha 
    
    
    //motors initial
    motors.setOperatingMode(3);
    motors.torqueEnable();
    motors.getPosition();
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
//if(VELX != 0)
    TimeForSwingPhase<< 8*TimeForGaitPeriod/16, 	11*TimeForGaitPeriod/16,		
                        0,		 		 					3*TimeForGaitPeriod/16,		
                        12*TimeForGaitPeriod/16, 	15*TimeForGaitPeriod/16,		
                        4*TimeForGaitPeriod/16, 	7*TimeForGaitPeriod/16;
// else 
//     TimeForSwingPhase<< 0, 	0,		
//                         0,	0,		
//                         0,  0,		
//                         0, 	0;
    rbt.SetPhase(TimePeriod, TimeForGaitPeriod, TimeForSwingPhase);

#if(INIMODE==2)
   float  float_initPos[12]={   84.0,65.5,-21.0,
                                84.0,-65.5,-21.0,
                               -84.0, 65.5,-21.0,
                               -84.0, -65.5,-21.0};
    // float  float_initPos[12]={    95,  50, -18,
    //                               85, -55, -18,
    //                               -65,  55, -22,
    //                               -70, -55, -12,
    //                                 };
// 60, 60, -30,
// 60,-60, -30,
// -60, 60, -30,
// -60,-60, -30,
// 65.5,70.0,21.0,
// 65.5,70.0,21.0,
// 65.5,84.0,21.0,
// 65.5,84.0,21.0
//  80,  50, -22,
//  80, -50, -22,
// -40,  50, -22,
// -40, -50, -23,

//  80,  55, -16,
//  80, -55, -16,
// -40,  55, -16,
// -40, -69, -18,
  // std::vector<float> float_initPos(12);
  // string2float2("../include/initPos.csv", float_initPos);//Foot end position
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
    SetPos(rbt.mfJointCmdPos,motors,rbt.vLastSetPos);
    cout<<"rbt.mfJointCmdPos:"<<rbt.mfJointCmdPos<<endl;
    rbt.UpdateJacobians();
    // float k=find_k(560.0/1000,OMEGA,Y0);
    // cout<<"k = " <<k<<endl;
    
#endif
    usleep(1e5);
    for (size_t i = 0; i < 4; i++)
        rbt.PumpNegtive(i);
    usleep(1e6);
    rbt.bInitFlag = 1;
}

void *runImpCtller(void *data)
{
    struct timeval startTime={0,0},endTime={0,0};
    double timeUse=0;
    int run_times=0;    // for debugging
    vector<vector<float>> filedata;   //3*4 * 800=2400 row

    readCSV("../include/filtered_legtrace43.csv",filedata);
    int t=200;
    while(rbt.bInitFlag == 0) //wait for initial
        usleep(1e2);
    
    //rbt.dxlMotors.torqueEnable();
    // float k=find_k(560.0/1000,OMEGA,Y0);
    // cout<<"k = " <<k<<endl;
    while (1)
    {
        if(runFlag)
        {
            gettimeofday(&startTime,NULL);
            /* get motors data  */
            // motors.getTorque();
            // ringBuffer_torque.push(motors.present_torque);
            
            // motors.getPosition();
            // motors.getVelocity();
        
            // /* update the data IMP need */
            // rbt.UpdatejointPresPosAndVel(motors.present_position);         
            // rbt.ForwardKinematics(1);
            // rbt.UpdateJacobians();
          

            // rbt.UpdateFtsPresVel();
            // rbt.UpdateFtsPresForce(motors.present_torque);  
            // ringBuffer_force.push(rbt.mfForce);
             Matrix<float,4,3> temp;
             if (t<800){
                temp<<filedata[4*t][0],filedata[4*t][1],filedata[4*t][2],
                filedata[4*t+1][0],filedata[4*t+1][1],filedata[4*t+1][2],
                filedata[4*t+2][0],filedata[4*t+2][1],filedata[4*t+2][2],
                filedata[4*t+3][0],filedata[4*t+3][1],filedata[4*t+3][2];
                cout<<"temp:"<<temp<<endl;
             }
           
            if(t>=0&&t<30) rbt.PumpPositive(1);
            else if(t>50&&t<180) rbt.PumpNegtive(1);
            else if(t>190&&t<230) rbt.PumpPositive(3);
            else if(t>250&&t<380) rbt.PumpNegtive(3);
            else if(t>390&&t<430) rbt.PumpPositive(0);
            else if(t>450&&t<580) rbt.PumpNegtive(0);
            else if(t>590&&t<630) rbt.PumpPositive(2);
            else if(t>650&&t<780) rbt.PumpNegtive(2);
            cout<<"t: "<<t<<endl;
            rbt.InverseKinematics(temp); //    Postion control

            /*      Postion control      */
             //rbt.InverseKinematics(rbt.mfLegCmdPos); 
           // cout<<"mfLegCmdPos:\n"<<rbt.mfLegCmdPos<<endl;
            //cout<<"mfJointCmdPos:\n"<<rbt.mfJointCmdPos<<endl;
            // cout<<"mfLegCmdPos: \n"<<rbt.mfLegCmdPos<<endl;
            // cout<<"target_pos: \n"<<rbt.mfTargetPos<<endl;
            // cout<<"legPresPos: \n"<<rbt.mfLegPresPos<<"; \nxc: \n"<<rbt.xc<<endl;
            //cout<<"force:"<<endl<<rbt.mfForce.transpose()<<endl;
            // cout<<"xc_dotdot: \n"<<rbt.mfXcDotDot<<"; \nxc_dot: \n"<<rbt.mfXcDot<<"; \nxc: \n"<<rbt.mfXc<<endl;
            // cout<<endl;
       
            /*      Set joint angle      */
            SetPos(rbt.mfJointCmdPos,motors,rbt.vLastSetPos);

            /*      Impedance control      */
            // for(int i=0; i<4; i++)  
            //     for(int j=0;j<3;j++)
            //         SetTorque[i*3+j] = rbt.target_torque(j,i);
            // motors.setTorque(SetTorque); 

            gettimeofday(&endTime,NULL);
            timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
            if(timeUse < 1e4)
                usleep(1.0/loopRateImpCtller*1e6 - (double)(timeUse) - 10); 
            t=t+1;
            // else
            //     cout<<"timeImpCtller: "<<timeUse<<endl;
        }
    }
  
}
std::vector<std::string> split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::stringstream ss(s);
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

void *timeUpdate(void *date)
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "创建socket失败" << std::endl;
        return NULL;
    }

    // 配置从机地址和端口
    struct sockaddr_in client_addr;
    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_addr.s_addr = inet_addr("192.168.137.88"); // 从机的IP地址
    client_addr.sin_port = htons(65432); // 和主机程序发送数据的端口一致

    // 绑定socket
    if (bind(sockfd, (struct sockaddr*)&client_addr, sizeof(client_addr)) < 0) {
        std::cerr << "绑定socket失败" << std::endl;
        close(sockfd);
        return NULL;
    }

    char buffer[200];
    while(rbt.bInitFlag == 0) //wait for initial
        usleep(1e2);
    while (true) {
        // 接收数据
        ssize_t recv_len = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
        if (recv_len < 0) {
            std::cerr << "接收数据失败" << std::endl;
            close(sockfd);
            return NULL;
        }

        buffer[recv_len] = '\0';

        //获取此时当前时间
        struct timeval  startTime2;
        double timeUse2;
        gettimeofday(&startTime2,NULL);

        char STtime2[50];
        snprintf(STtime2,sizeof(STtime2),"%1d,%1d",startTime2.tv_sec,startTime2.tv_usec);
        std::cout << "startTime2: " << STtime2 << std::endl;

        //解析接受到的数据
        double program_run_time_vibration;
        char STtime[50];
        sscanf(buffer, "%lf,%s", &program_run_time_vibration,STtime);
        std::cout << "timeuse1: " << program_run_time_vibration << std::endl;
        std::cout << "STtime: " << STtime << std::endl;
        
        //将转换成字符串格式的时间重新解析
        // 解析秒数和微秒数
        char delimiter = ',';

    // 拆分字符串
     std::vector<std::string> tokens = split(STtime, delimiter);

    // 输出结果
        for (const auto &token : tokens) {
            std::cout << token << std::endl;
        }


        std::cout << "startTime2.tv_sec: " << startTime2.tv_sec << std::endl;
        std::cout << "startTime2.tv_usec: " << startTime2.tv_usec << std::endl;
        std::cout << "program_run_time_vibration: " << program_run_time_vibration << std::endl;

        //计算传输延迟
        timeUse2 = 1e6*(startTime2.tv_sec - stod(tokens[0])) + startTime2.tv_usec - stod(tokens[1]); 

        //计算接受程序运行时间
        double adjusted_run_time = program_run_time_vibration + timeUse2*1e-06;
        std::cout << "传输延迟: " << timeUse2 << " 微秒" << std::endl;
        std::cout << "接收时的程序运行时间: " << adjusted_run_time << " 微秒" << std::endl;
        std::cout << "------------------------------------------------" << std::endl;
        program_run_time.store(adjusted_run_time); 
        if(fabs(program_run_time.load()-0.5)<0.1)   runFlag.store(true);
    }

    close(sockfd);
    return 0;


}
int main(int argc, char ** argv)
{   
    
    
    pthread_t th1, th2, th3, th4,th5,th6;
	int ret;
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
     ret = pthread_create(&th6,NULL,timeUpdate,NULL);
    if(ret != 0)
	{
		printf("create pthread6 error!\n");
		exit(1);
	}
    pthread_join(th1, NULL);
    pthread_join(th2, NULL);
    pthread_join(th3, NULL);
    pthread_join(th4, NULL);
    //pthread_join(th5, NULL);
    while(1);

    
    
    return 0;
}