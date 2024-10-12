
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
// std::atomic<bool> runFlag(true); 
bool runFlag = 1;
std::atomic<float> program_run_time(0.0);
boost::lockfree::spsc_queue<vector<float>, boost::lockfree::capacity<1024>> ringBuffer_torque;
boost::lockfree::spsc_queue<Matrix<float,3,4>, boost::lockfree::capacity<1024>> ringBuffer_force;


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
        ret=commandJudge((char*)string("pumpPositive").c_str(),(char *)buf.c_str());
        if(ret) {rbt.PumpAllPositve(); goto END;}
        ret=commandJudge((char*)string("pumpNegative").c_str(),(char *)buf.c_str());
        if(ret) {rbt.PumpAllNegtive(); goto END;}
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
    struct timeval startTime={0,0},endTime={0,0};
    double timeUse=0.0;;
    ofstream data_IMU, data_Force, data_Torque;
    string add="../include/data_IMU.csv";
    float fAngleZero[3], fDataForce[12];//fDataTorque[16];
    Matrix<float,3,4> mDataForce;
    std::vector<float> fDataTorque;
    int status[4];

    //data_IMU.open(add, ios::app); // All outputs are attached to the end of the file.
    // data_IMU.open(add);   // cover the old file
    // if (data_IMU)    cout<<add<<" file open Successful"<<endl;
    // else    cout<<add<<" file open FAIL"<<endl;
    // data_IMU<<"Angle_pitch_roll_yaw:"<<endl;
    // usleep(1e3);

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
    //  rbt.UpdateImuData();
    // for (int i = 0; i < 3; i++)
    //     fAngleZero[i] = rbt.api.fAngle[i];
    // WitCaliRefAngle();                               //  归零失败
    // u16 xx = rbt.api.fAngle[0] * 32768.0f / 180.0f;  
    // WitWriteReg(XREFROLL, xx); //sReg[Roll]          //  归零失败
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
            // for (int i = 0; i < 3; i++)
            // {
            //     data_IMU<<rbt.api.fAngle[i]-fAngleZero[i]<<",";  
            //     // cout<<"angle_"<<i<<": "<<rbt.api.fAngle[i]-fAngleZero[i]<<endl;
            // }
            // data_IMU<<endl;
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
    vector<float>initialMotorVel(16,80);
    vector<int>initialMotorAcc(16,80);
    TCV << VELX, 0, 0,0,0,0 ;// X, Y , alpha 
    
    
    //motors initial
    motors.setOperatingMode(3);
    motors.torqueEnable();
    motors.setVelocity(initialMotorVel);
    motors.setAcceleration(initialMotorAcc);
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
    rbt.UpdateJacobians();
    // float k=find_k(560.0/1000,OMEGA,Y0);
    // cout<<"k = " <<k<<endl;
    
#endif
    usleep(1e5);
    for (size_t i = 0; i < 4; i++)
        //rbt.PumpNegtive(i);
        rbt.PumpAllPositve();
    usleep(1e6);
    rbt.bInitFlag = 1;
    static float t=0.0;
   // std::string filename = "../include/recoverinswing.csv";
    // ofstream legcmdpos;
    //   legcmdpos.open(filename);   // cover the old file
    // if (legcmdpos)    cout<<filename<<" file open Successful"<<endl;
    // else    cout<<filename<<" file open FAIL"<<endl;
    usleep(1e3);
    while(1)
    {
        if(runFlag){
            struct timeval startTime={0,0},endTime={0,0};
            double timeUse=0.0;
            gettimeofday(&startTime,NULL);
            //If stay static, annotate below one line.
            // if(rbt.runTimes==0){
          
            rbt.NextStep();
            

            //     for(int i=0;i<4;i++){
            //         for(int j=0;j<3;j++)
            //         {
            //             legcmdpos<<rbt.mfLegCmdPos(i,j)<<",";
            //         }
            //         legcmdpos<<endl;
            //     }
            //     cout<<"rbt.legcmdpos:"<<rbt.mfLegCmdPos<<endl;
            // }
            // else{
            //            cout<<"write finished"<<endl;
            //            legcmdpos.close();legcmdpos.clear();
            //            break;
            // }
             
           rbt.AirControl();

           // rbt.AttitudeCorrection180();
            /*******************vibration*************/
            

            // // 使用t
            // t=vibration_time;    
            // // 重置标志
            // data_ready = false;
            t=program_run_time.load()-0.5; //0.5 to equal the sin curve
           // cout<<"t"<<t<<endl;
           float z=quadSprings(t,OMEGA,Y0);
            cout<<"z:"<<z<<endl;
            /************************************/
            rbt.mfCompensation<<0,0,z,
                                0,0,z,
                                0,0,z,
                                0,0,z;

             rbt.ParaDeliver();
            
            // //cout<<"LegCmdPos:\n"<<rbt.mfLegCmdPos<<endl;    
            // cout<<"t: "<<t<<endl;
             cout<<"TargetPos:\n"<<rbt.mfTargetPos<<endl<<endl; 
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
    struct timeval startTime={0,0},endTime={0,0};
    double timeUse=0;
    int run_times=0;    // for debugging

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
            motors.getTorque();
            ringBuffer_torque.push(motors.present_torque);
            
            motors.getPosition();
            motors.getVelocity();
        
            /* update the data IMP need */
            rbt.UpdatejointPresPosAndVel(motors.present_position);         
            rbt.ForwardKinematics(1);
            rbt.UpdateJacobians();
          

            rbt.UpdateFtsPresVel();
            rbt.UpdateFtsPresForce(motors.present_torque);  
            ringBuffer_force.push(rbt.mfForce);
            // cout<<"torque: ";
            // for(auto a:motors.present_torque)
            //     cout<<a<<" ";
            // cout<<endl;
            // for (size_t i = 0; i < 12; i++)
            // {
            //     torque[i] = rbt.dxlMotors.present_torque[i];
            // }            


            /*      Admittance control     */ 
            // rbt.Control();   
           // rbt.VibrationControl_quad(k,1,800.0/1000);
           // rbt.InverseKinematics(rbt.mfXc);    // Admittance control
           //  cout<<"mf Force:"<<rbt.mfForce<<endl;
           // cout<<"xc_dotdot: \n"<<rbt.mfXcDotDot<<"; \nxc_dot: \n"<<rbt.mfXcDot<<"; \nxc: \n"<<rbt.mfXc<<endl;
            // /*      Postion control with Comp      */
            rbt.InverseKinematics(rbt.mfTargetPos); //    Postion control

            /*      Postion control      */
           // rbt.InverseKinematics(rbt.mfLegCmdPos); 
           // cout<<"mfLegCmdPos:\n"<<rbt.mfLegCmdPos<<endl;
            //cout<<"mfJointCmdPos:\n"<<rbt.mfJointCmdPos<<endl;
            // cout<<"mfLegCmdPos: \n"<<rbt.mfLegCmdPos<<endl;
            // cout<<"target_pos: \n"<<rbt.mfTargetPos<<endl;
            // cout<<"legPresPos: \n"<<rbt.mfLegPresPos<<"; \nxc: \n"<<rbt.xc<<endl;
            //cout<<"force:"<<endl<<rbt.mfForce.transpose()<<endl;
            // cout<<"xc_dotdot: \n"<<rbt.mfXcDotDot<<"; \nxc_dot: \n"<<rbt.mfXcDot<<"; \nxc: \n"<<rbt.mfXc<<endl;
            // cout<<endl;

            /*      Set joint angle      */
          //  cout<<"-------------------------------------------"<<endl;
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
            else
                cout<<"timeImpCtller: "<<timeUse<<endl;
        }
    }
  
}
#ifdef PRESSDETECT
void *SvUpdate(void *data)
{   
    ADS1015 ads;
    vector<int> value(4),preValue(4),prepreValue(4);
    for(auto a:value)
        a=0;
    preValue=value;
    prepreValue=value;
    while(1){
        if(1){

        struct timeval startTime={0,0},endTime={0,0};
        double timeUse=0.0;
        int gain=1;
        for(int i=0;i<4;i++)
        {
            value[i]=(int)ads.read_adc(i,gain);
            usleep(10000);
        }
        swap(value[0],value[1]); //to fit the queue of sensor in real world;
        // for(auto a:value)   cout<<a<<" ";
        // cout<<endl;
        rbt.UpdateTouchStatus(value,preValue,prepreValue);
       
        prepreValue=preValue;
        preValue=value;

        if(runFlag == false){
            int count=0;
            for(auto a:rbt.m_glLeg){
                if(a->getTouchStatus()==true)   count++;
            }
            if(count == 4){
                //rbt.autoControlFlag=true;
                runFlag=true;
                for (size_t i = 0; i < 4; i++)
                {
                    if(rbt.m_glLeg[i]->GetLegStatus()!=recover&&rbt.m_glLeg[i]->GetLegStatus()!=stance)
                      rbt.probeTrigger[i]=false;
                }
            }
        }


        gettimeofday(&endTime,NULL);
        timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
        if(timeUse < 1e4)
        usleep(1.0/loopRateSVRead*1e6 - (double)(timeUse) - 10); 
        }

    }

}
#endif

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
    client_addr.sin_addr.s_addr = inet_addr("192.168.137.27"); // 从机的IP地址
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

        // // 获取接收时间
        // auto received_time_point = std::chrono::high_resolution_clock::now();
        // auto received_time_t = std::chrono::high_resolution_clock::to_time_t(received_time_point);
        // char received_time_str[100];
        // strftime(received_time_str, sizeof(received_time_str), "%Y-%m-%d %H:%M:%S", std::gmtime(&received_time_t));

        // // 解析接收到的数据
        // double program_run_time;
        // char sent_time_str[100];
        // sscanf(buffer, "%lf,%31[^\n]", &program_run_time, sent_time_str);
        // std::cout << "buffer: " << buffer << std::endl;
        // std::cout << "senttime: " << sent_time_str << std::endl;

        // // 计算传输延迟
        // std::tm tm2 = {};
        // strptime(sent_time_str, "%Y-%m-%d %H:%M:%S", &tm2);
      
        // auto sent_time_t = std::mktime(&tm2);
        // std::chrono::duration<double> transmission_delay = std::chrono::high_resolution_clock::from_time_t(received_time_t) - std::chrono::high_resolution_clock::from_time_t(sent_time_t);
        

        // std::cout << "received_time_t: " << received_time_t << std::endl;
        // std::cout << "Sent time_t: " << sent_time_t << std::endl;


        

        // // 计算接收时的程序运行时间
        // double adjusted_run_time = program_run_time + transmission_delay.count();

        // std::cout << "program_run_time: " << program_run_time << std::endl;
        // std::cout << "transmission_delay.count(): " << transmission_delay.count() << std::endl;

        // std::cout << "接收到的程序运行时间: " << program_run_time << " 秒" << std::endl;
        // std::cout << "发送时的世界时间: " << sent_time_str << std::endl;
        // std::cout << "接收时的世界时间: " << received_time_str << std::endl;
        std::cout << "传输延迟: " << timeUse2 << " 微秒" << std::endl;
        std::cout << "接收时的程序运行时间: " << adjusted_run_time << " 微秒" << std::endl;
        std::cout << "------------------------------------------------" << std::endl;
        program_run_time.store(adjusted_run_time); 
      


    }

    close(sockfd);
    return 0;
}

// void *test(void *date){

//     std::vector<float> TempPosition;
//     for(int i=0;i<15;++i)
//         TempPosition.push_back(0.0);
//     TempPosition.push_back(1.0);
//     while(1)
//     {
//         motors.setPosition(TempPosition);
//     }

// }

// int main(int argc, char ** argv){

//     pthread_t th7;
//     int ret;
//     ret = pthread_create(&th7,NULL,test,NULL);
//     if(ret != 0)
//     {
//         printf("create pthread1 error!\n");
//         exit(1);
//     }
//     pthread_join(th7, NULL);

//     while(1);

    
    
//     return 0;
// }

int main(int argc, char ** argv)
{   
    
    
    pthread_t th1, th2, th3, th4,th5,th6;
	int ret;
    // ret = pthread_create(&th1,NULL,udpConnect,NULL);
    // if(ret != 0)
	// {
	// 	printf("create pthread1 error!\n");
	// 	exit(1);
	// }
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
    //  ret = pthread_create(&th5,NULL,SvUpdate,NULL);
    // if(ret != 0)
	// {
	// 	printf("create pthread5 error!\n");
	// 	exit(1);
	// }
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
    pthread_join(th5, NULL);
    while(1);

    
    
    return 0;
}