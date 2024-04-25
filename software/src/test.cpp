// #include "i2c.h"
// #include "api.h"
// #include <stdio.h>
// #include <unistd.h>
// #include <wiringPi.h>
// #include "dynamixel.h"
// #include <vector>
// #include <time.h>
// #include <stdlib.h>
//#include "robotcontrol.h"
#include "ADS1x15.h"
#define loopRate 100 //hz

#define LF_PIN      1
#define RF_PIN      24
#define LH_PIN      28
#define RH_PIN      29
uint8_t svStatus=0b01010101;
// uint8_t svStatus=0b10101010;


int main()
{
      /*********adc test********/
    ADS1015 ads;
    int gain=1;
    uint16_t value[4];
   
    while(1){
        struct timeval startTime,endTime;
        double timeUse;
        gettimeofday(&startTime,NULL);
        for(int i=0;i<4;i++){
            value[i]=ads.read_adc(i,gain);
              usleep(8000);
        }
        for(auto a:value){
            std::cout<<a<<" ";
        }
        std::cout<<std::endl;
        gettimeofday(&endTime,NULL);
        timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec)+ endTime.tv_usec - startTime.tv_usec;
        std::cout<<"timeUse="<<timeUse<<std::endl;
    }
    /*********adc test********/
    // struct timeval startTime,endTime;
    // double timeUse;
    // API api;
    // vector<int> ID;
    // vector<float> start_pos;
    // vector<float> target_tor;
    // for(int i=1; i<=12; i++)
    // {
    // ID.push_back(i);
    // start_pos.push_back(0.00);
    // }
    // DxlAPI gecko("/dev/ttyAMA0", 2000000, ID, 2); //ttyUSB0
    // // gecko.setBaudRate(5);
    // gecko.setOperatingMode(3);  //3 position control; 0 current control
    // gecko.torqueEnable();
    // gecko.setPosition(start_pos);
    // gecko.getPosition();
    //~ usleep(1e6);
    //~ api.setPump(1, LOW);
    //~ api.setPump(24, LOW);
    //~ api.setPump(28, LOW);
    //~ api.setPump(29, LOW);
    // float torque[12];
    // usleep(1e6);
    // api.setSV(svStatus);

//     CRobotControl rbt(110.0,60.0,20.0,800.0,ADMITTANCE);
//     Matrix<float,4,2> TimeForSwingPhase;
//     Matrix<float, 4, 3> InitPos;
//     Matrix<float, 6,1> TCV;
//     TCV << 0.001/1000, 0, 0,0,0,0 ;// X, Y , alpha 

//     float  float_initPos[12];
//     string2float("../include/initPos.csv", float_initPos);//Foot end position
//     for(int i=0; i<4; i++)
//         for(int j=0;j<3;j++)
//         {
//             InitPos(i, j) = float_initPos[i*3+j]/1000;
//             //cout<<InitPos(i, j)<<endl;
//         }
//     rbt.SetInitPos(InitPos);
//     rbt.InverseKinematics(rbt.mfLegCmdPos);
//     rbt.SetPos(rbt.mfJointCmdPos);
//     usleep(1e6);
//    rbt.SetCoMVel(TCV);
//     TimeForSwingPhase<< 8*TimeForGaitPeriod/16, 	11*TimeForGaitPeriod/16,		
//                         0,		 		 					3*TimeForGaitPeriod/16,		
//                         12*TimeForGaitPeriod/16, 	15*TimeForGaitPeriod/16,		
//                         4*TimeForGaitPeriod/16, 	7*TimeForGaitPeriod/16;
//     rbt.SetPhase(TimePeriod, TimeForGaitPeriod, TimeForSwingPhase);
//     for(int times=0; times<2000; times++)
//     {
//         std::cout<<times<<std::endl;
//         rbt.NextStep();
//         rbt.InverseKinematics(rbt.mfLegCmdPos);
//        // rbt.SetPos(rbt.mfJointCmdPos);
//         usleep(1e3);
//     }
    
   

    


}
