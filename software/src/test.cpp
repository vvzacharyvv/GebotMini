// #include "i2c.h"
// #include "api.h"
// #include <stdio.h>
// #include <unistd.h>
// #include <wiringPi.h>
// #include "dynamixel.h"
// #include <vector>
// #include <time.h>
// #include <stdlib.h>
#include "robotcontrol.h"
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
    struct timeval startTime,endTime;
    double timeUse;
   // API api;
    vector<int> ID;
    vector<float> start_pos;
    vector<float> target_tor;

    // for(int i=0; i<4; i++)
    // {
    // ID.push_back(3*i);
    // ID.push_back(3*i+1);
    // start_pos.push_back(0.00);
    // start_pos.push_back(0.00);
    // }
     for(int i=0; i<16; i++)
    {
    ID.push_back(i);
    start_pos.push_back(0.00);
    }
     DxlAPI gecko("/dev/ttyAMA0", 1000000, ID, 2); //ttyUSB0
//     // gecko.setBaudRate(5);
     gecko.setOperatingMode(3);  //3 position control; 0 current control
     gecko.torqueEnable();
     gecko.setPosition(start_pos);
   // gecko.getPosition();
//     int times=1;
//     bool reverse=false;
//     while(1)
//     {
//             gecko.getTorque();
//             gecko.getPosition();
//             gecko.getVelocity();
//             cout<<" present postion: ";
//             for(int i=0;i<16;i++)
//             cout<<gecko.present_position[i]<<" ";
//             cout<<endl;
//     }
//     //~ usleep(1e6);
    //  api.setPump(1, LOW);
    //  api.setPump(24, LOW);
    //  api.setPump(28, LOW);
    //  api.setPump(29, LOW);
    // float torque[12];
    // usleep(1e6);
     //api.setSV(svStatus);


    CRobotControl rbt(110.0,60.0,20.0,800.0,ADMITTANCE);
    Matrix<float,4,2> TimeForSwingPhase;
    Matrix<float, 4, 3> InitPos;
    Matrix<float, 6,1> TCV;
    TCV << 3.0/1000.0, 0, 0,0,0,0 ;// X, Y , alpha 
   
float  float_initPos[12]={   70.0,65.5,-21.0,
                             70.0,-65.5,-21.0,
                            -84.0, 65.5,-21.0,
                            -84.0, -65.5,-21.0};
    // float  float_initPos[12];
    // string2float("../include/initPos.csv", float_initPos);//Foot end position
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
           
            InitPos(i, j) = float_initPos[i*3+j]/1000;
            // cout<<InitPos(i, j)<<endl;
        }
    rbt.SetInitPos(InitPos);
    rbt.InverseKinematics(rbt.mfLegCmdPos);
    cout<<"cmdPos: "<<rbt.mfJointCmdPos<<endl;
   // rbt.SetPos(rbt.mfJointCmdPos);
 //  rbt.PumpAllPositve();
    usleep(1e6);
   rbt.SetCoMVel(TCV);
    TimeForSwingPhase<< 8*TimeForGaitPeriod/16, 	11*TimeForGaitPeriod/16,		
                        0,		 		 					3*TimeForGaitPeriod/16,		
                        12*TimeForGaitPeriod/16, 	15*TimeForGaitPeriod/16,		
                        4*TimeForGaitPeriod/16, 	7*TimeForGaitPeriod/16;
    rbt.SetPhase(TimePeriod, TimeForGaitPeriod, TimeForSwingPhase);


//     //ADS1015 ads;

   
//     // while(1){
//     //     struct timeval startTime,endTime;
//     //     double timeUse;
//     //     gettimeofday(&startTime,NULL);
//     //     for(int i=0;i<4;i++){
//     //         value[i]=ads.read_adc(i,gain);
//     //           usleep(8000);
//     //     }
//     //     for(auto a:value){
//     //         std::cout<<a<<" ";
//     //     }
//     //     std::cout<<std::endl;
//     //     gettimeofday(&endTime,NULL);
//     //     timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec)+ endTime.tv_usec - startTime.tv_usec;
//     //     std::cout<<"timeUse="<<timeUse<<std::endl;
//     // }
//     /*********adc test********/
//      vector<int> value(4),preValue(4),prepreValue(4);
//     for(auto a:value)
//         a=0;
//     preValue=value;
//     prepreValue=value;
   struct timeval startTimeswing,endTimeswing;
    double timeUseswing;
     gettimeofday(&startTimeswing,NULL);
    for(int times=0; times<2000; times++)
    {
        struct timeval startTime,endTime;
        double timeUse;
        gettimeofday(&startTime,NULL);
        std::cout<<std::endl;
        std::cout<<"times"<<times<<std::endl;
        // gecko.getTorque();
        // gecko.getPosition();
        // gecko.getVelocity();
        rbt.NextStep();
        rbt.InverseKinematics(rbt.mfLegCmdPos);
        cout<<"mfLegCmdPos: \n"<<rbt.mfLegCmdPos<<"\n";//<<"mfJointCmdPos: \n"<<rbt.mfJointCmdPos<<endl;

        rbt.UpdatejointPresPosAndVel(motorMapping(rbt.mfJointCmdPos));
        rbt.ForwardKinematics(1);
       // cout<<"forward: " <<rbt.mfLegPresPos<<endl;
        // cout<<" present postion: ";
        // for(int i=0;i<16;i++)
        //         cout<<gecko.present_position[i]<<" ";
        // cout<<endl;
        SetPos(rbt.mfJointCmdPos,gecko,rbt.vLastSetPos);
        gettimeofday(&endTime,NULL);
        timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
        if(timeUse < 1.0/loopRateStateUpdateSend*1e6)
          usleep(1.0/loopRateStateUpdateSend*1e6 - (double)(timeUse) - 2000); 

        // if(swingtimeFlag==true){
        //     gettimeofday(&endTimeswing,NULL);
        //     timeUseswing = 1e6*(endTimeswing.tv_sec - startTimeswing.tv_sec) + endTime.tv_usec - startTime.tv_usec;
        //     cout<<"timeuse="<<timeUseswing<<endl;
        //     exit(0);
        //}
    }
// //     pos2[11]=((float)times)/2000.0 * 3.14;
// //     pos2[15]=((float)times)/2000.0 * 3.14;
// //     pos2[8]=-((float)times)/2000.0 * 3.14;
// //     pos2[14]=((float)times)/2000.0 * 3.14;
// //     pos2[5]=-((float)times)/2000.0 * 3.14;
// //     pos2[13]=((float)times)/2000.0 * 3.14;
// //     pos2[2]=((float)times)/2000.0 * 3.14;
// //     pos2[12]=((float)times)/2000.0 * 3.14;
// //    rbt.dxlMotors.setPosition(pos2);
// //         // int gain=1;
// //         // for(int i=0;i<4;i++)
// //         // {
// //         //     value[i]=(int)ads.read_adc(i,gain);
// //         //     usleep(10000);
// //         // }
// //         // rbt.UpdateTouchStatus(value,preValue,prepreValue);
// //         // prepreValue=preValue;
// //         // preValue=value;

// //        // rbt.SetPos(rbt.mfJointCmdPos);
// //         usleep(1e3);
// //     }
//     int times=0;
//    bool reverse=false;
//     while(1){
        
//        // std::cout<<times<<std::endl;
//        // rbt.NextStep();
//        //rbt.InverseKinematics(rbt.mfLegCmdPos);
//       rbt.dxlMotors.getTorque();
//      rbt.dxlMotors.getPosition();
//      rbt.dxlMotors.getVelocity();
//     vector<float> pos2(16);
//     for(auto a : pos2){
//         a=0;
//     }
//     if(!reverse){
//     times++;
//     pos2[11]=((float)times)/2000.0 * 3.14;
//     pos2[15]=((float)times)/2000.0 * 3.14;
//     pos2[8]=-((float)times)/2000.0 * 3.14;
//     pos2[14]=((float)times)/2000.0 * 3.14;
//     pos2[5]=-((float)times)/2000.0 * 3.14;
//     pos2[13]=((float)times)/2000.0 * 3.14;
//     pos2[2]=((float)times)/2000.0 * 3.14;
//     pos2[12]=((float)times)/2000.0 * 3.14;
//     if(times>1000) reverse=true;
//     }
//     else{
//         times--;
//     pos2[11]=((float)times)/2000.0 * 3.14;
//     pos2[15]=((float)times)/2000.0 * 3.14;
//     pos2[8]=-((float)times)/2000.0 * 3.14;
//     pos2[14]=((float)times)/2000.0 * 3.14;
//     pos2[5]=-((float)times)/2000.0 * 3.14;
//     pos2[13]=((float)times)/2000.0 * 3.14;
//     pos2[2]=((float)times)/2000.0 * 3.14;
//     pos2[12]=((float)times)/2000.0 * 3.14;
//     if(times<0) reverse=false;
//     }
   
//    rbt.dxlMotors.setPosition(pos2);   
//    usleep(1e3); 
// }
}