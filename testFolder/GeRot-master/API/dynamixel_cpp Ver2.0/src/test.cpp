#include <dynamixel.h>
#include <unistd.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <fstream>
#include <string>
#define TC 0

using namespace std;
int main()
{
    vector<int> ID;
    vector<float> start_pos;
    vector<float> target_tor;
    struct timeval startTime={0,0},endTime={0,0};
    double timeUse;
    float K = 0.1;
    float D = 0.005;
    for(int i=0; i<12; i++)
    {
        ID.push_back(i);
        start_pos.push_back(0.0);
        target_tor.push_back(0.0);
    }
    
    DxlAPI gecko("/dev/ttyAMA0", 3000000, ID, 2);

    gecko.setOperatingMode(3);  //3 position control; 0 current control
    gecko.torqueEnable();
    gecko.setPosition(start_pos);
    usleep(1e6);
    if(TC)
    {
        gecko.torqueDisable();
        gecko.setOperatingMode(0);
        gecko.torqueEnable();
    }
    
    for(int times=0; times<100; times++)
    {
        if(TC)
        {
            gecko.getPosition();
            gecko.getVelocity();
            target_tor[0] = K*(0-gecko.present_position[0]) + D*(0-gecko.present_velocity[0]);
            gecko.setTorque(target_tor);
            cout<<"Time: "<< times<<" , Vel: "<<gecko.present_velocity[0]<<" , Pos: "<<gecko.present_position[0]<<" , tor: "<< target_tor[0]<<endl;
        }
        else{
            gettimeofday(&startTime,NULL);
            // gecko.getTorque();
            gecko.setPosition(start_pos);
             gettimeofday(&endTime,NULL);
             usleep(1e3);
            timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
            cout<<"Time: "<< times<<" , Time comsumption: "<<timeUse<<endl;
        }
    }
    gecko.torqueDisable();
}