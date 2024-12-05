#include "i2c.h"
#include "api.h"
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include "dynamixel.h"
#include <vector>
#include <time.h>
#include <stdlib.h>

#define LF_PIN      1
#define RF_PIN      24
#define LH_PIN      28
#define RH_PIN      29
uint8_t svStatus=0b01010101;

int main(){
    API api;
    while(1){
        api.updateIMU();
        cout<<"angel"<<api.fAngle[0]<<api.fAngle[1]<<endl;

        usleep(10000);
    }

}
