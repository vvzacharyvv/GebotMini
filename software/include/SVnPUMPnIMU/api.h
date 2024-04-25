#ifndef API_H
#define API_H

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/rtc.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <wiringPi.h> 
#include <i2c.h>
#include <string>
#include "wit_c_sdk.h"
#include "GestionINA219.h"
#include "REG.h"

#define I2C_ADD_PCF 0x20
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

typedef unsigned char           u8;
typedef unsigned short          u16;
typedef unsigned int            u32;
typedef unsigned long long      u64;
typedef signed char             s8;
typedef short                   s16;            
typedef int                     s32;
typedef long long               s64;



static void AutoScanSensor(void);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static int i2c_read(u8 addr, u8 reg, u8 *data, u32 len);
static int i2c_write(u8 addr, u8 reg, u8 *data, u32 len);

class API
{
public:
    API();
    ~API();
    GestionINA219 ina219;
    static int fd;
    u8 sendValue[2];
    float fAcc[3], fGyro[3], fAngle[3];
    float busVoltage, current, power;
    void setSV(u8 value); //3 position control; 0 current control
    void setPump(u8 num, u8 status);
    void updateIMU();
    void updatePowerStatus();

    uint8_t svStatus;
    
};




#endif
