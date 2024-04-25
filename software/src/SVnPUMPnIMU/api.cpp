#include "api.h"
#include <iostream>
#include <wiringPi.h> 
#include<bitset>
using namespace std;
// #define DATALOG 1
static volatile char s_cDataUpdate = 0;

API::API()
{
    static int fd;
    unsigned char *i2c_dev = (unsigned char *)"/dev/i2c-1";
    fd = i2c_open(i2c_dev, 3, 3);
    svStatus=0b01010101;
    wiringPiSetup();
    pinMode(1, OUTPUT); 
    pinMode(24, OUTPUT); 
    pinMode(28, OUTPUT);
    pinMode(29, OUTPUT);
    WitInit(WIT_PROTOCOL_I2C, 0x50);
	WitI2cFuncRegister(i2c_write, i2c_read);
	WitRegisterCallBack(CopeSensorData);
	WitDelayMsRegister(Delayms);
    // AutoScanSensor();

    ina219.init(ADDR_40);
    ina219.reset();
    ina219.setCalibration_0_4A(_16V, B_12Bits_128S_69MS, S_12Bits_128S_69MS, ShuntAndBusVoltageContinuous);

}

API::~API()
{
    cout<<"I2C ENDING..."<<endl;
}

/**
 * @brief a function to set solid valves' status
 * 
 * @param value a value int number, sequencing at RF-RH-LH-LF, 10-positive,01-negetive,example as 01101010 represent RF negetive & RH LH LF positive;
 */
void API::setSV(u8 value)
{
    sendValue[0] = value;
    i2c_write_data(I2C_ADD_PCF, sendValue[0], sendValue, 1);
}

void API::setPump(u8 num, u8 status)
{
    digitalWrite(num, status);
}
/**
 * @brief update IMU data, include Acc, Gyro, Angle
 *   Roll Pitch Yaw (X Y Z axis) in the coordinate system of the sensor.
 */
void API::updateIMU()
{
    WitReadReg(AX, 12);
    // usleep(500000);
    if(s_cDataUpdate)
    {
        // printf("\r\n");
        for(int i = 0; i < 3; i++)
        {
            fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
            fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
            fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
        }
        if(s_cDataUpdate & ACC_UPDATE)
        {
            s_cDataUpdate &= ~ACC_UPDATE;
        }
        if(s_cDataUpdate & GYRO_UPDATE)
        {
            s_cDataUpdate &= ~GYRO_UPDATE;
        }
        if(s_cDataUpdate & ANGLE_UPDATE)
        {
            s_cDataUpdate &= ~ANGLE_UPDATE;
        }
        #ifdef DATALOG
        cout<<"Angle: "<<fAngle[0]<<", "<<fAngle[1]<<", "<<fAngle[2]<<endl;
        #endif
    }
}

void API::updatePowerStatus()
{
    bool conversion = false;
    while(!conversion){
        conversion  = ina219.isConversionOk();
    }
    busVoltage = ina219.getBusVoltage_V();
    current  = ina219.getCurrent_mA();
    power  = ina219.getPower_W();
    #ifdef DATALOG
    cout<<busVoltage<<", "<<current<<", "<<power<<endl;
    #endif
}


static int i2c_read(u8 addr, u8 reg, u8 *data, u32 len)
{
	if(i2c_read_data(addr>>1, reg, data, len) < 0)return 0;
	return 1;
}
static int i2c_write(u8 addr, u8 reg, u8 *data, u32 len)
{
	if(i2c_write_data(addr>>1, reg, data, len) < 0)return 0;
	return 1;
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void Delayms(uint16_t ucMs)
{
	usleep(ucMs*1000);
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0x00; i < 0x7F; i++)    // 0x20     40     50
	{
		// i=0x50;      
		WitInit(WIT_PROTOCOL_I2C, i);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			usleep(5);
			if(s_cDataUpdate != 0)
			{
                #ifdef DATALOG
                printf("find %02X addr sensor\n", i);
                #endif
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
    cout<<"can not find sensor\r\n";
	cout<<"please check your connection\r\n";
}

