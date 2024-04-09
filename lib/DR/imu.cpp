#include "imu.h"
#include "pinout.h"
#include <LSM6.h>

LSM6 imu;
bool IMU_status = false;
bool IMU_available_checked = false;

bool checkIMUAvailable(bool force)
{
    //log_w("Checking IMU availability 0");
    if (IMU_available_checked && !force)
        return IMU_status;

    //log_w("Checking IMU availability 1");
    int16_t test = imu.testReg(LSM6::device_auto, LSM6::WHO_AM_I);
    //log_d("IMU test: %d", test);
    if (test==-1)//imu.init(LSM6::device_auto, LSM6::sa0_low)
    {
        log_e("IMU is not available");
        IMU_status = false;
    }
    else
    {
        //IMU_available_checked = true;
        IMU_status = true;
        //log_w("IMU is available");
    }
    IMU_available_checked = true;
    return IMU_status;
}

void IMUread()
{
    if(!IMU_status && IMU_available_checked)
    {
        log_e("IMU not available");
        return;
    }
    if(!imu.readAcc())
    {
        log_e("Error reading IMU");
        if(checkIMUAvailable(true))
        {
            imu.readAcc();
        }
        else
        {
            log_e("IMU not available");
            return;
        }
    }
    if(!imu.readGyro())
    {
        log_e("Error reading IMU");
    }
    if(!readTemp())
    {
        log_e("Error reading IMU");
    }
}

bool readTemp()
{
    //imu.readTemp();
    return true;
}
String getIMUmeasure()
{
    String out = "Accel: ";
    out += String(imu.a.x);
    out += ", ";
    out += String(imu.a.y);
    out += ", ";
    out += String(imu.a.z);
    out += " Gyro: ";
    out += String(imu.g.x);
    out += ", ";
    out += String(imu.g.y);
    out += ", ";
    out += String(imu.g.z);
    return out;
}
void disableIMU()
{
    imu.DisableIMU();
    // set CS, INT1, INT2 to input mode
    pinMode(IMU_CS, INPUT);
    pinMode(IMU_INT1, INPUT);
    pinMode(IMU_INT2, INPUT);
    IMU_status=false;
    
}
void enableIMU()
{
    if (!imu.init(LSM6::device_auto, LSM6::sa0_low))
    {
        Serial.println("Failed to detect and initialize IMU!");
        while (1);
    }
    //imu.enableDefault();
    IMU_status=true;
}

void setDefaults()
{
    imu.enableDefault();
    //imu.setAccScale(LSM6::ACCELRANGE_2G);
    //imu.setGyroScale(LSM6::GYRORANGE_500DPS);
    //imu.setAccODR(LSM6::ACCELODR_104HZ);
    //imu.setGyroODR(LSM6::GYROODR_104HZ);
}
void imuReadRegs()
{
    // Read all the registers
    for (int i = 0; i < 0x6C; i++)
    {
        Serial.print("0x");
        Serial.print(i, HEX);
        Serial.print(": 0x");
        Serial.println(imu.readReg(i), HEX);
    }

}

bool getIMUstatus()
{
    return IMU_status;
}