/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
* File Name          : stc3100.c
* Author             : AMS team
* Version            : V1.2
* Date               : 2014/10
* Description        : Configuration & various feature implementation of STC3100
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.

* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <arduino.h>
#include "stc3100.h"
//#include <DR_types.h>

#include <Wire.h>
/* Private define ------------------------------------------------------------*/
//#define STC3100_OK 0

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
short s16_BattVoltage;		   // battery voltage in mV
short s16_BattCurrent;		   // battery current in mA
short s16_BattTemperature;	   // battery Temperature in 0.1C
short s16_BattChargeCount;	   // battery charge in mA.h
short s16_BattChargeCountRAW;  // battery charge in mA.h
short s16_BattChargeCountRAW2; // battery charge in mA.h
short s16_BattCounter;		   // conv counter

/* --- constants ---------------------------------------------------------- */

#define CurrentFactor (48210 / SENSERESISTOR)
// LSB=11.77uV/R= ~48210/R/4096 - convert to mA

#define ChargeCountFactor (27443 / SENSERESISTOR)
// LSB=6.7uVh/R ~27443/R/4096 - converter to mAh

#define VoltageFactor 9994
// LSB=2.44mV ~9994/4096 - convert to mV

#define TemperatureFactor 5120
// LSB=0.125C ~5120/4096 - convert to 0.1C

/*******************************************************************************
* Function Name  : conv
* Description    : conversion utility 
*  convert a raw 16-bit value from STC3100 registers into user units (mA, mAh, mV, C)
*  (optimized routine for efficient operation on 8-bit processors such as ST7/STM8)
* Input          :s16_value, u16_factor
* Return         : result = value * factor / 4096
*******************************************************************************/
static s16 conv(s16 s16_value, u16 u16_factor)
{
	return (((s32)s16_value * u16_factor) >> 12);
}

/*******************************************************************************
* Function Name  : STC3100_ReadByte
* Description    : utility function to read the value stored in one register
* Input          : u8_register: STC3100 register,
* Return         : 8-bit value, or 0 if error
*******************************************************************************/
int STC3100::STC3100_ReadByte(u8 Addr)
{
	s8 s8_value;
	u8 pu8_data[2];
	int res;

	res = STC3100_Read_1((unsigned char)Addr, pu8_data);
	if (res == 0)
	{
		delay(200);
		res = STC3100_Read_1((unsigned char)Addr, pu8_data);
	}

	s8_value = pu8_data[0];
	return (s8_value);
}

/*******************************************************************************
* Function Name  : STC3100_ReadWord
* Description    : utility function to read the value stored in a register pair
* Input          : u8_register: STC3100 register,
* Return         : 16-bit value, or 0 if error
*******************************************************************************/
int STC3100::STC3100_ReadWord(u8 Addr)
{
	s16 s16_value;
	u8 pu8_data[2];
	int res;

	res = STC3100_Read(2, Addr, pu8_data);

	if (res == STC3100_OK)
	{
		// no error
		s16_value = pu8_data[0];
		s16_value |= ((s16)pu8_data[1]) << 8;
	}
	else
	{
		return (int)(-1); //error
	}

	return (s16_value);
}

/*******************************************************************************
* Function Name  : STC3100_WriteByte
* Description    : utility function to write a 8-bit value into a register
* Input          : u8_register: STC3100 register, u8_value: 8-bit value to write
* Return         : error status (STC3100_OK, !STC3100_OK)
*******************************************************************************/
int STC3100::STC3100_WriteByte(u8 u8_Register, u8 u8_value)
{
	int res;
	u8 pu8_data[2];

	pu8_data[0] = u8_value;
	res = STC3100_Write(1, u8_Register, pu8_data);

	if (res == STC3100_OK)
	{
		// Ok, no error
		res = 0;
	}
	else
	{
		res = (int)(-1); //error
	}

	return (res);
}

/* -----------------------------------------------------------------
----------------------------------------------------------------- */
/*******************************************************************************
* Function Name  : STC3100_WriteWord
* Description    : utility function to write a 16-bit value into one register pair
* Input          : u8_register: STC3100 register, s16_value: 16-bit value to write
* Return         : error status (STC3100_OK, !STC3100_OK)
*******************************************************************************/
int STC3100::STC3100_WriteWord(u8 u8_Register, s16 s16_value)
{
	int res;
	u8 pu8_data[2];

	pu8_data[0] = s16_value & 0xFF;
	pu8_data[1] = s16_value >> 8;
	res = STC3100_Write(2, u8_Register, pu8_data);

	if (res == STC3100_OK)
	{
		// Ok, no error
		res = 0;
	}
	else
	{
		res = (int)(-1); //error
	}

	return (res);
}

/*******************************************************************************
* Function Name  : STC3100_Startup
* Description    :  initialize and start the STC3100 at application startup
* Input          : None
* Return         : error status (STC3100_OK, !STC3100_OK)
*******************************************************************************/
int STC3100::STC3100_Startup()
{
	//log_d("STC3100_Startup");
	int s32_res;

	// first, check the presence of the STC3100 by reading first byte of dev. ID
	//if (!STC3100_isConnected())
	//	return -1;

	// read the REG_CTRL to reset the GG_EOC and VTM_EOC bits
	int rv1 = STC3100_ReadByte(STC3100_REG_CTRL);
	int rv2 = STC3100_ReadByte(STC3100_REG_MODE);
	//log_d("STC3100_ReadByte(STC3100_REG_CTRL): 0x%x", rv1);
	if (rv2 == 16)
	{
		//log_d("STC3100_ReadByte(STC3100_REG_MODE): 0x%x", rv2);
	}
	else
		log_e("CAUTION, check STC3100_ReadByte(STC3100_REG_MODE) ()!=0x20: 0x%x", rv2);
	// write 0x02 into the REG_CTRL to reset the accumulator and counter and clear the PORDET bit,
	//STC3100_resetGauge();

	// then 0x10 into the REG_MODE register to start the STC3100 in 14-bit resolution mode.
	s32_res = STC3100_WriteByte(STC3100_REG_MODE, 0x10);
	if (s32_res != STC3100_OK)
	{
		log_e("error writing gasgauge startup register");
		return (s32_res);
	}

	return (STC3100_OK);
}
int STC3100::STC3100_resetGauge(void)
{
	int s32_res;
	// write 0x02 into the REG_CTRL to reset the accumulator and counter and clear the PORDET bit,
	s32_res = STC3100_WriteByte(STC3100_REG_CTRL, 0x02);
	if (s32_res != STC3100_OK)
		return (s32_res);

	return (STC3100_OK);
}

bool STC3100::STC3100_isConnected()
{
	int s32_res;

	// first, check the presence of the STC3100 by reading first byte of dev. ID
	s32_res = STC3100_ReadByte(STC3100_REG_ID0);

	if (s32_res != 0x10)
	{
		log_e("STC3100 not found. STC3100_ReadByte(STC3100_REG_ID0):: %d", s32_res);
		//log_e("Reattempt after 500ms");
		//delay(500);
		//s32_res = STC3100_ReadByte(STC3100_REG_ID0);

		//if (s32_res != 0x10)
		//{
		//	log_e("STC3100 not found. STC3100_ReadByte(STC3100_REG_ID0):: %d", s32_res);
		return false;
		//}
		//log_d("Success");
	}

	return true;
}

/*******************************************************************************
* Function Name  : STC3100_Powerdown
* Description    :  stop the STC3100 at application power down
* Input          : None
* Return         : error status (STC3100_OK, !STC3100_OK)
*******************************************************************************/
int STC3100::STC3100_Powerdown(void)
{
	int s32_res;

	// write 0 into the REG_MODE register to put the STC3100 in standby mode
	s32_res = STC3100_WriteByte(STC3100_REG_MODE, 0);
	if (s32_res != STC3100_OK)
		return (s32_res);

	return (STC3100_OK);
}

/*******************************************************************************
* Function Name  : ReadBatteryData
* Description    :  utility function to read the battery data from STC3100
*                  to be called every 2s or so
* Input          : None
* Return         : error status (STC3100_OK, !STC3100_OK)
* Affect         : global battery variables
*******************************************************************************/
int STC3100::ReadBatteryData(void)
{
	//log_w("ReadBatteryData");
	u8 pu8_data[12];
	int s32_res;
	s16 s16_value;

	//Serial.println("// read STC3100 registers 0 to 11");
	//log_ck("############# STC3100 ReadBatteryData 1");
	//if (!STC3100_isConnected())
	//	return -1;

	//log_ck("############# STC3100 ReadBatteryData 2");
	// read STC3100 registers 0 to 11
	s32_res = STC3100_Read_12(0, pu8_data);
	if (s32_res == 0)
	{
		delay(200);
		s32_res = STC3100_Read_12(0, pu8_data);
	}

	if (s32_res != 1)
	{
		log_e("read error");
		return (s32_res); // read failed
	}

	// fill the battery status data

	// charge count
	s16_value = pu8_data[3];
	s16_value = (s16_value << 8) + pu8_data[2];
	s16_BattChargeCountRAW = pu8_data[3];
	s16_BattChargeCountRAW2 = pu8_data[2];
	s16_BattChargeCount = conv(s16_value, ChargeCountFactor); // result in mAh

	// conversion counter
	s16_value = pu8_data[5];
	s16_value = (s16_value << 8) + pu8_data[4];
	s16_BattCounter = s16_value;

	// current
	s16_value = pu8_data[7];
	s16_value = (s16_value << 8) + pu8_data[6];
	s16_value &= 0x3fff; // mask unused bits
	if (s16_value >= 0x2000)
		s16_value -= 0x4000;						  // convert to signed value
	s16_BattCurrent = conv(s16_value, CurrentFactor); // result in mA

	// voltage
	s16_value = pu8_data[9];
	s16_value = (s16_value << 8) + pu8_data[8];
	s16_value &= 0x0fff; // mask unused bits
	if (s16_value >= 0x0800)
		s16_value -= 0x1000;						  // convert to signed value
	s16_BattVoltage = conv(s16_value, VoltageFactor); // result in mV

	// temperature
	s16_value = pu8_data[11];
	s16_value = (s16_value << 8) + pu8_data[10];
	s16_value &= 0x0fff; // mask unused bits
	if (s16_value >= 0x0800)
		s16_value -= 0x1000;								  // convert to signed value
	s16_BattTemperature = conv(s16_value, TemperatureFactor); // result in 0.1C
	//log_w("readed from gasgauge: voltage: ");
	return (STC3100_OK);
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
unsigned long ggtimer = millis();
bool STC3100::ggtimedmeasure(bool force)
{
	if ((millis() > ggtimer + 10000) || force)
	{
		delay(100);
		log_v("getting real (forced) battery measures");

		ReadBatteryData();
		//log_d("ggtimedmeasure 0");
		if (s16_BattVoltage == 0)
		{
			log_w("Caution: s16_BattVoltage==0 so STC3100reinit is need");
			delay(100);
			STC3100_Startup();
			log_w("Waiting 2s");
			delay(2000);
			ReadBatteryData();
		}

		//log_d("ggtimedmeasure 1");
		ggtimer = millis();
		return true;
	}
	return false;
}

int STC3100::stc3100_getTEMP(bool force)
{
	if (!ggtimedmeasure(force))
	{
		//log_d("%dmV. (Battery Voltage) getting from buffer", s16_BattVoltage);
	}
	else
	{
		//log_d("%dmV (Battery Voltage) ", s16_BattVoltage);
	}

	return s16_BattTemperature;
}
int STC3100::stc3100_getVbat(bool force)
{
	if (!ggtimedmeasure(force))
	{
		//log_d("%dmV. (Battery Voltage) getting from buffer", s16_BattVoltage);
	}
	else
	{
		//log_d("%dmV (Battery Voltage) ", s16_BattVoltage);
	}

	return s16_BattVoltage;
}

int STC3100::stc3100_getmAbat(bool force)
{
	if (!ggtimedmeasure(force))
	{
		//log_d("%dmA. (Battery current) getting from buffer", s16_BattCurrent);
	}
	else
	{
		//log_d("%dmA. (Battery current)", s16_BattCurrent);
	}

	return s16_BattCurrent;
}

short STC3100::stc3100_getChargeCount(bool force)
{
	if (!ggtimedmeasure(force))
	{
		//log_d("%dmA. (Battery chargecount) getting from buffer", s16_BattChargeCount);
	}
	else
	{
		//log_d("%dmA (Battery chargecount)", s16_BattChargeCount);
	}

	return s16_BattChargeCount;
}

int STC3100::STC3100_Read_12(unsigned char RegisterAddr, unsigned char *RxBuffer)
{

	Wire.beginTransmission(STC3100_ADDRESS); // Get the slave's attention, tell it we're sending a command byte
	Wire.write(RegisterAddr);
	int error_code = Wire.endTransmission(false);
	if (error_code)
	{
		log_e("i2c error code: %d", error_code);
	}
	Wire.requestFrom((int)STC3100_ADDRESS, (int)12);
	int i = 0;
	while (Wire.available())
	{
		RxBuffer[i] = Wire.read();
		i++;
	}
	Wire.endTransmission();
	int j = 0;

	//for (j = 0; j < i; j++)
	//{
	//	log_d("%d readed 0x%x, %d", j, RxBuffer[j], RxBuffer[j]);
	//}
	//delay(100);
	if (i < 12)
	{
		log_e("error reading gas gauge (STC3100_Read_12 i(%d)<12) 0x%x", i, RegisterAddr);
		return 0;
	}

	else
		return 1;
}

int STC3100::STC3100_Read_1(unsigned char RegisterAddr, unsigned char *RxBuffer)
{
	int i = 0;
	Wire.beginTransmission(STC3100_ADDRESS); // Get the slave's attention, tell it we're sending a command byte
	Wire.write(RegisterAddr);
	int error_code = Wire.endTransmission(false);
	if (error_code)
	{
		log_e("i2c error code: %d", error_code);
	}
	Wire.requestFrom((int)STC3100_ADDRESS, (int)1);

	while (Wire.available())
	{
		RxBuffer[i] = Wire.read();
		i++;
	}
	Wire.endTransmission();
	int j = 0;
	//log_w("t1");

	//CAUTION!! do not remove. this delay is needed for STC3100 and delay() function does not work (maybe is beeing runned on other core?)
	for (j = 0; j < i; j++)
	{
		//log_d("%d readed 0x%x, %d from 0x%x register", j, RxBuffer[j], RxBuffer[j], RegisterAddr);
	}
	//taskYIELD();
	//vTaskDelay(200);
	//log_w("t2");
	//delay(200);
	//log_w("t3");
	if (i < 1)
	{
		log_e("error reading gas gauge register r1 0x%x", RegisterAddr);
		return 0;
	}

	else
		return 1;
}

int STC3100::STC3100_Write(unsigned char ByteCount, unsigned char RegisterAddr, unsigned char *TxBuffer)
{
	int Status = STC31xx_I2C_ERROR;
	int retry;

	for (retry = 0; retry < NBRETRY; retry++)
	{
		Wire.beginTransmission(STC3100_ADDRESS);
		Wire.write(RegisterAddr);
		int i = 0;
		for (i = 0; i < ByteCount; i++)
		{
			Wire.write(TxBuffer[i]);
		}
		Wire.endTransmission();
		Status = STC31xx_I2C_OK;
		delay(50);
		if (Status != STC31xx_I2C_OK)
		{
			//printf("I2C write error\n");
		}
		else
		{
			break; //exit loop
		}
	}

	return Status;
}

int STC3100::STC3100_Read(unsigned char ByteCount, unsigned char RegisterAddr, unsigned char *RxBuffer)
{
	log_w("STC3100_Read. %d bytes from 0x%x register", ByteCount, RegisterAddr);
	int Status = STC31xx_I2C_ERROR;
	int retry;

	if (RxBuffer == (void *)0)
		Status = STC31xx_I2C_ERR_BUFFER;
	else
	{
		//for (retry = 0; retry < NBRETRY; retry++)
		//{
		int i = 0;
		Wire.beginTransmission((uint8_t)STC3100_ADDRESS); // Get the slave's attention, tell it we're sending a command byte
		Wire.write((uint8_t)RegisterAddr);
		Wire.endTransmission();

		Wire.requestFrom((uint8_t)STC3100_ADDRESS, (uint8_t)ByteCount); // request 6 bytes from slave device #2

		while (Wire.available()) // slave may send less than requested
		{
			RxBuffer[i] = Wire.read(); // receive a byte as character
			i++;
		}
		Wire.endTransmission();
		if (i < ByteCount)
		{
			log_e("STC3100 I2C read error. Less reads than expected (%d/%d)\n", i, ByteCount);
		}
		Status = STC31xx_I2C_OK;
		delay(50);
		if (Status != STC31xx_I2C_OK)
		{
			log_e("STC3100 I2C read error\n");
		}
		else
		{
			//break; //exit loop
		}
		//}
	}
	delay(50);
	return Status;
}