/*
 * i2cdrv.c
 *
 *  Created on: Mar 22, 2021
 *      Author: jobaek
 */

#include <i2c_sensor.h>
#include "main.h"
#include "cmsis_os.h"
#include "bno055.h"

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_delay_msek(u32 msec);


#ifdef  BNO055_API

/************** I2C buffer length******/

#define I2C_BUFFER_LEN 										8
#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX ((u8)1)

I2C_HandleTypeDef *hi2c_sensor;

/*--------------------------------------------------------------------------*
 *  The following API is used to map the I2C bus read, write, delay and
 *  device address with global structure bno055_t
 *-------------------------------------------------------------------------*/

/* USER CODE BEGIN Header_BNO055_Task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BNO055_Task */
void BNO055_Task(void *argument)
{
  /* USER CODE BEGIN BNO055_Task */
  /* Variable used to return value of
   * communication routine*/
  s32 comres = BNO055_ERROR;

	/*  Based on the user need configure I2C interface.
	 *  It is example code to explain how to use the bno055 API*/
	I2C_routine();

  /*--------------------------------------------------------------------------*
   *  This API used to assign the value/reference of
   *  the following parameters
   *  I2C address
   *  Bus Write
   *  Bus read
   *  Chip id
   *  Page id
   *  Accel revision id
   *  Mag revision id
   *  Gyro revision id
   *  Boot loader revision id
   *  Software revision id
   *-------------------------------------------------------------------------*/
  comres = bno055_init(&bno055);

  printf("= CHIP ID 	: 0x%X\r\n", bno055.chip_id);
  printf("= SW Rev 		: 0x%X\r\n", bno055.sw_rev_id);
  printf("= PAGE ID 	: 0x%X\r\n", bno055.page_id);
  printf("= ACC ID 		: 0x%X\r\n", bno055.accel_rev_id);
  printf("= MAG ID 		: 0x%X\r\n", bno055.mag_rev_id);
  printf("= GYRO ID 	: 0x%X\r\n", bno055.gyro_rev_id);
  printf("= BOOT Rev	: 0x%X\r\n", bno055.bl_rev_id);

  /* Infinite loop */
  for(;;)
  {

  	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

  	osDelay(500);
  }
  /* USER CODE END BNO055_Task */
}

/*-------------------------------------------------------------------------*
 *  By using bno055 the following structure parameter can be accessed
 *  Bus write function pointer: BNO055_WR_FUNC_PTR
 *  Bus read function pointer: BNO055_RD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *--------------------------------------------------------------------------*/
s8 I2C_routine(void)
{
	hi2c_sensor = &hi2c1;

	bno055.bus_write = BNO055_I2C_bus_write;
	bno055.bus_read = BNO055_I2C_bus_read;
	bno055.delay_msec = BNO055_delay_msek;

	bno055.dev_addr = BNO055_I2C_ADDR1;

  return BNO055_INIT_VALUE;
}


/*-------------------------------------------------------------------*
 *
 *  This is a sample code for read and write the data by using I2C
 *  Use either I2C  based on your need
 *  The device address defined in the bno055.h file
 *
 *--------------------------------------------------------------------*/

/*  \Brief: The API is used as I2C bus write
 *  \Return : Status of the I2C write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *      will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 BNO055_iERROR = BNO055_INIT_VALUE;
    u8 array[I2C_BUFFER_LEN];
    u8 stringpos = BNO055_INIT_VALUE;

    array[BNO055_INIT_VALUE] = reg_addr;
    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] = *(reg_data + stringpos);
    }

/*
 * Please take the below APIs as your reference for
 * write the data using I2C communication
 * "BNO055_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
 * add your I2C write APIs here
 * BNO055_iERROR is an return value of I2C read API
 * Please select your valid return value
 * In the driver BNO055_SUCCESS defined as 0
 * and FAILURE defined as -1
 * Note :
 * This is a full duplex operation,
 * The first read data is discarded, for that extra write operation
 * have to be initiated. For that cnt+1 operation done
 * in the I2C write string function
 * For more information please refer data sheet SPI communication:
 */
    BNO055_iERROR = HAL_I2C_Master_Transmit(hi2c_sensor, (uint16_t)(bno055.dev_addr << 1), array, cnt, 1000);

    return (s8)BNO055_iERROR;
}

/*  \Brief: The API is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *  will data is going to be read
 *  \param reg_data : This data read from the sensor,
 *   which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 BNO055_iERROR = BNO055_INIT_VALUE;
    u8 array[I2C_BUFFER_LEN] = { BNO055_INIT_VALUE };
    u8 stringpos = BNO055_INIT_VALUE;

    array[BNO055_INIT_VALUE] = reg_addr;

    /* Please take the below API as your reference
     * for read the data using I2C communication
     * add your I2C read API here.
     * "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
     * ARRAY, ARRAY, 1, CNT)"
     * BNO055_iERROR is an return value of SPI write API
     * Please select your valid return value
     * In the driver BNO055_SUCCESS defined as 0
     * and FAILURE defined as -1
     */
    BNO055_iERROR = HAL_I2C_Master_Transmit(hi2c_sensor, (uint16_t)(bno055.dev_addr << 1), array, 1, 1000);

    if (BNO055_iERROR == HAL_OK)
    {
    	BNO055_iERROR = HAL_I2C_Master_Receive(hi2c_sensor, (uint16_t)(bno055.dev_addr << 1), array, cnt, 1000);

    	if (BNO055_iERROR == HAL_OK)
    	{
				for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
				{
						*(reg_data + stringpos) = array[stringpos];
				}
    	}
    }

    return (s8)BNO055_iERROR;
}

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BNO055_delay_msek(u32 msec)
{
    /*Here you can write your own delay routine*/
	HAL_Delay(msec);
}

#endif



//HAL_StatusTypeDef readData(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, uint8_t len) {
//	data[0] = reg;
//	HAL_StatusTypeDef status;
//
//	status = HAL_I2C_Master_Transmit(hi2c, (uint16_t)BNO_055_DEVICE_ADDRESS << 1, data, 1, 100);
//	if(status != HAL_OK) {
//		printf("readData HAL_I2C_Master_Transmit() error!\r\n");
//		return status;
//	}
//
//	status = HAL_I2C_Master_Receive(hi2c, (uint16_t)(BNO_055_DEVICE_ADDRESS << 1), data, len, 100);
//	if(status != HAL_OK) {
//		printf("readData HAL_I2C_Master_Receive() error!\r\n");
//		return status;
//	}
//
//	return status;
//}
//
//HAL_StatusTypeDef writeData(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data, size_t data_len) {
//	uint8_t *buf = (uint8_t*)malloc(data_len+1);
//	if(buf == NULL) {
//		printf("writeData malloc() error!\r\n");
//		return -1;
//	}
//
//	buf[0] = reg;
//	memcpy(buf+1, data, data_len);
//	HAL_StatusTypeDef status;
//
//	status = HAL_I2C_Master_Transmit(hi2c, (uint16_t)(BNO_055_DEVICE_ADDRESS << 1), buf, data_len+1, 100);
//	if(status != HAL_OK) {
//		printf("writeData HAL_I2C_Master_Transmit() error!\r\n");
//	}
//
//	free(buf);
//	return status;
//}
