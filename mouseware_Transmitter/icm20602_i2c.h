#ifndef __ICM20602_I2C_H
#define __ICM20602_I2C_H

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf_delay.h"
#include "nrf_drv_twi.h"

#define ICM20602_ADDR                   0x68
#define ICM20602_ID                     0x12	
#define ICM_SELF_TESTX_REG		0X0D	
#define ICM_SELF_TESTY_REG		0X0E	
#define ICM_SELF_TESTZ_REG		0X0F	
#define ICM_SELF_TESTA_REG		0X10	
#define ICM_SAMPLE_RATE_REG		0X19	
#define ICM_CFG_REG                     0X1A	
#define ICM_GYRO_CFG_REG		0X1B	
#define ICM_ACCEL_CFG1_REG		0X1C	
#define ICM_ACCEL_CFG2_REG		0X1D
#define ICM_MOTION_DET_REG		0X1F	
#define ICM_ACCEL_WOM_X_THR_REG		0X20	
#define ICM_ACCEL_WOM_Y_THR_REG		0X21	
#define ICM_ACCEL_WOM_Z_THR_REG		0X22
#define ICM_FIFO_EN_REG			0X23	

#define ICM_I2CMST_STA_REG		0X36	
#define ICM_INTBP_CFG_REG		0X37	
#define ICM_INT_EN_REG			0X38	
#define ICM_INT_STA_REG			0X3A	

#define ICM_ACCEL_XOUTH_REG		0X3B	
#define ICM_ACCEL_XOUTL_REG		0X3C	
#define ICM_ACCEL_YOUTH_REG		0X3D	
#define ICM_ACCEL_YOUTL_REG		0X3E	
#define ICM_ACCEL_ZOUTH_REG		0X3F	
#define ICM_ACCEL_ZOUTL_REG		0X40	

#define ICM_TEMP_OUTH_REG		0X41	
#define ICM_TEMP_OUTL_REG		0X42	

#define ICM_GYRO_XOUTH_REG		0X43	
#define ICM_GYRO_XOUTL_REG		0X44	
#define ICM_GYRO_YOUTH_REG		0X45	
#define ICM_GYRO_YOUTL_REG		0X46	
#define ICM_GYRO_ZOUTH_REG		0X47	
#define ICM_GYRO_ZOUTL_REG		0X48	

#define ICM_I2CSLV0_DO_REG		0X63	
#define ICM_I2CSLV1_DO_REG		0X64	
#define ICM_I2CSLV2_DO_REG		0X65	
#define ICM_I2CSLV3_DO_REG		0X66	

#define ICM_I2CMST_DELAY_REG            0X67	
#define ICM_SIGPATH_RST_REG		0X68	
#define ICM_MDETECT_CTRL_REG            0X69	
#define ICM_USER_CTRL_REG		0X6A	
#define ICM_PWR_MGMT1_REG		0X6B	
#define ICM_PWR_MGMT2_REG		0X6C	
#define ICM_FIFO_CNTH_REG		0X72	
#define ICM_FIFO_CNTL_REG		0X73	
#define ICM_FIFO_RW_REG			0X74	
#define WHO_AM_I		        0X75	

#define SAMPLE_RATE_1kHZ                1000
#define SAMPLE_RATE_500HZ               500
#define SAMPLE_RATE_4HZ                 4

#define ACC_FS_2G                       0
#define ACC_FS_4G                       1
#define ACC_FS_8G                       2
#define ACC_FS_16G                      3

#define GYRO_FS_250DPS                  0
#define GYRO_FS_500DPS                  1
#define GYRO_FS_1000DPS                 2
#define GYRO_FS_2000DPS                 3

/**
 * @brief function to write sensor registers.
 */
ret_code_t write_register(nrf_drv_twi_t twi_instance, uint8_t device_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

/**
 * @brief function to read sensor registers.
 */
ret_code_t read_register(nrf_drv_twi_t twi_instance, uint8_t device_addr, uint8_t register_addr, uint8_t *p_data, uint8_t bytes, bool no_stop);

/**
 * @brief function to test availablility of IMU by reading WHO_AM_I register.
 */
int8_t ICM20602_who_am_i(void);

/**
 * @brief function to initialize IMU sensor.
 */
int8_t ICM20602_init(uint8_t acc_fs, uint8_t gy_fs,uint16_t rate);

void ICM20602_read_accel(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis);
void ICM20602_read_temp(int16_t *temp);
void ICM20602_read_gyro(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis);


#endif
