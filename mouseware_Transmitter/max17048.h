#ifndef __MAX17048_I2C_H
#define __MAX17048_I2C_H

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf_delay.h"
#include "nrf_drv_twi.h"

#define MAX17048_ADDR                   0x36
#define ID_REG                          0X19
#define MAX17048_VCELL                  0x02
#define MAX17048_SOC                    0x04
#define MAX17048_MODE                   0x06
#define MAX17048_VERSION		0x08
#define MAX17048_HIBRT                  0x0A
#define MAX17048_CONFIG                 0x0C	
#define MAX17048_VALRT                  0x14	
#define MAX17048_CRATE                  0x16	
#define MAX17048_VRESET                 0x18	
#define MAX17048_STATUS                 0x1A	
#define MAX17048_CMDH          	        0xFE

#define gyro_
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
 
int8_t ICM20602_who_am_i(void);
*/
/**
 * @brief function to initialize IMU sensor.
 */
//int8_t ICM20602_init();

void MAX17048_read_soc(uint16_t *soc);

void MAX17048_read_status(uint8_t *status);
#endif
