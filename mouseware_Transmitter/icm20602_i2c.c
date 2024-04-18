#include <math.h>
#include <stdint.h>
#include "icm20602_i2c.h"

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);


int8_t ICM20602_who_am_i(void)
{
    ret_code_t err_code;
    uint8_t who_am_i = 0;    
    err_code = read_register(m_twi, ICM20602_ADDR, WHO_AM_I, &who_am_i, 1, false);
    APP_ERROR_CHECK(err_code);    
    NRF_LOG_INFO("who_am_i = :0x%x",who_am_i);
    NRF_LOG_FLUSH();
    return err_code;
}

void ICM20602_read_accel(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis)
{
    ret_code_t err_code;    
    uint8_t data[6];
    err_code = read_register(m_twi, ICM20602_ADDR, ICM_ACCEL_XOUTH_REG, data, sizeof(data), true);
    APP_ERROR_CHECK(err_code);
 
  *x_axis = (data[0] << 8) | data[1];
  *y_axis = (data[2] << 8) | data[3];
  *z_axis = (data[4] << 8) | data[5];
}

void ICM20602_read_temp(int16_t *temp)
{
    ret_code_t err_code;    
    uint8_t data[2];
    err_code = read_register(m_twi, ICM20602_ADDR, ICM_TEMP_OUTH_REG, data, sizeof(data), true);
    APP_ERROR_CHECK(err_code);
 
  *temp = (data[0] << 8) | data[1];
  
}

void ICM20602_read_gyro(int16_t *x_axis, int16_t *y_axis, int16_t *z_axis)
{
    ret_code_t err_code;    
    uint8_t data[6];
    err_code = read_register(m_twi, ICM20602_ADDR, ICM_GYRO_XOUTH_REG, data, sizeof(data), true);
    APP_ERROR_CHECK(err_code);
 
  *x_axis = (data[0] << 8) | data[1];
  *y_axis = (data[2] << 8) | data[3];
  *z_axis = (data[4] << 8) | data[5];
}

void wake_on_motion()
{
   uint8_t tx_data[1];
   //step 1
   tx_data[0] = read_register(m_twi, ICM20602_ADDR, ICM_PWR_MGMT1_REG, tx_data, sizeof(tx_data), true);
   tx_data[0] = tx_data[0] & 0x8F;
   write_register(m_twi,ICM20602_ADDR, ICM_PWR_MGMT1_REG, tx_data, sizeof(tx_data));             
   tx_data[0] = 0x00;
   write_register(m_twi,ICM20602_ADDR, ICM_PWR_MGMT2_REG, tx_data, sizeof(tx_data));
   //step 2
   tx_data[0] = 0x21;
   write_register(m_twi,ICM20602_ADDR, ICM_ACCEL_CFG2_REG, tx_data, sizeof(tx_data));
   //step 2.5
   tx_data[0] = read_register(m_twi, ICM20602_ADDR, ICM_INTBP_CFG_REG, tx_data, sizeof(tx_data), true);
   tx_data[0] = (tx_data[0] | 0x88) & 0xDF;
   write_register(m_twi, ICM20602_ADDR, ICM_INTBP_CFG_REG, tx_data, sizeof(tx_data));
   //step 3
   tx_data[0] = 0xE0;
   write_register(m_twi, ICM20602_ADDR, ICM_INT_EN_REG, tx_data, sizeof(tx_data));
   //step 4
   tx_data[0] = 0x03;
   write_register(m_twi, ICM20602_ADDR, ICM_ACCEL_WOM_X_THR_REG, tx_data, sizeof(tx_data));
   write_register(m_twi, ICM20602_ADDR, ICM_ACCEL_WOM_Y_THR_REG, tx_data, sizeof(tx_data));
   write_register(m_twi, ICM20602_ADDR, ICM_ACCEL_WOM_Z_THR_REG, tx_data, sizeof(tx_data));
   //step 5
   tx_data[0] = 0xC2;
   write_register(m_twi, ICM20602_ADDR, ICM_MDETECT_CTRL_REG, tx_data, sizeof(tx_data));
   //step 6
   tx_data[0] = 0x0F;
   write_register(m_twi, ICM20602_ADDR, ICM_SAMPLE_RATE_REG, tx_data, sizeof(tx_data));
   //step 7
   tx_data[0] = read_register(m_twi, ICM20602_ADDR, ICM_PWR_MGMT1_REG, tx_data, sizeof(tx_data), true);
   tx_data[0] |= 0x00;
   write_register(m_twi,ICM20602_ADDR, ICM_PWR_MGMT1_REG, tx_data, sizeof(tx_data));             
   tx_data[0] = read_register(m_twi, ICM20602_ADDR, ICM_INT_STA_REG, tx_data, sizeof(tx_data), true);
   NRF_LOG_INFO("status: %x",tx_data[0]);
}


int8_t ICM20602_init(uint8_t acc_fs, uint8_t gy_fs,uint16_t rate)
{
    ret_code_t err_code;
    uint8_t tx_data[1];

    err_code = ICM20602_who_am_i();
    if (err_code == NRF_SUCCESS)
    {      
      tx_data[0] = 0x80;
      write_register(m_twi,ICM20602_ADDR, ICM_PWR_MGMT1_REG, tx_data, sizeof(tx_data));             //  reset the ICM
      nrf_delay_ms(500);
      err_code = ICM20602_who_am_i();
      tx_data[0] = 0x80;
      write_register(m_twi,ICM20602_ADDR, ICM_CFG_REG, tx_data, sizeof(tx_data));             //  reset the ICM
      nrf_delay_ms(50);
      uint8_t pdata;
      err_code = read_register(m_twi, ICM20602_ADDR, ICM_PWR_MGMT1_REG, &pdata, sizeof(pdata), true);
      APP_ERROR_CHECK(err_code);
      pdata |= (0x08 &0xFF);
      NRF_LOG_INFO("data : %x ",pdata);
      tx_data[0] = 0x01;
      write_register(m_twi,ICM20602_ADDR, ICM_PWR_MGMT1_REG, tx_data, sizeof(tx_data));             //   Autoselect CLock PLL
      nrf_delay_ms(50);
      tx_data[0] = 0x0F;
      write_register(m_twi,ICM20602_ADDR, ICM_PWR_MGMT2_REG, tx_data, sizeof(tx_data));             //   Accel & Gyro Standby
      nrf_delay_ms(50);
      tx_data[0] = 0x00;
      write_register(m_twi,ICM20602_ADDR, ICM_USER_CTRL_REG, tx_data, sizeof(tx_data));             //   disable FIFO
      nrf_delay_ms(50);      
      tx_data[0] = acc_fs << 3;
      write_register(m_twi,ICM20602_ADDR, ICM_ACCEL_CFG1_REG, tx_data, sizeof(tx_data));             //   ACC config
      nrf_delay_ms(50);
      tx_data[0] = gy_fs << 3;
      write_register(m_twi,ICM20602_ADDR, ICM_GYRO_CFG_REG, tx_data, sizeof(tx_data));             //   Gyro Config
      nrf_delay_ms(50);
      uint8_t data;
      if(rate>1000)rate=1000;
      if(rate<4)rate=4;
      data=1000/rate-1;
      NRF_LOG_INFO("DATA : %d ", data);
      tx_data[0] = data;
      write_register(m_twi,ICM20602_ADDR, ICM_SAMPLE_RATE_REG, tx_data, sizeof(tx_data));             //   Sample rate
      nrf_delay_ms(50);
      uint8_t lpf = rate/2;
      data=0;
      if(lpf>=188)data=1;
      else if(lpf>=98)data=2;
      else if(lpf>=42)data=3;
      else if(lpf>=20)data=4;
      else if(lpf>=10)data=5;
      else data=6; 
      NRF_LOG_INFO("DATA : %d ", data);
      tx_data[0] = data;
      write_register(m_twi,ICM20602_ADDR, ICM_CFG_REG, tx_data, sizeof(tx_data));             //   LPF config
      nrf_delay_ms(50);
      tx_data[0] = 0x00;
      write_register(m_twi,ICM20602_ADDR, ICM_INT_EN_REG, tx_data, sizeof(tx_data));             //   Interrupt disable
      nrf_delay_ms(50);
      tx_data[0] = 0x00;
      write_register(m_twi,ICM20602_ADDR, ICM_PWR_MGMT2_REG, tx_data, sizeof(tx_data));             //   Accel & Gyro Enable
      nrf_delay_ms(50);
      wake_on_motion();

return NRF_SUCCESS;
    }


}