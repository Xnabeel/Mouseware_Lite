#include <math.h>
#include <stdint.h>
#include "max17048.h"

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);


int8_t MAX17048_ID(void)
{
    ret_code_t err_code;
    uint8_t ID = 0;    
    err_code = read_register(m_twi, MAX17048_ADDR, ID_REG, &ID, 1, false);
    APP_ERROR_CHECK(err_code);    
    NRF_LOG_INFO("ID = :0x%x",ID);
    NRF_LOG_FLUSH();
    return err_code;
}

void MAX17048_read_soc(uint16_t *soc)
{
    ret_code_t err_code;    
    uint8_t data[2];
    //uint8_t mode_reg;
    err_code = read_register(m_twi, MAX17048_ADDR, MAX17048_SOC, data, sizeof(data), true);
    APP_ERROR_CHECK(err_code);

    //err_code = read_register(m_twi, MAX17048_ADDR, MAX17048_MODE, mode_reg, sizeof(mode_reg), true);
    //APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("%d",data[0]);
    NRF_LOG_INFO("%d",data[1]);
    //NRF_LOG_INFO("%X",mode_reg);
 
  *soc = (data[0] << 8) | data[1];
}

void MAX17048_read_status(uint8_t *status)
{
    ret_code_t err_code;    
    uint8_t *data;
    err_code = read_register(m_twi, MAX17048_ADDR, MAX17048_STATUS, data, sizeof(data), true);
    APP_ERROR_CHECK(err_code);
 
  status = data;
  
}

int8_t MAX17048_init()
{
    int8_t *status;
    //int16_t *stat;
    ret_code_t err_code;
    uint8_t tx_data[2];
    uint8_t *data = tx_data;
    err_code = MAX17048_ID();

    if (err_code == NRF_SUCCESS)
    {      
    	MAX17048_read_status(status);
    	if(*status & 0x0100 == 0x0100)
    	{
    		tx_data[0] = 0x97;
                tx_data[1] = 0x4C;
      		write_register(m_twi,MAX17048_ADDR, MAX17048_CONFIG, data, sizeof(tx_data));            
      		nrf_delay_ms(500);
      		*status = *status & ~(0x0100);
                //stat = &status;
      		write_register(m_twi,MAX17048_ADDR, MAX17048_STATUS, status, sizeof(status));           
      		nrf_delay_ms(500);
	}

return NRF_SUCCESS;
    }


}
