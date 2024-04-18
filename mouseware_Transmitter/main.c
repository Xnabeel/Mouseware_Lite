/**
 *  Mouseware_Transmitter
 *  ICM-20602 Interface
 *  NUS service enabled
 *
 */
// PM_CENTRAL_ENABLED made 1 to 0

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"           
#include "boards.h"                   
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "ble_nus.h"
#include "ble_cus.h"
#include "fds.h"
#include "nrf_drv_clock.h"
#include "nrf_power.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_bootloader_info.h"
//#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "icm20602_i2c.h"
#include "max17048.h"           
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"       
//#if defined (UART_PRESENT)
//#include "nrf_uart.h"
//#endif
//#if defined (UARTE_PRESENT)
//#include "nrf_uarte.h"
//#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Mouseware"                                 /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_DURATION                12000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (7.5 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (15 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(3000, UNIT_10_MS)             /**< Connection supervisory timeout (3 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define ICM_INTERVAL                    20
#define POWER                           6          
#define ICM_INT_2                       20

#define ALRT                            4            

#define INDICATION_LED_1                12                                          /**< Blue LED indication for BLE connection. */

char str[30];
uint16_t length1;
uint16_t length2;                                    
int16_t mousex,mousey,mousez;                        
int16_t wakex, wakey, wakez;                         
static uint8_t batt_per;

uint16_t idle_count = 0;                                                            /**< Counter variable to handle sleep. */
                        
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
APP_TIMER_DEF(icm);                                                                 /**< Application Timer instance. */
BLE_CUS_DEF(m_cus);                                                                 /**< Custom service instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

ble_uuid_t m_adv_uuids[] = {{CUSTOM_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}};     /**< Universally unique service identifiers. */

const nrfx_timer_t TIMER_LED = NRFX_TIMER_INSTANCE(1);                              /**< Timer 1 Enabled for Blue LED indication. */

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

 /* Number of possible TWI addresses. */
 #define TWI_ADDRESSES      127

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static void read_sensor_data();
static int8_t read_batt_soc();                  
static int8_t read_batt_status();               
static void sleep_mode_enter();                 
static void go_to_sleep();

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


 static void app_timer_periodic_handler(void * unused)
{ 
    read_sensor_data();
    go_to_sleep();
}

//Configuration of wake-up button and entering sleep mode. 
static void go_to_sleep(void)
{
    NRF_LOG_INFO("%d\n",idle_count);
    if(idle_count>=4000)
    {
       idle_count=0;
       sleep_mode_enter();
    }

    if((mousex == 0) && (mousey == 0))
    {
      idle_count++;
    }
    else
    {
      idle_count=0;
    }
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&icm,APP_TIMER_MODE_REPEATED,app_timer_periodic_handler);
    APP_ERROR_CHECK(err_code);
}

//Handler function for the timer 1 
static void timer1_handler(nrf_timer_event_t event_type, void* p_context)
{
  nrf_gpio_pin_toggle(INDICATION_LED_1);
}

/**Timer for blue led.
*/
static void led_timer_init(void)
{
  uint32_t err_code = NRF_SUCCESS;
  uint32_t time_ms = 500;
  uint32_t time_ticks;

  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;          // Configure the timer instance to default settings

  err_code = nrfx_timer_init(&TIMER_LED, &timer_cfg, timer1_handler); // Initialize the timer0 with default settings
  APP_ERROR_CHECK(err_code);                                          // check if any error occured

  time_ticks = nrfx_timer_ms_to_ticks(&TIMER_LED, time_ms);           // convert ms to ticks

  // Assign a channel, pass the number of ticks & enable interrupt
  nrfx_timer_extended_compare(&TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

  nrfx_timer_enable(&TIMER_LED);
}

/**DFU code start.
*/

//Handler for shutdown preparation.
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
  ret_code_t err_code;
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            
            err_code = nrf_sdh_disable_request();
            APP_ERROR_CHECK(err_code);
            err_code = app_timer_stop_all();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

/*Register application shutdown handler with priority 0.*/
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

//Handler for shutdown preparation.
static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        /* Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.*/
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        /*Go to system off.*/
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};

//DFU Advertisement configuration.
static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}

//Function for disconnection.
static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

//Function for handling dfu events from the Buttonless Secure DFU service
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            /*Prevent device from advertising on disconnect.*/
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            /*Disconnect all other bonded devices that currently are connected.
              This is required to receive a service changed indication
              on bootup after a successful (or aborted) Device Firmware Update.*/
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}
/**DFU code ends.
*/


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                //err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                //if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                //{
                //    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                //    APP_ERROR_CHECK(err_code);
                //}
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            //while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_dfu_buttonless_init_t dfus_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);                     
    APP_ERROR_CHECK(err_code);

    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);
    /*Initialize DFU.*/
    dfus_init.evt_handler = ble_dfu_evt_handler;
    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    //uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);

    //// Prepare wakeup buttons.
    //uint32_t err_code = bsp_btn_ble_sleep_mode_prepare();
    //APP_ERROR_CHECK(err_code);
    nrf_gpio_pin_set(INDICATION_LED_1);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);     //Configure to generate interrupt and wakeup on pin signal low. "false" means that gpiote will use the PORT event, which is low power, i.e. does not add any noticable current consumption (<<1uA). Setting this to "true" will make the gpiote module use GPIOTE->IN events which add ~8uA for nRF52 and ~1mA for nRF51.
    uint32_t err_code = nrf_drv_gpiote_in_init(POWER, &in_config, NULL);             //Initialize the wake-up pin
    APP_ERROR_CHECK(err_code);                                                       //Check error code returned
    nrf_drv_gpiote_in_event_enable(POWER, true);                            //Enable event and interrupt for the wakeup pin
    NRF_LOG_INFO("Sleep!");

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            nrfx_timer_disable(&TIMER_LED);
            nrf_gpio_pin_set(INDICATION_LED_1);
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            err_code = app_timer_start(icm,APP_TIMER_TICKS(ICM_INTERVAL), NULL);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_set(INDICATION_LED_1); //Modified
            nrfx_timer_enable(&TIMER_LED);
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            app_timer_stop(icm);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    /* LOW POWER SETUP */
    err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    APP_ERROR_CHECK(err_code);

    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;
    ble_cus_init_t     cus_init;

    err_code = ble_cus_init(&m_cus, &cus_init);
    APP_ERROR_CHECK(err_code);

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;                              /**< Adding Device full name in advertising packet. */
    init.advdata.include_appearance = false;                                              /**< Excluding appearance in advertising packet. */
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;        /**< Setting flag to LE only and limited discovery mode in advertising packet. */

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);   
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;                                    /**< Adding complete 128bit UUID in scan response packet. */

    init.config.ble_adv_fast_enabled  = true;                                             /**< Enabling fast advertisement. */
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;                                 /**< Setting advertising interval. */
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;                                 /**< Setting advertising duration. */
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);                               /**< Initializing ble advertisement. */
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS , bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


/*TWI initialization*/

void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    
    APP_ERROR_CHECK(err_code);
    if (NRF_SUCCESS == err_code)
	{
		nrf_drv_twi_enable(&m_twi);
		NRF_LOG_INFO("TWI init success...");	
	}
}


/**
 * @brief Function for reading data from temperature sensor.
 */
static void read_sensor_data()
{
     int16_t accX, accY, accZ;
    int16_t gyroX, gyroY, gyroZ;
    int8_t percentage;
    //int8_t position_flag = 1;

    ICM20602_read_gyro(&gyroX,&gyroY,&gyroZ);
    ICM20602_read_accel(&accX, &accY, &accZ);
    mousex = -((gyroX + 100)/300);                               /**< mouseX and Y sign changes represent the position of wearing device*/
    mousey = ((gyroZ + 100)/300);
    //mousex = -((gyroY + 100)/300);                             /**< mouseX and Y sign changes represent the position of wearing device*/
    mousez = ((gyroX - 100)/300);
    wakex  =  -((accX + 100)/300);
    wakey  =  ((accY - 100)/300);
    wakez  =  -((accZ + 100)/300);
    //length1 = sprintf(str,"%d,%d\n", gyroX,gyroZ);
    
    percentage = read_batt_soc();
    if(percentage == 0)
    {
     NRF_LOG_INFO("Sleep!");
     sleep_mode_enter(); 
    }
    //if(position_flag == 0)
    //{
    //length1 = sprintf(str,"%d,%d,%d\n", mousex,mousey,percentage);
    //NRF_LOG_INFO("mousex: %d, mousey : %d, percentage: %d", mousex, mousey, percentage);
    //}
    //else
    //{
    //length1 = sprintf(str,"%d,%d,%d\n", mousex,mousez,percentage);
    //NRF_LOG_INFO("mousex: %d, mousez: %d, percentage: %d", mousex, mousez, percentage);
    //}
    length1 = sprintf(str,"%d,%d,%d,%d\n",percentage,mousex,mousey,mousez);
    //NRF_LOG_INFO("%d, %d, %d, percentage: %d",wakex,wakey,wakez,percentage);
    NRF_LOG_INFO("mousex: %d, mousey : %d, mousez: %d, percentage: %d", mousex, mousey, mousez, percentage);
    ret_code_t err_code = ble_nus_data_send(&m_nus, str, &length1, m_conn_handle);    
}


static int8_t read_batt_soc()
{
    uint16_t batt;
    MAX17048_read_soc(&batt);
    batt_per = batt>>8; 
    return batt_per;
}

ret_code_t write_register(nrf_drv_twi_t twi_instance, uint8_t device_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  ret_code_t err_code;
  uint8_t data[len + 1];
	data[0] = reg_addr;
	for (uint16_t i = 0; i < len; i++) {
		data[i + 1] = reg_data[i];
	}
	
  err_code = nrf_drv_twi_tx(&twi_instance, device_addr, data, len+1, false);
  APP_ERROR_CHECK(err_code);
  //NRF_LOG_INFO("WRITE: dev_id: %x reg_addr: %x reg_data: %x len: %i\n", device_addr, reg_addr, *reg_data, len);
   NRF_LOG_FLUSH();
  if(err_code != NRF_SUCCESS) {
    return err_code;
  }

}


ret_code_t read_register(nrf_drv_twi_t twi_instance, uint8_t device_addr, uint8_t register_addr, uint8_t *p_data, uint8_t bytes, bool no_stop)
{
  ret_code_t err_code;
   //NRF_LOG_INFO("READ: dev_id: %x reg_addr: %x len: %i\n", device_addr, register_addr, bytes);
  err_code = nrf_drv_twi_tx(&twi_instance, device_addr, &register_addr, 1, no_stop);
  APP_ERROR_CHECK(err_code);

  if(err_code != NRF_SUCCESS) {
    return err_code;
  }

  err_code = nrf_drv_twi_rx(&twi_instance, device_addr, p_data, bytes);
  //NRF_LOG_INFO("READ: %x\n",*p_data);
  return err_code;
}


static void twi_scanner(void)
{
    ret_code_t err_code;
    uint8_t address;
    uint8_t sample_data;
    bool detected_device = false;

    for (address = 1; address <= TWI_ADDRESSES; address++)
    {
        err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            detected_device = true;
            NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
        }
        NRF_LOG_FLUSH();
    }

    if (!detected_device)
    {
        NRF_LOG_INFO("No device was found.");
        NRF_LOG_FLUSH();
    }
    err_code = ICM20602_init(ACC_FS_2G,GYRO_FS_250DPS,SAMPLE_RATE_1kHZ);
}

static void gpio_init(void)
{
    ret_code_t err_code;

    //Initialize gpiote module
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_uninit(POWER);           
    nrf_drv_gpiote_in_event_disable(POWER);
    
}

/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;
    nrf_gpio_cfg_output(INDICATION_LED_1);
    nrf_gpio_pin_set(INDICATION_LED_1);

    // Initialize.    
    log_init();
    timers_init(); 
    led_timer_init();   
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    twi_init();
    twi_scanner();
    gpio_init();   
    // Start execution.    
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
    advertising_start();    

    // Enter main loop.
     for (;;)
    {
        idle_state_handle();        
    }
    
}


/**
 * @}
 */
