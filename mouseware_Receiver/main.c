/**
 * 
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "nrf_gpio.h"
#include "nordic_common.h"
#include "app_error.h"
//#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "nrf.h"
#include "bsp.h"

#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "ble_nus_c.h"
#include "ble_cus.h"

#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_bootloader_info.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_advertising.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "ble_dfu.h"

#include "nrf_drv_usbd.h"
#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_hid_generic.h"
#include "app_usbd_hid_mouse.h"
#include "app_usbd_hid_kbd.h"
#include "nrf_drv_power.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"

#define push_btn                4
#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

//#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA  0                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */

#define BLUE_LED                2                                       /**< Blue LED for BLE connection indication. */
#define RED_LED                 3                                       /**< Red LED for low battery indication indication. */
//#define GREEN_LED               28                                      

#define APP_ADV_INTERVAL        64                                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_DURATION        18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define SENSE                   1
static char const m_target_periph_name[] = "Mouseware";

char read_data[15];
uint8_t read_data_len;
int16_t accX,accY,accZ,batt_per;
int16_t mousex,mousey;
uint8_t led_alert = 0;
uint8_t connection_status = 0;

const nrfx_timer_t TIMER_IDLE = NRFX_TIMER_INSTANCE(1);                 /**< Timer 1 Enabled for Blue LED indication. */
const nrfx_timer_t TIMER_RED = NRFX_TIMER_INSTANCE(2);                  /**< Timer 2 Enabled for red LED indication. */

BLE_CUS_DEF(m_cus);                                                     /**< Custom service instance. */
BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                     /**< Advertising module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief CUS UUID. */
static ble_uuid_t const m_cus_uuid =
{
    .uuid = CUSTOM_SERVICE_UUID,
    .type = BLE_UUID_TYPE_VENDOR_BEGIN
};

static ble_uuid_t const m_cus_2_uuid =
{
    .uuid = UNIQUE_SERVICE_UUID,
    .type = BLE_UUID_TYPE_VENDOR_BEGIN
};

/**
 * @brief Enable USB power detection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

/**
 * @brief HID generic class interface number.
 * */
#define HID_GENERIC_INTERFACE  0

/**
 * @brief HID generic class endpoint number.
 * */
#define HID_GENERIC_EPIN       NRF_DRV_USBD_EPIN1

/**
 * @brief Mouse speed (value sent via HID when board button is pressed).
 * */
#define CONFIG_MOUSE_MOVE_SPEED (3)

/**
 * @brief Mouse move repeat time in milliseconds
 */
#define CONFIG_MOUSE_MOVE_TIME_MS (5)


/* GPIO used as LED & buttons in this example */
#define LED_USB_START    (BSP_BOARD_LED_0)
#define LED_HID_REP_IN   (BSP_BOARD_LED_2)

#define BTN_MOUSE_X_POS  0
#define BTN_MOUSE_Y_POS  1
#define BTN_MOUSE_LEFT   2
#define BTN_MOUSE_RIGHT  3

/**
 * @brief Left button mask in buttons report
 */
#define HID_BTN_LEFT_MASK  (1U << 0)

/**
 * @brief Right button mask in buttons report
 */
#define HID_BTN_RIGHT_MASK (1U << 1)

/* HID report layout */
#define HID_BTN_IDX   0 /**< Button bit mask position */
#define HID_X_IDX     1 /**< X offset position */
#define HID_Y_IDX     2 /**< Y offset position */
#define HID_W_IDX     3 /**< Wheel position  */
#define HID_REP_SIZE  4 /**< The size of the report */

/**
 * @brief Number of reports defined in report descriptor.
 */
#define REPORT_IN_QUEUE_SIZE    1

/**
 * @brief Size of maximum output report. HID generic class will reserve
 *        this buffer size + 1 memory space. 
 *
 * Maximum value of this define is 63 bytes. Library automatically adds
 * one byte for report ID. This means that output report size is limited
 * to 64 bytes.
 */
#define REPORT_OUT_MAXSIZE  0

/**
 * @brief Feature report maximum size. HID generic class will reserve
 *        this buffer size + 1 memory space. 
 */
#define REPORT_FEATURE_MAXSIZE  31

/**
 * @brief HID generic class endpoints count.
 * */
#define HID_GENERIC_EP_COUNT  1

/**
 * @brief List of HID generic class endpoints.
 * */
#define ENDPOINT_LIST()                                      \
(                                                            \
        HID_GENERIC_EPIN                                     \
)

/**
 * @brief Additional key release events
 *
 * This example needs to process release events of used buttons
 */
enum {
    BSP_USER_EVENT_RELEASE_0 = BSP_EVENT_KEY_LAST + 1, /**< Button 0 released */
    BSP_USER_EVENT_RELEASE_1,                          /**< Button 1 released */
    BSP_USER_EVENT_RELEASE_2,                          /**< Button 2 released */
    BSP_USER_EVENT_RELEASE_3,                          /**< Button 3 released */
    BSP_USER_EVENT_RELEASE_4,                          /**< Button 4 released */
    BSP_USER_EVENT_RELEASE_5,                          /**< Button 5 released */
    BSP_USER_EVENT_RELEASE_6,                          /**< Button 6 released */
    BSP_USER_EVENT_RELEASE_7,                          /**< Button 7 released */
};

/**
 * @brief HID generic mouse action types
 */
typedef enum {
    HID_GENERIC_MOUSE_X,
    HID_GENERIC_MOUSE_Y,
    HID_GENERIC_MOUSE_BTN_LEFT,
    HID_GENERIC_MOUSE_BTN_RIGHT,
} hid_generic_mouse_action_t;

/**
 * @brief User event handler.
 * */
static void hid_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event);

/**
 * @brief Reuse HID mouse report descriptor for HID generic class
 */
APP_USBD_HID_GENERIC_SUBCLASS_REPORT_DESC(mouse_desc,APP_USBD_HID_MOUSE_REPORT_DSC_BUTTON(2));

static const app_usbd_hid_subclass_desc_t * reps[] = {&mouse_desc};

/*lint -save -e26 -e64 -e123 -e505 -e651*/

/**
 * @brief Global HID generic instance
 */
APP_USBD_HID_GENERIC_GLOBAL_DEF(m_app_hid_generic,
                                HID_GENERIC_INTERFACE,
                                hid_user_ev_handler,
                                ENDPOINT_LIST(),
                                reps,
                                REPORT_IN_QUEUE_SIZE,
                                REPORT_OUT_MAXSIZE,
                                REPORT_FEATURE_MAXSIZE,
                                APP_USBD_HID_SUBCLASS_BOOT,
                                APP_USBD_HID_PROTO_MOUSE);

/*lint -restore*/


/**
 * @brief Mouse state
 *
 * Current mouse status
 */
struct
{
    int16_t acc_x;    /**< Accumulated x state */
    int16_t acc_y;    /**< Accumulated y state */
    uint8_t btn;      /**< Current btn state */
    uint8_t last_btn; /**< Last transfered button state */
}m_mouse_state;

/**
 * @brief Mark the ongoing transmission
 *
 * Marks that the report buffer is busy and cannot be used until transmission finishes
 * or invalidates (by USB reset or suspend event).
 */
static bool m_report_pending;

/**
 * @brief Timer to repeat mouse move
 */
APP_TIMER_DEF(m_mouse_move_timer);

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**Timer 2 handler to handle low battery indication.
*/
void timer0_handler(nrf_timer_event_t event_type, void* p_context)
{   
    if((connection_status == 1) && (led_alert == 1))
    {
     nrf_gpio_pin_set(BLUE_LED);
     nrf_gpio_pin_toggle(RED_LED);
     led_alert = 0;
    }
    else
    {
      nrf_gpio_pin_set(BLUE_LED);
      nrf_gpio_pin_set(RED_LED);
    }
}

/**Timer 1 handler to handle ble connection indication.
*/
void timer1_handler(nrf_timer_event_t event_type, void* p_context)
{    
    nrf_gpio_pin_set(RED_LED);
    nrf_gpio_pin_toggle(BLUE_LED);        
}

/**Timer 2 Initialization
*/ 
void timer0_init(void)
{
  uint32_t err_code = NRF_SUCCESS;
  uint32_t time_ms = 500;
  uint32_t time_ticks;
 
  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;          // Configure the timer instance to default settings

  err_code = nrfx_timer_init(&TIMER_RED, &timer_cfg, timer0_handler); // Initialize the timer1 with default settings
  APP_ERROR_CHECK(err_code);

  time_ticks = nrfx_timer_ms_to_ticks(&TIMER_RED, time_ms);           // convert ms to ticks

  // Assign a channel, pass the number of ticks & enable interrupt
  nrfx_timer_extended_compare(&TIMER_RED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
      
}

/**Timer 1 Initialization
*/ 
void timer1_init(void)
{
  uint32_t err_code = NRF_SUCCESS;
  uint32_t time_ms = 500;
  uint32_t time_ticks;
 
  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;           // Configure the timer instance to default settings

  err_code = nrfx_timer_init(&TIMER_IDLE, &timer_cfg, timer1_handler); // Initialize the timer1 with default settings
  APP_ERROR_CHECK(err_code);

  time_ticks = nrfx_timer_ms_to_ticks(&TIMER_IDLE, time_ms);           // convert ms to ticks

  // Assign a channel, pass the number of ticks & enable interrupt
  nrfx_timer_extended_compare(&TIMER_IDLE, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
   
  nrfx_timer_enable(&TIMER_IDLE);
}


/** DFU code start.
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

/** DFU code ends.
*/

/**
 * @brief Get maximal allowed accumulated value
 *
 * Function gets maximal value from the accumulated input.
 * @sa m_mouse_state::acc_x, m_mouse_state::acc_y
 */
static int8_t hid_acc_for_report_get(int16_t acc)
{
    if(acc > INT8_MAX)
    {
        return INT8_MAX;
    }
    else if(acc < INT8_MIN)
    {
        return INT8_MIN;
    }
    else
    {
        return (int8_t)(acc);
    }
}

/**
 * @brief Internal function that process mouse state
 *
 * This function checks current mouse state and tries to send
 * new report if required.
 * If report sending was successful it clears accumulated positions
 * and mark last button state that was transfered.
 */
static void hid_generic_mouse_process_state(void)
{
    if (m_report_pending)
        return;
    if ((m_mouse_state.acc_x != 0) ||
        (m_mouse_state.acc_y != 0) ||
        (m_mouse_state.btn != m_mouse_state.last_btn))
    {
        ret_code_t ret;
        static uint8_t report[HID_REP_SIZE];
        /* We have some status changed that we need to transfer */
        report[HID_BTN_IDX] = m_mouse_state.btn;
        report[HID_X_IDX]   = (uint8_t)hid_acc_for_report_get(m_mouse_state.acc_x);
        report[HID_Y_IDX]   = (uint8_t)hid_acc_for_report_get(m_mouse_state.acc_y);
        /* Start the transfer */
        //printf("R[%d] = %d;   R[%d] = %d;   R[%d] = %d;\n",HID_BTN_IDX,report[HID_BTN_IDX],HID_X_IDX,report[HID_X_IDX],HID_Y_IDX,report[HID_Y_IDX]);
        ret = app_usbd_hid_generic_in_report_set(
            &m_app_hid_generic,
            report,
            sizeof(report));
        if (ret == NRF_SUCCESS)
        {
            m_report_pending = true;
            m_mouse_state.last_btn = report[HID_BTN_IDX];
            CRITICAL_REGION_ENTER();
            /* This part of the code can fail if interrupted by BSP keys processing.
             * Lock interrupts to be safe */
            //m_mouse_state.acc_x   -= (int8_t)report[HID_X_IDX];
            //m_mouse_state.acc_y   -= (int8_t)report[HID_Y_IDX]; 
            m_mouse_state.acc_x   = 0;
            m_mouse_state.acc_y   = 0;
            CRITICAL_REGION_EXIT();
        }
         m_mouse_state.acc_x   = 0;
         m_mouse_state.acc_y   = 0;
    }
}

/**
 * @brief HID generic IN report send handling
 * */
static void hid_generic_mouse_action(hid_generic_mouse_action_t action, int8_t param)
{
    CRITICAL_REGION_ENTER();
    /*
     * Update mouse state
     */
    switch (action)
    {
        case HID_GENERIC_MOUSE_X:
            m_mouse_state.acc_x += param;
           // printf("mx : %d\n",m_mouse_state.acc_x);
            break;
        case HID_GENERIC_MOUSE_Y:
            m_mouse_state.acc_y += param;
           // printf("my : %d\n",m_mouse_state.acc_y);
            break;
        case HID_GENERIC_MOUSE_BTN_RIGHT:
            if(param == 1)
            {
                m_mouse_state.btn |= HID_BTN_RIGHT_MASK;
            }
            else
            {
                m_mouse_state.btn &= ~HID_BTN_RIGHT_MASK;
            }
            break;
        case HID_GENERIC_MOUSE_BTN_LEFT:
            if(param == 1)
            {
                m_mouse_state.btn |= HID_BTN_LEFT_MASK;
            }
            else
            {
                m_mouse_state.btn &= ~HID_BTN_LEFT_MASK;
            }
            break;
    }
    CRITICAL_REGION_EXIT();
}

/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
              ble_gap_evt_connected_t const * p_connected =
                               p_scan_evt->params.connected.p_connected;
             // Scan is automatically stopped by the connection.
             NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
         } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.");
             scan_start();
         } break;

         default:
             break;
    }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;
    ble_cus_init_t     cus_init;

    err_code = ble_cus_init(&m_cus, &cus_init);
    APP_ERROR_CHECK(err_code);

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    //if (strlen(m_target_periph_name) != 0)
    //{
    //    err_code = nrf_ble_scan_filter_set(&m_scan,
    //                                       SCAN_NAME_FILTER,
    //                                       m_target_periph_name);
    //    APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_cus_uuid);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_cus_2_uuid);
        APP_ERROR_CHECK(err_code);

    //}
    //err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    //APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}


/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;
    uint8_t flag = 1;

    NRF_LOG_DEBUG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {  
             if(p_data[i]==',')
            {
              read_data_len++;
            }
            read_data[i] = p_data[i];
            //ret_val = app_uart_put(p_data[i]);
            //if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            //{
            //    NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
            //    APP_ERROR_CHECK(ret_val);
            //}
        } while (ret_val == NRF_ERROR_BUSY);
    }
    //if (p_data[data_len-1] == '\r')
    //{
    //    while (app_uart_put('\n') == NRF_ERROR_BUSY);
    //}
    if (ECHOBACK_BLE_UART_DATA)
    {
        // Send data back to the peripheral.
        do
        {
            ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if(read_data != '\0')
    { 
      char *mX = strtok(read_data,",");
        batt_per=atoi(mX);
        mX = strtok(NULL,",");
        accX = atoi(mX);
        mX = strtok(NULL,",");
        accY = atoi(mX);
        mX = strtok(NULL,",");
        accZ = atoi(mX); 
        if(flag == 0)
        {
          accY = accZ;
        } 
        //else if(read_data_len==4)
        //{
        //  mX = strtok(NULL,",");
        //  batt_per=atoi(mX);
        //  NRF_LOG_INFO("20% battery left");
        //}
        //mousex = -((accX +100)/300);
        //mousey = (accY-100)/300;
        //m_mouse_state.acc_x += (accX * SENSE);
        //m_mouse_state.acc_y += (accY * SENSE); 
        //m_mouse_state.acc_x += (mousex * SENSE);
        //m_mouse_state.acc_y += (mousey * SENSE);
        //NRF_LOG_INFO("%d :  %d   accX : %d  accY : %d",accX,accY,mousex,mousey);
        // while (app_usbd_event_queue_process())
        //{
        //    /* Nothing to do */
        //}
        //hid_generic_mouse_process_state(); 
    }
}

/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            accX = 0; accY = 0;
            m_mouse_state.acc_x   = 0;
            m_mouse_state.acc_y   = 0;
            scan_start();
            break;
    }
}

/**@snippet [Handling events from the ble_nus_c module] */


/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            
            connection_status = 1;
             
            nrfx_timer_disable(&TIMER_IDLE);
            nrf_gpio_pin_set(BLUE_LED);
            nrfx_timer_enable(&TIMER_RED);
            
            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:

            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
                         accX = 0;
                         accY = 0;
                         m_mouse_state.acc_x   = 0;
                         m_mouse_state.acc_y   = 0;
                         led_alert = 0;
                         connection_status = 0;
                         nrfx_timer_disable(&TIMER_RED);
                         nrf_gpio_pin_set(RED_LED);
                         nrfx_timer_enable(&TIMER_IDLE);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
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

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
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
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        //case BSP_EVENT_SLEEP:
        //    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
        //    break;

        //case BSP_EVENT_DISCONNECT:
        //    err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
        //                                     BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        //    if (err_code != NRF_ERROR_INVALID_STATE)
        //    {
        //        APP_ERROR_CHECK(err_code);
        //    }
        //    break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_MOUSE_X_POS):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_X, CONFIG_MOUSE_MOVE_SPEED);
            UNUSED_RETURN_VALUE(app_timer_start(m_mouse_move_timer, APP_TIMER_TICKS(CONFIG_MOUSE_MOVE_TIME_MS), NULL));
            //printf("Button_press \n");
           
            break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_MOUSE_Y_POS):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_Y, CONFIG_MOUSE_MOVE_SPEED);
            UNUSED_RETURN_VALUE(app_timer_start(m_mouse_move_timer, APP_TIMER_TICKS(CONFIG_MOUSE_MOVE_TIME_MS), NULL));
            break;

        case CONCAT_2(BSP_EVENT_KEY_, BTN_MOUSE_RIGHT):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_BTN_RIGHT, 1);
            break;
        case CONCAT_2(BSP_USER_EVENT_RELEASE_, BTN_MOUSE_RIGHT):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_BTN_RIGHT, -1);
            break;

        case CONCAT_2(BSP_EVENT_KEY_,BTN_MOUSE_LEFT):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_BTN_LEFT, 1);
            break;
        case CONCAT_2(BSP_USER_EVENT_RELEASE_,BTN_MOUSE_LEFT):
            hid_generic_mouse_action(HID_GENERIC_MOUSE_BTN_LEFT, -1);
										  
			 
            break;

        default:
            break;
    }
}



/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;
  
    init.evt_handler   = ble_nus_c_evt_handler;
    init.error_handler = nus_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;
    
    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}

#define INIT_BSP_ASSIGN_RELEASE_ACTION(btn)                      \
    APP_ERROR_CHECK(                                             \
        bsp_event_to_button_action_assign(                       \
            btn,                                                 \
            BSP_BUTTON_ACTION_RELEASE,                           \
            (bsp_event_t)CONCAT_2(BSP_USER_EVENT_RELEASE_, btn)) \
    )

/**@brief Function for initializing the Device Firmware Update(DFU). */
static void dfu_init(void)
{
  ret_code_t       err_code;
  ble_dfu_buttonless_init_t dfus_init = {0};
  
  err_code = ble_dfu_buttonless_async_svci_init();
  APP_ERROR_CHECK(err_code);
  /*Initialize DFU.*/
  dfus_init.evt_handler = ble_dfu_evt_handler;
  err_code = ble_dfu_buttonless_init(&dfus_init);
  APP_ERROR_CHECK(err_code); 
}

/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    /*ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    //err_code = bsp_btn_ble_init(NULL, &startup_event);
    //APP_ERROR_CHECK(err_code);*/
    ret_code_t ret;
    ret = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(ret);

    INIT_BSP_ASSIGN_RELEASE_ACTION(BTN_MOUSE_LEFT);
    INIT_BSP_ASSIGN_RELEASE_ACTION(BTN_MOUSE_RIGHT);

    /* Configure LEDs */
    bsp_board_init(BSP_INIT_LEDS);
}

static void mouse_move_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    bool used = false;

    if (bsp_button_is_pressed(BTN_MOUSE_X_POS))
    {
        hid_generic_mouse_action(HID_GENERIC_MOUSE_X, CONFIG_MOUSE_MOVE_SPEED);
        used = true;
       // printf("mx\n");
    }
    if (bsp_button_is_pressed(BTN_MOUSE_Y_POS))
    {
        hid_generic_mouse_action(HID_GENERIC_MOUSE_Y, CONFIG_MOUSE_MOVE_SPEED);
        used = true;
    }

    if(!used)
    {
        UNUSED_RETURN_VALUE(app_timer_stop(m_mouse_move_timer));
    }
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_mouse_move_timer, APP_TIMER_MODE_REPEATED, mouse_move_timer_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
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


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**
 * @brief Class specific event handler.
 *
 * @param p_inst    Class instance.
 * @param event     Class specific event.
 * */
static void hid_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                app_usbd_hid_user_event_t event)
{
    switch (event)
    {
        case APP_USBD_HID_USER_EVT_OUT_REPORT_READY:
        {
            /* No output report defined for this example.*/
            ASSERT(0);
            break;
        }
        case APP_USBD_HID_USER_EVT_IN_REPORT_DONE:
        {
            m_report_pending = false;
            hid_generic_mouse_process_state();
            bsp_board_led_invert(LED_HID_REP_IN);
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_BOOT_PROTO:
        {
            UNUSED_RETURN_VALUE(hid_generic_clear_buffer(p_inst));
            NRF_LOG_INFO("SET_BOOT_PROTO");
            break;
        }
        case APP_USBD_HID_USER_EVT_SET_REPORT_PROTO:
        {
            UNUSED_RETURN_VALUE(hid_generic_clear_buffer(p_inst));
            NRF_LOG_INFO("SET_REPORT_PROTO");
            break;
        }
        default:
            break;
    }
}

/**
 * @brief USBD library specific event handler.
 *
 * @param event     USBD library event.
 * */
static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SOF:
            break;
        case APP_USBD_EVT_DRV_RESET:
            m_report_pending = false;
            break;
        case APP_USBD_EVT_DRV_SUSPEND:
            m_report_pending = false;
            app_usbd_suspend_req(); // Allow the library to put the peripheral into sleep mode
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_DRV_RESUME:
            m_report_pending = false;
            bsp_board_led_on(LED_USB_START);
            break;
        case APP_USBD_EVT_STARTED:
            m_report_pending = false;
            bsp_board_led_on(LED_USB_START);
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}



static ret_code_t idle_handle(app_usbd_class_inst_t const * p_inst, uint8_t report_id)
{
    switch (report_id)
    {
        case 0:
        {
            uint8_t report[] = {0xBE, 0xEF};
            return app_usbd_hid_generic_idle_report_set(
              &m_app_hid_generic,
              report,
              sizeof(report));
        }
        default:
            return NRF_ERROR_NOT_SUPPORTED;
    }
    
}


int main(void)
{
     ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };
    // Initialize.
    nrf_gpio_cfg_output(push_btn);
    nrf_gpio_pin_set(push_btn);
    nrf_gpio_cfg_output(BLUE_LED);
    nrf_gpio_pin_set(BLUE_LED);
    nrf_gpio_cfg_output(RED_LED);
    nrf_gpio_pin_set(RED_LED);
    timer1_init();
    timer0_init();
    log_init();
    timer_init();    
    buttons_leds_init();
    ret = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(ret);
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    NRF_LOG_INFO("USBD BLE UART example started.");

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);
    app_usbd_class_inst_t const * class_inst_generic;
    class_inst_generic = app_usbd_hid_generic_class_inst_get(&m_app_hid_generic);

    ret = hid_generic_idle_handler_set(class_inst_generic, idle_handle);
    APP_ERROR_CHECK(ret);

    ret = app_usbd_class_append(class_inst_generic);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
    }

    db_discovery_init();
    power_management_init();
    ble_stack_init(); 
    gatt_init();
    scan_init();
    nus_c_init();
    dfu_init();

    // Start execution.
    //printf("BLE UART central example started.\r\n");
    NRF_LOG_INFO("BLE UART central example started.");
    scan_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
        nrf_delay_ms(15);
        // mousex = -((accX +100)/300);
        //mousey = (accY-100)/300;
        m_mouse_state.acc_x += (accX * SENSE);
        m_mouse_state.acc_y += (accY * SENSE);
        NRF_LOG_INFO("Battery Percentage: %d, %d, %d",batt_per,accX,accY);
        if((batt_per <= 15) && (connection_status == 1)) 
        {
          //NRF_LOG_INFO("Battery Percentage: %d, %d, %d",batt_per,accX,accY);
          //NRF_LOG_INFO("led_alert = 1")
          led_alert = 1;
        }
        ////m_mouse_state.acc_x += (mousex * SENSE);
        //m_mouse_state.acc_y += (mousey * SENSE);
        //NRF_LOG_INFO("%d :  %d   accX : %d  accY : %d",accX,accY,mousex,mousey);
         while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
        hid_generic_mouse_process_state();       
    }
}
