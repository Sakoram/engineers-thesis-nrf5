#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"
#include "nrf_drv_timer.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"

#include "boards.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "simulation.h"
#include "doa.h"

#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */



NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
static doa_t m_doa;                                                     /**< DoA module instance. */
const nrfx_timer_t m_timer_doa = NRF_DRV_TIMER_INSTANCE(1);

static ble_uuid_t const some_random_uuid =
{
    .uuid = 0xABCD,
    .type = 1
};



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

/**
 * @brief Handler for timer events.
 */
static void timer_doa_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    //printf("@TIMER@ %d \r\n", event_type);
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
         
            doa_receive_RSS_timeout(&m_doa);
            break;

        default:
            //Do nothing.
            break;
    }
}

/**@brief Function initializing timer used for doa packet timeout.
 *
 * @param[in]   uint32_t   Time(in miliseconds) between consecutive compare events.
 */
static void timer_doa_init(uint32_t time_ms)
{
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;

    //Configure m_timer_doa
    nrfx_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;

    err_code = nrf_drv_timer_init(&m_timer_doa, &timer_cfg, timer_doa_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrfx_timer_ms_to_ticks(&m_timer_doa, time_ms);


        nrf_drv_timer_extended_compare(
         &m_timer_doa, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrf_drv_timer_enable(&m_timer_doa);
}

/**@brief Function to start scanning. */
static void scan_start(void)
{
    //!!printf("scan start\r\n");
    NRF_LOG_INFO("scan start");
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

}

/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;
    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
        {
            //printf("Packet from %02x%02x%02x%02x%02x%02x \r\n",
            //        p_scan_evt->params.p_whitelist_adv_report->peer_addr.addr[0],
            //        p_scan_evt->params.p_whitelist_adv_report->peer_addr.addr[1],
            //        p_scan_evt->params.p_whitelist_adv_report->peer_addr.addr[2],
            //        p_scan_evt->params.p_whitelist_adv_report->peer_addr.addr[3],
            //        p_scan_evt->params.p_whitelist_adv_report->peer_addr.addr[4],
            //        p_scan_evt->params.p_whitelist_adv_report->peer_addr.addr[5]
            //        );
            doa_receive_RSS(&m_doa, p_scan_evt->params.p_whitelist_adv_report->rssi);
            nrf_drv_timer_clear(&m_timer_doa);
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

    memset(&init_scan, 0, sizeof(init_scan));

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);


    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &some_random_uuid);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    APP_ERROR_CHECK(err_code);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
static void uart_event_handle(app_uart_evt_t * p_event)
{
   static uint8_t data_array[20];
    static uint16_t index = 0;
    uint32_t ret_val;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
        {
        
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= (20))) 
            {
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);
                if (memcmp(data_array, "next\r",index) == 0)
                {
                    bsp_board_leds_off();
                    uint16_t angle = doa_inc_angle(&m_doa);
                    //!!printf("NEXT ENGLE SET!! engle == %d\r\n", angle);
                }
                else if (memcmp(data_array, "prev\r",index) == 0)
                {
                    bsp_board_leds_off();
                    uint16_t angle = doa_dec_angle(&m_doa);
                    //!!printf("PREVIOUS ENGLE SET!! engle == %d\r\n", angle);
                }
                index = 0;
            }
        } 
        break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("Communication error occurred while handling UART.");
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
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
    // ble events discarded. 
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

/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

//static void bsp_event_handler(bsp_event_t evt)
//{
//    uint32_t err_code;
//    uint16_t angle;
//    switch(evt)
//    {
//        case BSP_EVENT_KEY_0:
//            bsp_board_leds_off();
//            angle = doa_inc_angle(&m_doa);
//            printf("NEXT ENGLE SET!! engle == %d\r\n", angle);
//        break;

//        case BSP_EVENT_KEY_1:
//            bsp_board_leds_off();
//            angle = doa_dec_angle(&m_doa);
//            printf("PREVIOUS ENGLE SET!! engle == %d\r\n", angle);
//        break;

//        default:
//            return;
//    }
//}

// create  a function to configure all the leds and buttons 
//static void bsp_configure(void)
//{
//    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
//    bsp_board_leds_off();
//    APP_ERROR_CHECK(err_code);
//}

/**@brief Function for initializing the app timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void doa_handler(doa_evt_t const * p_evt)
{
    switch(p_evt->evt_id)
    {
        case DOA_DIRECTION_ESTIMATED:
            //!!printf("DOA_DIRECTION_ESTIMATED == %d \r\n", p_evt->param.direction);
            NRF_LOG_DEBUG("DOA_DIRECTION_ESTIMATED == %d \r\n", p_evt->param.direction);
            break;

        case DOA_CALIBRATION_END:
            if (p_evt->param.calibration_successful)
            {
                //!!printf("DOA_CALIBRATION SUCCESS! \r\n");
                //bsp_board_led_on(2); 
            }
            else
            {
                //!!printf("DOA_CALIBRATION FAIL! \r\n");
                //bsp_board_led_on(0); 
            }
            break;

        case DOA_ANGLE_SETUP_REQUIRED:
            //!!printf("DOA_ANGLE_SETUP_REQUIRED \r\n");
            //bsp_board_led_on(3); 
            break;

        case DOA_PACKET_TIMEOUT:
            //!!printf("DOA_PACKET_TIMEOUT \r\n");
            //bsp_board_led_on(1);
            break;

        default:
            break;
    }
}

int main(void)
{
    log_init();
   // uart_init();
    ble_stack_init();
    scan_init();
    doa_init(&m_doa, doa_handler);
    timer_init();
    //bsp_configure();
    timer_doa_init(1000);
    scan_start();

    doa_start_calibration(&m_doa, false);

#if 1
    simulation_send_calibration_packets(&m_doa);
    for(int i = 0; i< 25; i++)
    {
       uint16_t simulated_dir = simulation_send_measurement_packets(&m_doa);
       //!!printf("simulated_dir == %d .\r\n", simulated_dir);
       //nrf_drv_timer_disable(&m_timer_doa);
    }
#endif
    //TODO flash storage for configs

    // Enter main loop.
    for (;;)
    {
        //__WFI();
        //__WFE();
    }
}
