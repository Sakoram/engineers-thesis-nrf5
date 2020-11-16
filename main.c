#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"
#include "app_util.h"
//#include "bsp_btn_ble.h"
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

#include "nrf_drv_clock.h"
//#include "nrf_gpio.h"
#include "nrf_drv_power.h"

//#include "bsp.h"
#include "usb.h"
#include "app_usbd.h"




#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */


//NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
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
    usb_print("scan start\r\n");
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


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing the app timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void doa_handler(doa_evt_t const * p_evt)
{
    size_t size;
    switch(p_evt->evt_id)
    {
        case DOA_DIRECTION_ESTIMATED:
        {
            //!!printf("DOA_DIRECTION_ESTIMATED == %d \r\n", p_evt->param.direction);
            usb_print("%d == DOA_DIRECTION_ESTIMATED  \r\n", p_evt->param.direction);


            break;
        }
        case DOA_CALIBRATION_END:
        {
            if (p_evt->param.calibration_successful)
            {
                //!!printf("DOA_CALIBRATION SUCCESS! \r\n");
                usb_print("DOA_CALIBRATION SUCCESS! \r\n");
            }
            else
            {
                //!!printf("DOA_CALIBRATION FAIL! \r\n");
                usb_print("DOA_CALIBRATION FAIL! \r\n");
            }
            break;
        }
        case DOA_ANGLE_SETUP_REQUIRED:
        {
            //!!printf("DOA_ANGLE_SETUP_REQUIRED \r\n");
           

            usb_print("angle = %d \r\n", m_doa.messered_angle );
            for(int i = 0; i < ANTENNA_CONFIGS_NUM; i++)
            {
                usb_print("sample from config %d = %d \r\n", i, m_doa.calibration_rss_samples[m_doa.messered_angle][i] );
            }
             usb_print("DOA_ANGLE_SETUP_REQUIRED \r\n");
        
            break;
        }

        case DOA_PACKET_TIMEOUT:
        {
            //!!printf("DOA_PACKET_TIMEOUT \r\n");
            usb_print("DOA_PACKET_TIMEOUT \r\n");
            break;
        }
        case DOA_WAITING_FOR_START:
        {
            usb_print("WAITING FOR \"start\" \r\n");
        }
        default:
            break;
    }
}

void clock_init()
{
    ret_code_t ret;
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
}

static void log_resetreason(void)
{
    /* Reset reason */
    uint32_t rr = nrf_power_resetreas_get();
    NRF_LOG_INFO("Reset reasons:");
    if (0 == rr)
    {
        NRF_LOG_INFO("- NONE");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_RESETPIN_MASK))
    {
        NRF_LOG_INFO("- RESETPIN");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_DOG_MASK     ))
    {
        NRF_LOG_INFO("- DOG");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_SREQ_MASK    ))
    {
        NRF_LOG_INFO("- SREQ");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_LOCKUP_MASK  ))
    {
        NRF_LOG_INFO("- LOCKUP");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_OFF_MASK     ))
    {
        NRF_LOG_INFO("- OFF");
    }
#if defined(NRF_POWER_RESETREAS_LPCOMP_MASK)
    if (0 != (rr & NRF_POWER_RESETREAS_LPCOMP_MASK  ))
    {
        NRF_LOG_INFO("- LPCOMP");
    }
#endif
    if (0 != (rr & NRF_POWER_RESETREAS_DIF_MASK     ))
    {
        NRF_LOG_INFO("- DIF");
    }
#if defined(NRF_POWER_RESETREAS_NFC_MASK)
    if (0 != (rr & NRF_POWER_RESETREAS_NFC_MASK     ))
    {
        NRF_LOG_INFO("- NFC");
    }
#endif
    if (0 != (rr & NRF_POWER_RESETREAS_VBUS_MASK    ))
    {
        NRF_LOG_INFO("- VBUS");
    }
}

void usb_handler(usb_evt_t const * p_evt)
{
    uint8_t size     = p_evt->param.usb_message.size;
    char*   payload  = p_evt->param.usb_message.payload;

    if (memcmp(payload, "next\r", size) == 0)
    {

        uint16_t angle = doa_inc_angle(&m_doa);
        usb_print("NEXT ENGLE SET!! engle == %d \r\n", angle);

    }
    else if (memcmp(payload, "prev\r", size) == 0)
    {
        uint16_t angle = doa_dec_angle(&m_doa);
        usb_print("PREVIOUS ENGLE SET!! engle == %d \r\n", angle);
    }
    else if (memcmp(payload, "start\r", size) == 0)
    {
        doa_start(&m_doa);
        usb_print("STARTED! \r\n");
    }
}

int main(void)
{
    nrf_delay_ms(100);
    log_init();
    log_resetreason();

    clock_init();
    nrf_delay_ms(100);
    usb_init(usb_handler);
    nrf_delay_ms(100);
    ble_stack_init();
    scan_init();
    doa_init(&m_doa, doa_handler);
    timer_init();
    timer_doa_init(1000);
    
    scan_start();

    doa_start_calibration(&m_doa, true);
#if 0
    simulation_send_calibration_packets(&m_doa);
    for(int i = 0; i< 25; i++)
    {
       uint16_t simulated_dir = simulation_send_measurement_packets(&m_doa);
       //!!printf("simulated_dir == %d .\r\n", simulated_dir);
       //nrf_drv_timer_disable(&m_timer_doa);
    }
#endif

    // Enter main loop.
    for (;;)
    {

        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
        //idle_state_handle();
        //__WFI();
        //__SEV();
    }
}
