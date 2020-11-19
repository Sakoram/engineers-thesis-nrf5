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

#include "doa.h"

#include "nrf_drv_clock.h"
//#include "nrf_gpio.h"
#include "nrf_drv_power.h"

//#include "bsp.h"
#include "usb.h"
#include "app_usbd.h"



#include "simulation.h"


#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */


//NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

/**< DoA module instance. */
#ifdef DOA_DEFAULT_VALUE
static doa_t m_doa = DOA_DEFAULT_VALUE
   // { .gain_samples = 
   //{{186, 184, 184, 183, 184, 186, 184, 185, 187, 187, 188, 188},
   // {189, 185, 185, 187, 185, 185, 184, 187, 188, 187, 190, 190},
   // {190, 188, 185, 187, 185, 190, 187, 187, 188, 188, 190, 191},
   // {193, 190, 186, 187, 188, 185, 188, 186, 188, 189, 191, 193},
   // {194, 193, 190, 187, 190, 190, 188, 187, 189, 191, 193, 194},
   // {195, 194, 190, 188, 188, 188, 188, 187, 187, 190, 193, 194},
   // {194, 194, 190, 187, 184, 187, 188, 188, 187, 188, 191, 193},
   // {191, 193, 191, 189, 184, 187, 187, 188, 185, 188, 189, 191},
   // {194, 194, 193, 189, 187, 187, 187, 188, 189, 191, 190, 193},
   // {195, 194, 194, 191, 188, 187, 185, 188, 189, 190, 192, 193},
   // {194, 195, 194, 193, 191, 189, 188, 189, 191, 192, 192, 194},
   // {194, 195, 196, 195, 193, 189, 188, 188, 189, 191, 192, 195},
   // {193, 194, 196, 194, 193, 191, 189, 186, 185, 189, 189, 192},
   // {193, 193, 195, 194, 193, 189, 186, 185, 186, 188, 187, 188},
   // {193, 194, 195, 196, 194, 191, 188, 186, 187, 184, 188, 189},
   // {192, 194, 195, 197, 196, 194, 190, 189, 191, 191, 192, 191},
   // {193, 194, 195, 196, 196, 195, 192, 191, 190, 191, 192, 192},
   // {193, 193, 194, 195, 195, 194, 192, 188, 188, 187, 187, 191},
   // {192, 193, 192, 194, 194, 193, 192, 189, 188, 187, 187, 190},
   // {192, 193, 194, 194, 194, 195, 194, 191, 189, 187, 188, 190},
   // {191, 191, 193, 194, 195, 194, 194, 193, 190, 189, 189, 189},
   // {189, 190, 192, 193, 194, 196, 195, 192, 189, 189, 187, 187},
   // {189, 190, 191, 193, 194, 195, 194, 193, 190, 189, 188, 189},
   // {187, 189, 190, 192, 192, 193, 193, 193, 190, 187, 184, 187},
   // {184, 185, 186, 188, 189, 190, 192, 192, 191, 188, 183, 181},
   // {184, 183, 185, 185, 187, 188, 190, 189, 189, 188, 184, 184},
   // {185, 186, 186, 187, 187, 191, 190, 190, 188, 187, 185, 186},
   // {186, 185, 185, 184, 187, 190, 190, 191, 191, 191, 188, 184},
   // {186, 189, 188, 184, 186, 189, 190, 191, 193, 192, 192, 190},
   // {185, 186, 186, 189, 191, 192, 192, 194, 195, 195, 194, 190},
   // {184, 187, 186, 187, 188, 190, 191, 193, 194, 194, 193, 189},
   // {188, 186, 186, 187, 187, 190, 191, 193, 194, 194, 192, 189},
   // {189, 185, 184, 187, 185, 188, 190, 192, 193, 194, 194, 192},
   // {188, 186, 187, 187, 187, 185, 182, 188, 189, 193, 193, 191},
   // {185, 186, 186, 190, 189, 189, 187, 189, 190, 191, 192, 189},
   // {184, 184, 187, 181, 181, 183, 182, 190, 189, 187, 189, 189}}};
#else
static doa_t m_doa;
#endif
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
    switch(p_evt->evt_id)
    {
        case DOA_DIRECTION_ESTIMATED:
        {
            usb_print("DIRECTION ESTIMATED = %d \r\n", p_evt->param.direction);


            break;
        }
        case DOA_CALIBRATION_END:
        {
            if (p_evt->param.calibration_successful)
            {
                usb_print("CALIBRATION SUCCESS! \r\n");
            }
            else
            {
                usb_print("CALIBRATION FAIL! \r\n");
            }
            break;
        }
        case DOA_ANGLE_SETUP_REQUIRED:
        {
            usb_print("ANGLE SETUP REQUIRED \r\n");
            break;
        }

        case DOA_PACKET_TIMEOUT:
        {
            usb_print("PACKET TIMEOUT \r\n");
            break;
        }
        case DOA_WAITING_FOR_START:
        {
            usb_print("WAITING FOR \"start\" \r\n");
            break;
        }
        case DOA_ANGLE_MEASURED:
        {
            #if DOA_SIMULATION_USED == false
            uint16_t angle   = p_evt->param.calibration_samples.angle;
            uint8_t* samples = p_evt->param.calibration_samples.samples;
            for(int i = 0; i < ANTENNA_CONFIGS_NUM; i++)
            {
                usb_print("sample angle %d, config %d = %d \r\n", 
                     angle , i, samples[i] );
            }
            #endif
            break;
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
    usb_print("Reset reasons: \r\n");
    if (0 == rr)
    {
        NRF_LOG_INFO("- NONE");
        usb_print("- NONE \r\n");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_RESETPIN_MASK))
    {
        NRF_LOG_INFO("- RESETPIN");
        usb_print("- RESETPIN \r\n");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_DOG_MASK     ))
    {
        NRF_LOG_INFO("- DOG");
        usb_print("- DOG \r\n");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_SREQ_MASK    ))
    {
        NRF_LOG_INFO("- SREQ");
        usb_print("- SREQ \r\n");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_LOCKUP_MASK  ))
    {
        NRF_LOG_INFO("- LOCKUP");
        usb_print("- LOCKUP \r\n");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_OFF_MASK     ))
    {
        NRF_LOG_INFO("- OFF");
        usb_print("- OFF \r\n");
    }
#if defined(NRF_POWER_RESETREAS_LPCOMP_MASK)
    if (0 != (rr & NRF_POWER_RESETREAS_LPCOMP_MASK  ))
    {
        NRF_LOG_INFO("- LPCOMP");
        usb_print("- LPCOMP \r\n");
    }
#endif
    if (0 != (rr & NRF_POWER_RESETREAS_DIF_MASK     ))
    {
        NRF_LOG_INFO("- DIF");
        usb_print("- DIF \r\n");
    }
#if defined(NRF_POWER_RESETREAS_NFC_MASK)
    if (0 != (rr & NRF_POWER_RESETREAS_NFC_MASK     ))
    {
        NRF_LOG_INFO("- NFC");
        usb_print("- NFC \r\n");
    }
#endif
    if (0 != (rr & NRF_POWER_RESETREAS_VBUS_MASK    ))
    {
        NRF_LOG_INFO("- VBUS");
        usb_print("- VBUS \r\n");
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
    else if (memcmp(payload, "reset!\r", size) == 0)
    {
        usb_print("RESET! \r\n"); // propobly wont be send
        NVIC_SystemReset();
    }
}

int main(void)
{
    nrf_delay_ms(100);
    log_init();
    clock_init();
    timer_init();
    usb_init(usb_handler);
    ble_stack_init();
    scan_init();
    doa_init(&m_doa, doa_handler, DOA_WITHOUT_CALIBRATION);
    timer_doa_init(1000);
    log_resetreason();

#if DOA_SIMULATION_USED == true
    simulation_send_calibration_packets(&m_doa);
    for(int i = 0; i< 25; i++)
    {
       uint16_t simulated_dir = simulation_send_measurement_packets(&m_doa);
       usb_print("random dir == %d \r\n", simulated_dir);
    }
#endif
    scan_start();

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
