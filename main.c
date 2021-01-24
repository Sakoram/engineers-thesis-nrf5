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
#ifdef DOA_GAIN_SNG_180
static doa_t m_doa = DOA_GAIN_SNG_180;
#else
static doa_t m_doa;
#endif
const nrfx_timer_t m_timer_doa = NRF_DRV_TIMER_INSTANCE(1);

static ble_uuid_t const some_random_uuid =
{
    .uuid = 0xABCD,
    .type = 1
};

static ble_gap_scan_params_t const m_scan_param =
{
    .active        = 0x01,
    .interval      = 0x100,
    .window        = 0x100,
    .filter_policy  = 0,
    .timeout       = 0,
    .scan_phys     = BLE_GAP_PHY_1MBPS,
    .extended      = 0,
    .channel_mask  = {0,0,0,0,0xA0},
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
            if(p_scan_evt->params.p_whitelist_adv_report->ch_index == 38)
            {
                NRF_LOG_INFO("SAMPLE for ch %d rssi %d", p_scan_evt->params.p_whitelist_adv_report->ch_index, p_scan_evt->params.p_whitelist_adv_report->rssi);
                doa_receive_RSS(&m_doa, p_scan_evt->params.p_whitelist_adv_report->rssi);
                nrf_drv_timer_clear(&m_timer_doa);
            }
                
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
    init_scan.p_scan_param = &m_scan_param;

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
            double* T = p_evt->param.direction_info.T;
            uint16_t size = p_evt->param.direction_info.size;
            usb_print("size = %d \r\n", size);
            for(uint16_t i = 0; i < size; i += 4)
            {
                usb_print("T%d:%.7f %.7f %.7f %.7f \r\n",
                    i, T[i], T[i + 1], T[i + 2], T[i + 3]);
            }
            usb_print("DIRECTION ESTIMATED = %d \r\n", p_evt->param.direction_info.direction);
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
            uint16_t angle   = p_evt->param.calibration_samples_per_angle.angle;
            uint8_t configs_num = p_evt->param.calibration_samples_per_angle.configs_num;
            int8_t* samples = p_evt->param.calibration_samples_per_angle.samples;
            for(uint8_t i = 0; i < configs_num; i++)
            {
                usb_print("sample-A angle %d, config %d = %d \r\n", 
                     angle , i, samples[i] );
            }
            #endif
            break;
        }
         case DOA_CALIBRATION_CONFIG_MEASURED:
        {
            #if DOA_SIMULATION_USED == false
            uint16_t angle   = p_evt->param.calibrationg_samples_per_config.angle;
            uint8_t config   = p_evt->param.calibrationg_samples_per_config.config;
            uint8_t size = p_evt->param.calibrationg_samples_per_config.samples_size;
            int8_t* samples = p_evt->param.calibrationg_samples_per_config.samples;
            for(uint8_t i = 0; i < size; i++)
            {
                usb_print("sample-C angle %d, config %d = %d \r\n", 
                     angle , config, samples[i] );
            }
            #endif
            break;
        }
        case DOA_MEASUREMENTS_CONFIG_MEASURED:
        {
            uint16_t config = p_evt->param.measurements_sample_per_config.config;
            float sample  = p_evt->param.measurements_sample_per_config.sample;
            usb_print("sample-M config %d, sample_dBm %f \r\n", config, sample);
        }
        break;
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
    else if (memcmp(payload, "doa\r", size) == 0)
    {
        doa_estimate(&m_doa);
        usb_print("estimating\r\n");
        
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
    doa_init(&m_doa, doa_handler, DOA_WITHOUT_CALIBRATION); //DOA_MANUAL_CALIBRATION
    log_resetreason();

#if DOA_SIMULATION_USED == true
    simulation_send_calibration_packets(&m_doa);
    for(int i = 0; i< 25; i++)
    {
       uint16_t simulated_dir = simulation_send_measurement_packets(&m_doa);
       usb_print("random dir == %d \r\n", simulated_dir);
    }
#endif
    timer_doa_init(1000);
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
