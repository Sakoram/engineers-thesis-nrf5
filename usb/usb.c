#include "usb.h"
#include "nrf_drv_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include <stdarg.h>
#include "app_timer.h"

#include "nrf_log.h"


/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif
#define CDC_READ_SIZE 32

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1


static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

static char m_rx_buffer[CDC_READ_SIZE];


/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);
typedef struct{
    uint8_t size;
    char payload[48];
} message_t;

#define QUEUE_SIZE 16
APP_TIMER_DEF(m_usb_timer);
static volatile bool usb_ready = 0;
static volatile uint8_t tail = 0;//QUEUE_SIZE -1;
static volatile uint8_t head = 0;
static volatile uint8_t usb_free = 1; //For some weird reason APP_USBD_CDC_ACM_USER_EVT_TX_DONE doesn't fire after the first message is sent.
static message_t message_queue[QUEUE_SIZE];
static usb_evt_handler_t  usb_evt_handler;  

static void usb_timer_handler(void* p_context)
{
    usb_ready = 1;
    if (tail == head) //queue empty
    {
        usb_free = 1; 
    }
    else 
    {
       // NRF_LOG_DEBUG("sending"); //debug 
        app_usbd_cdc_acm_write(&m_app_cdc_acm, message_queue[tail].payload, message_queue[tail].size);
    }
}

void usb_print(const char *format, ...)
{ 

    NRF_LOG_DEBUG("usb_print head %d tail %d usb_free %d", head, tail, usb_free);
    //NRF_LOG_DEBUG("message send:");
    //NRF_LOG_DEBUG(format);
    uint8_t temp_head = (head + 1) % QUEUE_SIZE;
    if (temp_head == tail)
        return; // queue full

    va_list args;
    va_start(args, format);
    message_queue[head].size = vsprintf(message_queue[head].payload, format, args);
    va_end(args);
    head = temp_head;
    if (usb_free && usb_ready)
    {
        //NRF_LOG_DEBUG("sending"); //debug 
        app_usbd_cdc_acm_write(&m_app_cdc_acm, message_queue[tail].payload, message_queue[tail].size);
        usb_free = 0;
    }


    
}


static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            NRF_LOG_DEBUG("APP_USBD_EVT_DRV_SUSPEND");
            break;
        case APP_USBD_EVT_DRV_RESUME:
            NRF_LOG_DEBUG("APP_USBD_EVT_DRV_RESUME");
            break;
        case APP_USBD_EVT_STARTED:
            NRF_LOG_DEBUG("APP_USBD_EVT_STARTED");
            break;
        case APP_USBD_EVT_STOPPED:
            NRF_LOG_DEBUG("APP_USBD_EVT_STOPPED");
            app_usbd_disable();
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

/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
           /*Setup first transfer*/
           ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   1);
            ret = app_timer_create(&m_usb_timer, APP_TIMER_MODE_SINGLE_SHOT, usb_timer_handler);
            APP_ERROR_CHECK(ret);
            ret = app_timer_start(m_usb_timer, APP_TIMER_TICKS(50), 0);
            APP_ERROR_CHECK(ret);
            break;
        }
        //case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
        //    break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
           NRF_LOG_DEBUG("APP_USBD_CDC_ACM_USER_EVT_TX_DONE"); //debug 
            tail = (tail + 1) % QUEUE_SIZE;
            if (tail == head) //queue empty
            {
                usb_free = 1; 
            }
            else 
            {
               // NRF_LOG_DEBUG("sending"); //debug 
                app_usbd_cdc_acm_write(&m_app_cdc_acm, message_queue[tail].payload, message_queue[tail].size);
            }
                
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            static uint8_t index = 0;
            index++;

            do
            {
                if ((m_rx_buffer[index - 1] == '\n') ||
                    (m_rx_buffer[index - 1] == '\r') ||
                    (index >= CDC_READ_SIZE))
                {
                    if (index > 1)
                    {
                        NRF_LOG_HEXDUMP_DEBUG(m_rx_buffer, index);
                        usb_evt_t evt;
                        evt.evt_id = USB_MESSAGE_RECEIVED;
                        evt.param.usb_message.payload = m_rx_buffer;
                        evt.param.usb_message.size    = index;
                        usb_evt_handler(&evt);
                    }

                    index = 0;
                }

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            &m_rx_buffer[index],
                                            1);
                if (ret == NRF_SUCCESS)
                {
                    index++;
                }
            }
            while (ret == NRF_SUCCESS);

            break;
        }
        default:
            break;
    }
}

void usb_init(usb_evt_handler_t evt_handler)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };
    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);
    NRF_LOG_INFO("USBD started.");

    app_usbd_serial_num_generate();

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);
    usb_evt_handler = evt_handler;

#if USBD_POWER_DETECTION == true

        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);

#else

        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();

#endif
}
