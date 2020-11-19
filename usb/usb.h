#ifndef __USB_H__
#define __USB_H__

#include <stdint.h>

typedef enum
{
    USB_MESSAGE_RECEIVED,
} usb_evt_id_t;

typedef struct
{
    usb_evt_id_t evt_id;       //!< Event ID.
    union // it is useless now, but maybe I will extend it in future
    {
        struct {
            uint8_t  size;
            char*    payload;
        } usb_message;
    } param;
} usb_evt_t;

typedef void (*usb_evt_handler_t) (usb_evt_t const * p_evt);

void usb_init(usb_evt_handler_t evt_handler);
void usb_print(const char *format, ...);

#endif // __USB_H__