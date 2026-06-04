#include "usbd_cdc_if.h"
#include <stdio.h>
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

int _write(int file, char *ptr, int len)
{
    while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED);

    while (CDC_Transmit_FS((uint8_t*)ptr, len) == USBD_BUSY)
    {
        // chờ gửi xong
    }
    
    CDC_Transmit_FS((uint8_t*)ptr, len);
    return len;
}