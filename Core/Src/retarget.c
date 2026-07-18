#include "usbd_cdc_if.h"
#include <stdio.h>
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

int _write(int file, char *ptr, int len)
{
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return len;
    }

    uint32_t timeout_tick = HAL_GetTick() + 10;
    while (CDC_Transmit_FS((uint8_t*)ptr, len) == USBD_BUSY)
    {
        if (HAL_GetTick() > timeout_tick)
        {
            break;
        }
    }
    return len;
}