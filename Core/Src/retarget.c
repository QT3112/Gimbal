#include "usbd_cdc_if.h"
#include <stdio.h>
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

int _write(int file, char *ptr, int len)
{
    /* Kiểm tra trạng thái USB. Nếu chưa được cấu hình (chưa cắm cáp hoặc chưa nhận diện xong),
       bỏ qua không gửi để tránh làm treo toàn bộ chương trình */
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return len;
    }

    /* Thử gửi dữ liệu. Nếu USB đang bận, chờ 1 khoảng thời gian (timeout) 
       để tránh vòng lặp vô hạn nếu có lỗi đường truyền */
    uint32_t timeout = 1000000; 
    while (CDC_Transmit_FS((uint8_t*)ptr, len) == USBD_BUSY)
    {
        timeout--;
        if (timeout == 0)
        {
            break; // Thoát nếu quá lâu không gửi được
        }
    }
    
    return len;
}