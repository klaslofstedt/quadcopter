#include "USB.h"
#include <stdio.h>
#include <delay.h>

void USB_Init(void)
{
	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
	setbuf(stdout, NULL);
	Delay(500000);
}
