#include <zephyr.h>
#include <stdio.h>
#include "ipmi.h"


void pal_APP_GET_DEVICE_ID(ipmi_msg *msg)
{
	if (msg->data_len != 0) {
		msg->completion_code = CC_INVALID_LENGTH;
		return;
	}

	msg->data[0] = 0xab;
	msg->data[1] = 0xcd;
	msg->data_len = 2;
	msg->completion_code = CC_SUCCESS;

	return;
}

