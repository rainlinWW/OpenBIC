/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <kernel.h>
#include <cmsis_os2.h>
#include <sys/printk.h>
#include "worker.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "hal_gpio.h"
#include "ipmi.h"

void main(void)
{
  I2C_MSG *msg;
  uint8_t ret, retry = 5;
  uint8_t status;

	printk("Hello yv35 cl\n");

  util_init_I2C();

  gpio_init();
	sensor_init();
	ipmi_init();

}
