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

	util_init_worker();
  util_init_I2C();

  gpio_init();
	sensor_init();
	ipmi_init();

  gpio_conf(0, 1);
  gpio_conf(2, 0);
  gpio_conf(31, 0);
  gpio_conf(32, 0);
  gpio_conf(33, 0);
  gpio_set(0, 1);

  gpio_cb_irq_init(2, GPIO_INT_EDGE_FALLING);
  gpio_cb_irq_init(31, GPIO_INT_EDGE_BOTH);
  gpio_cb_irq_init(32, GPIO_INT_EDGE_BOTH);
  gpio_cb_irq_init(33, GPIO_INT_EDGE_BOTH);
//while(1)
//	k_msleep(10000);

/*
  gpio_conf(1, 1);
  gpio_set(1, 1);

  gpio_init_cb(33);
  gpio_init_cb(2);
  gpio_add_cb(33);
  gpio_add_cb(2);
  gpio_conf(33, 0);
  gpio_conf(2, 0);
  gpio_interrupt_conf(33, GPIO_INT_EDGE_FALLING);
  gpio_interrupt_conf(2, GPIO_INT_EDGE_BOTH);
*/
  while(0) {
    if (!gpio_set(33, 0))
      printk("set ok\n");
    printk("gg %x\n", gpio_get(32));
    k_msleep(5000);
    if (!gpio_set(33, 1))
      printk("set ok\n");
    printk("gg %x\n", gpio_get(32));
    k_msleep(5000);
  }

	while (1) {
/*    msg = k_malloc(sizeof(I2C_MSG));
    msg->bus = 6;
    msg->slave_addr = 0x20;
    msg->data[0] = 0x18;
    msg->data[1] = 0xa8;
    msg->data[2] = 0x40;
    msg->data[3] = 0xd4;
    msg->data[4] = 0x01;
    msg->data[5] = 0xeb;
    msg->tx_len = 6;
    msg->rx_len = 1;
    ret = i2c_master_write(msg, retry);
    printk("ret: %d\n", ret);
    k_free(msg);*/
		printk("idle...\n");
		k_msleep(50000);
//    sys_reboot(0);
	}

}
