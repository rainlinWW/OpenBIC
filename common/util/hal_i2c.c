#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os2.h"
#include "hal_dev.h"
#include "hal_i2c.h"
#include "timer.h"

static const struct device *dev_i2c[MAX_I2C_BUS_NUM];


struct k_mutex i2c_mutex[MAX_I2C_BUS_NUM];

bool i2c_master_read(I2C_MSG *msg, uint8_t retry) {
  uint8_t i;
  uint8_t txbuf[I2C_BUFF_SIZE], rxbuf[I2C_BUFF_SIZE];
  int ret, status;

  if (DEBUG_I2C) {
    printf("i2c_master_read: bus %d, addr %x, rxlen %d, txlen %d, txbuf:", msg->bus, msg->slave_addr, msg->rx_len, msg->tx_len);
    for (int i = 0; i < msg->tx_len; i++) {
      printf(" %x",msg->data[i]);
    }
    printf("\n");
  }

  if(msg->rx_len == 0) {
    printf("i2c_master_read with rx_len = 0\n");
    return false;
  }

  do { // break while getting mutex success but tranmission fail
    status = k_mutex_lock(&i2c_mutex[msg->bus], K_MSEC(1000));
    if (status == osOK) {
      for (i = 0; i < retry; i++) {
        memcpy(txbuf, &msg->data[0], msg->tx_len);

        ret = i2c_write_read(dev_i2c[msg->bus], msg->slave_addr, txbuf, msg->tx_len, rxbuf, msg->rx_len);

        memcpy(&msg->data[0], rxbuf, msg->rx_len);

        if (DEBUG_I2C) {
          printf("rxbuf:");
          for (int i = 0; i < msg->rx_len; i++) {
            printf(" %x",msg->data[i]);
          }
          printf("\n");
        }

        status = k_mutex_unlock(&i2c_mutex[msg->bus]);
        if (status != osOK) {
          printf("I2C %d master read release mutex fail\n",msg->bus);
        }
        return ret; // i2c write and read success
      }
      printf("I2C %d master read retry reach max\n",msg->bus);
      status = k_mutex_unlock(&i2c_mutex[msg->bus]);
      if (status != osOK) {
        printf("I2C %d master read release mutex fail\n",msg->bus);
      }
      return false;
    } else {
      printf("I2C %d master read get mutex timeout\n",msg->bus);
      return false;
    }
  } while(0);

  
  return false; // should not reach here
}

bool i2c_master_write(I2C_MSG *msg, uint8_t retry) {
  uint8_t i;
  uint8_t txbuf[I2C_BUFF_SIZE];
  int status;

  if (DEBUG_I2C) {
    printf("i2c_master_write: bus %d, addr %x, txlen %d, txbuf:",msg->bus, msg->slave_addr, msg->tx_len);
    for (int i = 0; i < msg->tx_len; i++) {
      printf(" %x",msg->data[i]);
    }
    printf("\n");
  }

  status = k_mutex_lock(&i2c_mutex[msg->bus], K_MSEC(1000));
  if (status == osOK) {
    for (i = 0; i < retry; i++) {
      memcpy(txbuf, &msg->data[0], msg->tx_len);
      if ( i2c_write(dev_i2c[msg->bus], txbuf, msg->tx_len, msg->slave_addr) ) {
        continue;        
      } else { // i2c write success
        status = k_mutex_unlock(&i2c_mutex[msg->bus]);
        if (status != osOK) {
          printf("I2C %d master write release mutex fail\n",msg->bus);
        }
        return true;
      }
    }
    printf("I2C %d master write retry reach max\n",msg->bus);
    status = k_mutex_unlock(&i2c_mutex[msg->bus]);
    if (status != osOK) {
      printf("I2C %d master write release mutex fail\n",msg->bus);
    }
    return false;

  } else {
    printf("I2C %d master write get mutex timeout\n",msg->bus);
    return false;
  }

  return false;
}

void util_init_I2C(void) {
  int status;

#ifdef DEV_I2C_0
  dev_i2c[0] = device_get_binding("I2C_0");
  status = k_mutex_init(&i2c_mutex[0]);
  if (status)
    printk("i2c0 mutex init fail\n");
#endif
#ifdef DEV_I2C_1
  dev_i2c[1] = device_get_binding("I2C_1");
  status = k_mutex_init(&i2c_mutex[1]);
  if (status)
    printk("i2c1 mutex init fail\n");
#endif
#ifdef DEV_I2C_2
  dev_i2c[2] = device_get_binding("I2C_2");
  status = k_mutex_init(&i2c_mutex[2]);
  if (status)
    printk("i2c2 mutex init fail\n");
#endif
#ifdef DEV_I2C_3
  dev_i2c[3] = device_get_binding("I2C_3");
  status = k_mutex_init(&i2c_mutex[3]);
  if (status)
    printk("i2c3 mutex init fail\n");
#endif
#ifdef DEV_I2C_4
  dev_i2c[4] = device_get_binding("I2C_4");
  status = k_mutex_init(&i2c_mutex[4]);
  if (status)
    printk("i2c4 mutex init fail\n");
#endif
#ifdef DEV_I2C_5
  dev_i2c[5] = device_get_binding("I2C_5");
  status = k_mutex_init(&i2c_mutex[5]);
  if (status)
    printk("i2c5 mutex init fail\n");
#endif
#ifdef DEV_I2C_6
  dev_i2c[6] = device_get_binding("I2C_6");
  status = k_mutex_init(&i2c_mutex[6]);
  if (status)
    printk("i2c6 mutex init fail\n");
#endif
#ifdef DEV_I2C_7
  dev_i2c[7] = device_get_binding("I2C_7");
  status = k_mutex_init(&i2c_mutex[7]);
  if (status)
    printk("i2c7 mutex init fail\n");
#endif
#ifdef DEV_I2C_8
  dev_i2c[8] = device_get_binding("I2C_8");
  status = k_mutex_init(&i2c_mutex[8]);
  if (status)
    printk("i2c8 mutex init fail\n");
#endif
#ifdef DEV_I2C_9
  dev_i2c[9] = device_get_binding("I2C_9");
  status = k_mutex_init(&i2c_mutex[9]);
  if (status)
    printk("i2c9 mutex init fail\n");
#endif
}
