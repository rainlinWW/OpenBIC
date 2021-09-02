#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "plat_i2c.h"
#include "sensor.h"
#include "ipmi.h"
#include "plat_ipmi.h"
#include "hal_gpio.h"

bool add_sel_evt_record(addsel_msg_t *sel_msg) {
  ipmb_error status;
  ipmi_msg *msg;
  uint8_t system_event_record = 0x02; // IPMI spec definition
  uint8_t evt_msg_version = 0x04; // IPMI spec definition
  static uint16_t record_id = 0x1;

  // According to IPMI spec, record id 0h and FFFFh is reserved for special usage
  if ( (record_id == 0) || (record_id == 0xFFFF) ) {
    record_id = 0x1;
  }

  msg = (ipmi_msg*)malloc(sizeof(ipmi_msg));
  memset(msg, 0, sizeof(ipmi_msg));

  msg->data_len = 16;
  msg->InF_source = Self_IFs;
  msg->InF_target = BMC_IPMB_IFs;
  msg->netfn = NETFN_STORAGE_REQ;
  msg->cmd = CMD_STORAGE_ADD_SEL;

  msg->data[0] = (record_id & 0xFF);                   // record id byte 0, lsb
  msg->data[1] = ( (record_id >> 8) & 0xFF);           // record id byte 1
  msg->data[2] = system_event_record;                  // record type
  msg->data[3] = 0x00;                                 // timestamp, bmc would fill up for bic
  msg->data[4] = 0x00;                                 // timestamp, bmc would fill up for bic
  msg->data[5] = 0x00;                                 // timestamp, bmc would fill up for bic
  msg->data[6] = 0x00;                                 // timestamp, bmc would fill up for bic
  msg->data[7] = (Self_I2C_ADDRESS << 1);              // generator id
  msg->data[8] = 0x00;                                // generator id
  msg->data[9] = evt_msg_version;                     // event message format version
  msg->data[10] = 0x00;                                // sensor type, TBD
  msg->data[11] = sel_msg->snr_name;                   // sensor name
  msg->data[12] = sel_msg->evt_type;                   // sensor type
  msg->data[13] = sel_msg->evt_data1;                  // sensor data 1
  msg->data[14] = sel_msg->evt_data2;                  // sensor data 2
  msg->data[15] = sel_msg->evt_data3;                  // sensor data 3
  record_id++;

  status = ipmb_read(msg, IPMB_inf_index_map[msg->InF_target]);
  free(msg);
  if (status == ipmb_error_failure) {
    printf("Fail to post msg to txqueue for addsel\n");
    return false;
  } else if (status == ipmb_error_get_messageQueue) {
    printf("No response from bmc for addsel\n");
    return false;
  }

  return true;
}

bool pal_is_to_ipmi_handler(uint8_t netfn, uint8_t cmd) {
  if ( (netfn == NETFN_OEM_1S_REQ) && (cmd == CMD_OEM_FW_UPDATE) ) {
    return 1;
  }

  return 0;
}

bool pal_is_not_return_cmd(uint8_t netfn, uint8_t cmd) {
  if ( (netfn == NETFN_OEM_1S_REQ) && (cmd == CMD_OEM_MSG_OUT) ) {
    return 1;
  } else if ( (netfn == NETFN_OEM_1S_REQ) && (cmd == CMD_OEM_MSG_IN) ) {
    return 1;
  }

  // Reserve for future commands

  return 0;
}

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

void pal_APP_MASTER_WRITE_READ(ipmi_msg *msg) {
  uint8_t retry = 3;
  uint8_t bus_7bit;
  I2C_MSG i2c_msg;

  if (msg->data_len < 4) { // at least include bus, addr, rx_len, offset
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }

  bus_7bit = ( (msg->data[0]-1) >> 1); // should ignore bit0, all bus public
  i2c_msg.bus = i2c_bus_to_index[bus_7bit];
  i2c_msg.slave_addr = (msg->data[1] >> 1); // 8 bit address to 7 bit
  i2c_msg.rx_len = msg->data[2];
  i2c_msg.tx_len = msg->data_len - 3;

  if (i2c_msg.tx_len == 0) {
    msg->completion_code = CC_INVALID_DATA_FIELD;
    return;
  }

  memcpy(&i2c_msg.data[0], &msg->data[3], i2c_msg.tx_len);
  msg->data_len = i2c_msg.rx_len;

  if (i2c_msg.rx_len == 0) {
    if ( !i2c_master_write(&i2c_msg, retry) ) {
      msg->completion_code = CC_SUCCESS;
    } else {
      msg->completion_code = CC_I2C_BUS_ERROR;
    }
  } else {
    if ( !i2c_master_read(&i2c_msg, retry) ) {
      memcpy(&msg->data[0], &i2c_msg.data, i2c_msg.rx_len);
      msg->completion_code = CC_SUCCESS;
    } else {
      msg->completion_code = CC_I2C_BUS_ERROR;
    }
  }

  return;
}

void pal_SENSOR_GET_SENSOR_READING(ipmi_msg *msg) {
  uint8_t status, snr_num;
  int reading;

  if (msg->data_len != 1) {
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }

  if (!enable_sensor_poll) {
    printf("Reading sensor cache while sensor polling disable\n");
    msg->completion_code = CC_CAN_NOT_RESPOND;
    return;
  }

  snr_num = msg->data[0];
  status = get_sensor_reading(snr_num, &reading, get_from_cache); // Fix to get_from_cache. As need real time reading, use OEM command to get_from_sensor.

  switch (status) {
    case SNR_READ_SUCCESS:
      msg->data[0] = reading;
      // SDR sensor initialization bit6 enable scan, bit5 enable event
      // retunr data 1 bit 7 set to 0 to disable all event msg. bit 6 set to 0 disable sensor scan
      msg->data[1] = ( (full_sensor_table[SnrNum_SDR_map[snr_num]].sensor_init & 0x60) << 1 );
      msg->data[2] = 0xc0; // fix to threshold deassert status, BMC will compare with UCR/UNR itself
      msg->data_len = 3;
      msg->completion_code = CC_SUCCESS;
      break;
    case SNR_FAIL_TO_ACCESS:
      msg->completion_code = CC_NODE_BUSY; // transection error
      break;
    case SNR_NOT_ACCESSIBLE:
      msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE; // DC off
      break;
    case SNR_NOT_FOUND:
      msg->completion_code = CC_INVALID_DATA_FIELD; // request sensor number not found
      break;
    case SNR_UNSPECIFIED_ERROR:
    default :
      msg->completion_code = CC_UNSPECIFIED_ERROR; // unknown error
      break;
  }
  return;
}

void pal_OEM_MSG_OUT(ipmi_msg *msg) {
  uint8_t  target_IF;
  ipmb_error status;
  ipmi_msg *bridge_msg = (ipmi_msg*)malloc(sizeof(ipmi_msg));

  memset(bridge_msg, 0, sizeof(ipmi_msg));

  if (msg->completion_code != CC_INVALID_IANA) {
    msg->completion_code = CC_SUCCESS;
  }

  if (msg->data_len <= 2) { // Should input target, netfn, cmd
    msg->completion_code = CC_INVALID_LENGTH;
  }

  target_IF = msg->data[0];

  if ( (IPMB_config_table[IPMB_inf_index_map[target_IF]].Inf == Reserve_IFs) || (IPMB_config_table[IPMB_inf_index_map[target_IF]].EnStatus == Disable) ) { // Bridge to invalid or disabled interface
    printf("OEM_MSG_OUT: Invalid bridge interface: %x\n",target_IF);
    msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
  }

  if (msg->completion_code == CC_SUCCESS) { // only send to target while msg is valid
    if (DEBUG_IPMI) {
      printf("bridge targetIf %x, len %d, netfn %x, cmd %x\n", target_IF, msg->data_len, msg->data[1] >> 2, msg->data[2]);
    }

    bridge_msg->data_len = msg->data_len -3;
    bridge_msg->seq_source = msg->seq_source;
    bridge_msg->InF_target = msg->data[0];
    bridge_msg->InF_source = msg->InF_source;
    bridge_msg->netfn = msg->data[1] >> 2;
    bridge_msg->cmd = msg->data[2];

    if (bridge_msg->data_len != 0) {
      memcpy( &bridge_msg->data[0], &msg->data[3], bridge_msg->data_len * sizeof(msg->data[0]) );
    }

    status = ipmb_send_request(bridge_msg, IPMB_inf_index_map[target_IF]);

    if (status != ipmb_error_success) {
      printf("OEM_MSG_OUT send IPMB req fail status: %x",status);
      msg->completion_code = CC_BRIDGE_MSG_ERR;
    }

    free(bridge_msg);
  }

  if (msg->completion_code != CC_SUCCESS) { // Return to source while data is invalid or sending req to Tx task fail

    msg->data_len=0;
    status = ipmb_send_response(msg, IPMB_inf_index_map[msg->InF_source]);

    if (status != ipmb_error_success) {
      printf("OEM_MSG_OUT send IPMB resp fail status: %x",status);
    }
  }

  return;
}

void pal_OEM_GET_GPIO(ipmi_msg *msg) {
  if (msg->data_len != 0) { // only input enable status
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }
  
  uint8_t eight_bit_value, gpio_num, gpio_value, gpio_cnt, data_len;
  gpio_cnt = gpio_ind_to_num_table_cnt + (8 - (gpio_ind_to_num_table_cnt % 8)); // Bump up the gpio_ind_to_num_table_cnt to multiple of 8.
  data_len = gpio_cnt / 8;
  msg->data_len = data_len;
  for(uint8_t i = 0; i < gpio_cnt; i++) {
    gpio_value = (i >= gpio_ind_to_num_table_cnt) ? 0 : gpio_get(gpio_ind_to_num_table[i]);
    eight_bit_value = (eight_bit_value << 1) | gpio_value;
    msg->data[i / 8] = eight_bit_value;
  }
  msg->completion_code = CC_SUCCESS;
  return;
}
