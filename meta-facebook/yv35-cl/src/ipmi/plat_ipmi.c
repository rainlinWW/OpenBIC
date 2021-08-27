#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "plat_i2c.h"
#include "sensor.h"
#include "ipmi.h"

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
