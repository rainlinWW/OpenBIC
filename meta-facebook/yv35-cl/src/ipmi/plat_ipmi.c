#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include "plat_i2c.h"
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

void pal_OEM_MSG_OUT(ipmi_msg *msg) {
  uint8_t  target_IF;
  ipmb_error status;
  ipmi_msg *bridge_msg = (ipmi_msg*)k_malloc(sizeof(ipmi_msg));

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

    k_free(bridge_msg);
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
