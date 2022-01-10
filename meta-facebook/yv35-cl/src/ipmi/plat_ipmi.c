#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "plat_i2c.h"
#include "sensor.h"
#include "ipmi.h"
#include "plat_ipmi.h"
#include "hal_gpio.h"
#include "ipmi_def.h"
#include "guid.h"
#include "plat_guid.h"
#include "fru.h"
#include "plat_fru.h"
#include "sensor_def.h"
#include "util_spi.h"
#include "hal_jtag.h"
#include "hal_snoop.h"
#include "hal_peci.h"
#include <drivers/peci.h>
#include "plat_func.h"

bool pal_is_to_ipmi_handler(uint8_t netfn, uint8_t cmd) {
  if (netfn == NETFN_OEM_1S_REQ) {
    if (  (cmd == CMD_OEM_1S_FW_UPDATE) ||
          (cmd == CMD_OEM_1S_RESET_BMC) ||
          (cmd == CMD_OEM_1S_GET_BIC_STATUS) ||
          (cmd == CMD_OEM_1S_RESET_BIC) )
      return 1;
  }

  return 0;
}

bool pal_ME_is_to_ipmi_handler(uint8_t netfn, uint8_t cmd) {
  if ( (netfn == NETFN_OEM_REQ) && (cmd == CMD_OEM_SENSOR_READ) ) {
    return 1;
  }

  return 0;
}

bool pal_is_not_return_cmd(uint8_t netfn, uint8_t cmd) {
  if ( (netfn == NETFN_OEM_1S_REQ) && (cmd == CMD_OEM_1S_MSG_OUT) ) {
    return 1;
  } else if ( (netfn == NETFN_OEM_1S_REQ) && (cmd == CMD_OEM_1S_MSG_IN) ) {
    return 1;
  }

  // Reserve for future commands

  return 0;
}

void pal_CHASSIS_GET_CHASSIS_STATUS(ipmi_msg *msg) {
  if (msg->data_len != 0) {
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }

  CHASSIS_STATUS chassis_status = {0};
  uint8_t result = 0;

  // CurrPwState
  chassis_status.currPwState.pwOn = get_DC_status();
  chassis_status.currPwState.pwFault = get_DC_status();
  // Update pwRestorePolicy
  chassis_status.currPwState.pwRestorePolicy = 0x2;
  result = (result | chassis_status.currPwState.reserve) << 2;
  result = (result | chassis_status.currPwState.pwRestorePolicy) << 1;
  result = (result | chassis_status.currPwState.pwControlFault) << 1;
  result = (result | chassis_status.currPwState.pwFault) << 1;
  result = (result | chassis_status.currPwState.interlock) << 1;
  result = (result | chassis_status.currPwState.pwOverload) << 1;
  result = (result | chassis_status.currPwState.pwOn);
  msg->data[0] = result;

  // LastPwEvt
  result = 0;
  // Update pwOnIpmi
  result = (result | chassis_status.lastPwEvt.reserve) << 1;
  result = (result | chassis_status.lastPwEvt.pwOnIpmi) << 1;
  result = (result | chassis_status.lastPwEvt.pwDownFault) << 1;
  result = (result | chassis_status.lastPwEvt.pwDownInterlock) << 1;
  result = (result | chassis_status.lastPwEvt.pwDownOverload) << 1;
  result = (result | chassis_status.lastPwEvt.acFail);
  msg->data[1] = result;

  // Misc
  result = 0;
  // Update idLedSupport, idLedState, fanFault, fpcLockout, intru
  result = (result | chassis_status.misc.reserve) << 1;
  result = (result | chassis_status.misc.idLedSupport) << 2;
  result = (result | chassis_status.misc.idLedState) << 1;
  result = (result | chassis_status.misc.fanFault) << 1;
  result = (result | chassis_status.misc.driveFault) << 1;
  result = (result | chassis_status.misc.fpcLockout) << 1;
  result = (result | chassis_status.misc.intru);
  msg->data[2] = result;

  // ChassisButtonEnables
  result = 0;
  result = (result | chassis_status.chassisButtonEnables.standbyDisableAllowed) << 1;
  result = (result | chassis_status.chassisButtonEnables.diagnosticDisableAllowed) << 1;
  result = (result | chassis_status.chassisButtonEnables.resetButtonDisableAllowed) << 1;
  result = (result | chassis_status.chassisButtonEnables.powerOffButtonDisableAllowed) << 1;
  result = (result | chassis_status.chassisButtonEnables.standbyDisable) << 1;
  result = (result | chassis_status.chassisButtonEnables.diagnosticDisable) << 1;
  result = (result | chassis_status.chassisButtonEnables.resetButtonDisable) << 1;
  result = (result | chassis_status.chassisButtonEnables.powerOffButtonDisable);
  msg->data[3] = result;

  msg->data_len = 4;
  msg->completion_code = CC_SUCCESS;
  return;

}

void pal_OEM_SENSOR_READ(ipmi_msg *msg) {
  uint8_t status, snr_num;
  int reading;

  if (msg->data_len < 3) { // only input enable status
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }

  // Follow INTEL NM SPEC
  if (msg->data[0] == 0x00) { // read platform pwr from HSC
    snr_num = SENSOR_NUM_PWR_HSCIN;
    status = get_sensor_reading(snr_num, &reading, get_from_cache);
    reading = (reading >> 8) * SDR_M(snr_num); // scale down to one byte and times SDR to get original reading
  } else {
    msg->completion_code = CC_INVALID_DATA_FIELD;
    return;
  }

  switch (status) {
    case SNR_READ_ACUR_SUCCESS:
      msg->data[1] = reading & 0xFF;
      msg->data[2] = ( reading >> 8 ) & 0xFF;
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
    default :
      msg->completion_code = CC_UNSPECIFIED_ERROR; // unknown error
      break;
  }
  return;
}

void pal_OEM_1S_PECIaccess(ipmi_msg *msg) {
  uint8_t addr, cmd, *writeBuf, *readBuf;
  uint8_t u8WriteLen, u8ReadLen, u8Index;
  uint16_t u16Param;
  int ret;

  if (msg->data_len < 3) {
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }

  addr = msg->data[0];
  if ((msg->data[1] == 0) && (msg->data[2] == 0)) {
    ret = peci_ping(addr);
    msg->data[0] = ret;
    msg->data_len = 1;
    msg->completion_code = CC_SUCCESS;
    return;
  }

  u8WriteLen = msg->data[1];
  u8ReadLen = msg->data[2];
  cmd = msg->data[3];
  u8Index = msg->data[5];
  u16Param = msg->data[7];
  u16Param = (u16Param << 8) + msg->data[6];
  readBuf = (uint8_t *)malloc(sizeof(uint8_t) * u8ReadLen);
  writeBuf = (uint8_t *)malloc(sizeof(uint8_t) * u8WriteLen);
  memcpy(&writeBuf[0], &msg->data[4], u8WriteLen);

  if (cmd == PECI_RD_PKG_CFG0_CMD) {
    ret = peci_read(cmd, addr, u8Index, u16Param, u8ReadLen, readBuf);
  } else if (cmd == PECI_WR_PKG_CFG0_CMD || cmd == PECI_CRASHDUMP_CMD) {
    ret = peci_write(cmd, addr, u8Index, u16Param, u8ReadLen, readBuf, u8WriteLen, writeBuf);
  } else {
    printf("command not support\n");
    msg->completion_code = CC_NOT_SUPP_IN_CURR_STATE;
    free(writeBuf);
    free(readBuf);
    return;
  }

  if (ret) {
    free(writeBuf);
    free(readBuf);
    msg->completion_code = CC_CAN_NOT_RESPOND;
    return;
  }

  memcpy(&msg->data[0], &readBuf[0], u8ReadLen);
  if (msg->data[0] != PECI_CC_RSP_SUCCESS) {
    msg->data[0] = (msg->data[0] == 0xf9) ? PECI_CC_ILLEGAL_REQUEST : msg->data[0];
    memset(&msg->data[1], 0xff, u8ReadLen - 1);
  }

  free(writeBuf);
  free(readBuf);
  msg->data_len = u8ReadLen;
  msg->completion_code = CC_SUCCESS;
  return;
}

void pal_OEM_1S_FW_UPDATE(ipmi_msg *msg) {
  /*********************************
 * buf 0:   target, 0x80 indicate last byte
 * buf 1~4: offset, 1 lsb
 * buf 5~6: length, 5 lsb
 * buf 7~N: data
 ***********************************/
  if (msg->data_len < 8) {
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }

  uint8_t target = msg->data[0];
  uint8_t status = -1;
  uint32_t offset = ((msg->data[4] << 24) | (msg->data[3] << 16) | (msg->data[2] << 8) | msg->data[1]);
  uint16_t length = ((msg->data[6] << 8) | msg->data[5]);

  if( (length == 0) || (length != msg->data_len-7) ) {
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }

  if(target == BIOS_UPDATE) {
    if(offset > 0x4000000) { // BIOS size maximum 64M bytes
      msg->completion_code = CC_PARAM_OUT_OF_RANGE;
      return;
    }
    status = fw_update(offset, length, &msg->data[7], (target & UPDATE_EN), devspi_spi1_cs0);

  } else if( (target == BIC_UPDATE) || (target == (BIC_UPDATE | UPDATE_EN)) ) {
    if(offset > 0x50000) { // Expect BIC firmware size not bigger than 320k
      msg->completion_code = CC_PARAM_OUT_OF_RANGE;
      return;
    }
    status = fw_update(offset, length, &msg->data[7], (target & UPDATE_EN), devspi_fmc_cs0);
  }

  msg->data_len = 0;

  switch (status) {
    case fwupdate_success:
      msg->completion_code = CC_SUCCESS;
      break;
    case fwupdate_out_of_heap:
      msg->completion_code = CC_LENGTH_EXCEEDED;
      break;
    case fwupdate_over_length:
      msg->completion_code = CC_OUT_OF_SPACE;
      break;
    case fwupdate_repeated_updated:
      msg->completion_code = CC_INVALID_DATA_FIELD;
      break;
    case fwupdate_update_fail:
      msg->completion_code = CC_TIMEOUT;
      break;
    case fwupdate_error_offset:
      msg->completion_code = CC_PARAM_OUT_OF_RANGE;
      break;
    default:
      msg->completion_code = CC_UNSPECIFIED_ERROR;
      break;
  }
  if (status != fwupdate_success) {
    printf("spi fw cc: %x\n", msg->completion_code);
  }

  return;

}

void pal_OEM_1S_GET_FW_VERSION(ipmi_msg *msg) {
  if (msg->data_len != 1) {
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }

  uint8_t component, retry = 3;
  component = msg->data[0];
  I2C_MSG i2c_msg;

  if(component == CPNT_PVCCIN || component == CPNT_PVCCFA_EHV_FIVRA) { i2c_msg.slave_addr = PVCCIN_addr; }
  if(component == CPNT_PVCCD_HV) { i2c_msg.slave_addr = PVCCD_HV_addr; }
  if(component == CPNT_PVCCINFAON || component == CPNT_PVCCFA_EHV) { i2c_msg.slave_addr = PVCCFA_EHV_addr; }

  switch (component) {
    case CPNT_CPLD:
      msg->completion_code = CC_UNSPECIFIED_ERROR;
      break;
    case CPNT_BIC:
      msg->data[0] = BIC_FW_YEAR_MSB;
      msg->data[1] = BIC_FW_YEAR_LSB;
      msg->data[2] = BIC_FW_WEEK;
      msg->data[3] = BIC_FW_VER;
      msg->data[4] = BIC_FW_platform_0;
      msg->data[5] = BIC_FW_platform_1;
      msg->data[6] = BIC_FW_platform_2;
      msg->data_len = 7;
      msg->completion_code = CC_SUCCESS;
      break;
    case CPNT_ME:
    {
      ipmb_error status;
      ipmi_msg *bridge_msg;
      bridge_msg = (ipmi_msg*)malloc(sizeof(ipmi_msg));

      bridge_msg->data_len = 0;
      bridge_msg->seq_source = 0xff;
      bridge_msg->InF_source = Self_IFs;
      bridge_msg->InF_target = ME_IPMB_IFs;
      bridge_msg->netfn = NETFN_APP_REQ;
      bridge_msg->cmd = CMD_APP_GET_DEVICE_ID;

      status = ipmb_read(bridge_msg, IPMB_inf_index_map[bridge_msg->InF_target]);
      if (status != ipmb_error_success) {
        printf("ipmb read fail status: %x", status);
        free(bridge_msg);
        msg->completion_code = CC_BRIDGE_MSG_ERR;
        return;
      } else {
        msg->data[0] = bridge_msg->data[2] & 0x7F;
        msg->data[1] = bridge_msg->data[3] >> 4;
        msg->data[2] = bridge_msg->data[3] & 0x0F;
        msg->data[3] = bridge_msg->data[12];
        msg->data[4] = bridge_msg->data[13] >> 4;
        msg->data_len = 5;
        msg->completion_code = CC_SUCCESS;
        free(bridge_msg);
      }
      break;
    }
    case CPNT_PVCCIN:
    case CPNT_PVCCFA_EHV_FIVRA:
    case CPNT_PVCCD_HV:
    case CPNT_PVCCINFAON:
    case CPNT_PVCCFA_EHV:
      i2c_msg.bus = i2c_bus5;
      i2c_msg.tx_len = 3;
      i2c_msg.data[0] = 0xC7;
      i2c_msg.data[1] = 0x94;
      i2c_msg.data[2] = 0x00;

      if(!i2c_master_write(&i2c_msg, retry)) {
        i2c_msg.tx_len = 1;
        i2c_msg.data[0] = 0xC5;
        i2c_msg.rx_len = 4;

        if(!i2c_master_read(&i2c_msg, retry)) {
          memcpy(&msg->data[0], &i2c_msg.data[3], 1);
          memcpy(&msg->data[1], &i2c_msg.data[2], 1);
          memcpy(&msg->data[2], &i2c_msg.data[1], 1);
          memcpy(&msg->data[3], &i2c_msg.data[0], 1);
          msg->data_len = 4;
          msg->completion_code = CC_SUCCESS;
        } else {
          msg->completion_code = CC_UNSPECIFIED_ERROR;
        }
      } else {
        msg->completion_code = CC_UNSPECIFIED_ERROR;
      }
      break;
    default:
      msg->completion_code = CC_UNSPECIFIED_ERROR;
      break;
  }
  return;
}

void pal_OEM_1S_SET_JTAG_TAP_STA(ipmi_msg *msg) {
  if (!msg){
    printf("pal_OEM_1S_SET_JTAG_TAP_STA: parameter msg is NULL\n");
    return;
  }
  if (msg->data_len != 2) {
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }
  uint8_t tapbitlen, tapdata;

  tapbitlen = msg->data[0];
  tapdata = msg->data[1];
  jtag_set_tap(tapdata, tapbitlen);

  msg->data_len = 0;
  msg->completion_code = CC_SUCCESS;
  return;
}

void pal_OEM_1S_ACCURACY_SENSNR(ipmi_msg *msg) {
  /*********************************
   * buf 0: target sensor number
   * buf 1: read option
   *          0: read from cache
   *          1: read from sensor
   * buf 2: sensor report status
   ***********************************/
  uint8_t status, snr_num, option, snr_report_status;
  uint8_t enable_snr_scan = 0xC0; // following IPMI sensor status response
  uint8_t disable_snr_scan = 0x80;
  int reading;

  if (msg->data_len != 2) {
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }

  if (enable_sensor_poll) {
    snr_report_status = enable_snr_scan;
  } else {
    snr_report_status = disable_snr_scan;
  }

  option = msg->data[1];
  snr_num = msg->data[0];

  if (option == 0) {
    if (enable_sensor_poll) {
      status = get_sensor_reading(snr_num, &reading, get_from_cache);
    } else {
      status = SNR_POLLING_DISABLE;
    }
  } else if (option == 1) {
      status = get_sensor_reading(snr_num, &reading, get_from_sensor);
  }

  switch (status) {
    case SNR_READ_SUCCESS:
      msg->data[0] = reading & 0xff;
      msg->data[1] = 0x00;
      msg->data[2] = snr_report_status; 
      msg->data_len = 3;
      msg->completion_code = CC_SUCCESS;
      break;
    case SNR_READ_ACUR_SUCCESS:
      msg->data[0] = (reading >> 8) & 0xff;
      msg->data[1] = reading & 0xff;
      msg->data[2] = snr_report_status;
      msg->data_len = 3;
      msg->completion_code = CC_SUCCESS;
      break;
    case SNR_NOT_ACCESSIBLE:
      msg->data[0] = 0x00;
      msg->data[1] = 0x00;
      msg->data[2] = ( snr_report_status | 0x20 ); // notice BMC about sensor temporary in not accessible status
      msg->data_len = 3;
      msg->completion_code = CC_SUCCESS;
      break;
    case SNR_POLLING_DISABLE:
      msg->completion_code = CC_SENSOR_NOT_PRESENT; // getting sensor cache while sensor polling disable
      break;
    case SNR_FAIL_TO_ACCESS:
      msg->completion_code = CC_NODE_BUSY; // transection error
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

void pal_OEM_1S_ASD_INIT(ipmi_msg *msg) {
  if (!msg){
    printf("pal_OEM_1S_ASD_INIT: parameter msg is NULL\n");
    return;
  }

  if (msg->data_len != 1) {
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }

  if (msg->data[0] == 0x01) {
    enable_PRDY_interrupt();
  } else if (msg->data[0] == 0xff) {
    disable_PRDY_interrupt();
  } else {
    disable_PRDY_interrupt();
    msg->completion_code = CC_INVALID_DATA_FIELD;
    return;
  }

  msg->data_len = 0;
  msg->completion_code = CC_SUCCESS;
  return;
}

void pal_OEM_1S_JTAG_DATA_SHIFT(ipmi_msg *msg) {
  if (!msg){
    printf("pal_OEM_1S_JTAG_DATA_SHIFT: parameter msg is NULL\n");
    return;
  }
  uint8_t lastidx;
  uint16_t writebitlen, readbitlen, readbyte, databyte;

  writebitlen = (msg->data[1] << 8) | msg->data[0];
  databyte = (writebitlen + 7) >> 3;
  readbitlen = (msg->data[3 + databyte] << 8) | msg->data[2 + databyte];
  readbyte = (readbitlen + 7) >> 3;
  lastidx = msg->data[4 + databyte];

  if (msg->data_len != (5 + databyte)) {
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }

  uint8_t shiftdata[databyte], receivedata[readbyte];
  memset(shiftdata, 0, databyte);
  memset(receivedata, 0, readbyte);
  memcpy(shiftdata, &msg->data[2], databyte);
  jtag_shift_data(writebitlen, shiftdata, readbitlen, receivedata, lastidx);

  memcpy(&msg->data[0], &receivedata[0], readbyte);
  msg->data_len = readbyte;
  msg->completion_code = CC_SUCCESS;
  return;
}

void pal_OEM_1S_GET_POST_CODE(ipmi_msg *msg) {
  int postcode_num = snoop_read_num;
  if ( msg->data_len != 0 ){
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }
  if ( postcode_num ){
    uint8_t offset = 0;
    if ( snoop_read_num > SNOOP_MAX_LEN ){
      postcode_num = SNOOP_MAX_LEN;
      offset = snoop_read_num % SNOOP_MAX_LEN;
    }
    copy_snoop_read_buffer( offset, postcode_num, msg->data );
  }
  msg->data_len = postcode_num;
  msg->completion_code = CC_SUCCESS;
  return;
}

void pal_OEM_1S_RESET_BMC(ipmi_msg *msg) {
  int postcode_num = snoop_read_num;
  if ( msg->data_len != 0 ){
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }

  submit_bmc_warm_reset();

  msg->data_len = postcode_num;
  msg->completion_code = CC_SUCCESS;
  return;
}

void pal_OEM_1S_GET_BIC_STATUS(ipmi_msg *msg) {
  if (!msg){
    printf("<error> pal_OEM_1S_GET_BIC_STATUS: parameter msg is NULL\n");
    return;
  }

  if ( msg->data_len != 0 ){
    msg->completion_code = CC_INVALID_LENGTH;
    return;
  }

  msg->data[0] = FIRMWARE_REVISION_1;
  msg->data[1] = FIRMWARE_REVISION_2;

  msg->data_len = 2;
  msg->completion_code = CC_SUCCESS;
  return;
}
