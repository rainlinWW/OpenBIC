#ifndef IPMI_DEF_H
#define IPMI_DEF_H

#define DEVICE_ID 0x00
#define DEVICE_REVISION 0x80
#define FIRMWARE_REVISION_1 0x00
#define FIRMWARE_REVISION_2 0x00
#define IPMI_VERSION 0x02
#define ADDITIONAL_DEVICE_SUPPORT 0xBF
#define WW_IANA_ID 0x009c9c
#define PRODUCT_ID 0x0000
#define AUXILIARY_FW_REVISION 0x00000000
#define GET_TEST_RESULT 0

// firmware update interface
enum {
  BIOS_UPDATE,
  CPLD_UPDATE,
  BIC_UPDATE,
  UPDATE_EN = 0x80,
};

#endif
