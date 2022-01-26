#ifndef FAN_H
#define FAN_H

#include <drivers/sensor.h>
#include <drivers/pwm.h>


#define MAX_FAN_DUTY 100
#define MAX_FAN_NUM 4

#define GET_ALL_FAN_DUTY 0xFF

enum fan_ctrl_cmd {
	FAN_SET_MANUAL_MODE = 0x00,
	FAN_SET_AUTO_MODE = 0x01,
	FAN_GET_MODE = 0x02,
};

enum fan_mode {
	FAN_MANUAL_MODE = 0x00,
	FAN_AUTO_MODE = 0x01,
};

static char *pwm_device_name = "PWM";

int pal_get_fan_ctrl_state(uint8_t *ctrl_state);
void pal_set_fan_ctrl_state(uint8_t ctrl_state);
int pal_get_fan_rpm(uint8_t fan_id, uint8_t *rpm);
int pal_get_fan_duty(uint8_t fan_id, uint8_t *duty);
int pal_set_fan_duty_manual(uint8_t fan_id, uint8_t duty);
int pal_set_fan_duty_auto(uint8_t fan_id, uint8_t duty, uint8_t slot_index);

#endif