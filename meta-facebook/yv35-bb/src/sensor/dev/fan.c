#include <stdio.h>
#include <drivers/sensor.h>
#include <drivers/pwm.h>
#include "fan.h"

// init fan device list
#undef DT_DRV_COMPAT
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_tach)
#define DT_DRV_COMPAT aspeed_tach
#endif

#define FAN_LABELS(node_id) { .device_label = DT_LABEL(node_id) },
#define FAN_NODE_LABELS(n) DT_FOREACH_CHILD(DT_DRV_INST(n), FAN_LABELS)
#define FAN_INIT_MACRO() DT_INST_FOREACH_STATUS_OKAY(FAN_NODE_LABELS)
static struct fan_handle {
	char *device_label;
} fan_list[] = {
	FAN_INIT_MACRO()
};

uint8_t ctrl_fan_mode = 1; //default FAN_AUTO_MODE
static int fan_record[2][4]={{0, 0, 0, 0},{0, 0, 0, 0}}; //[0][x] - slot1 BMC , [1][x] - slot3 BMC

int pal_get_fan_ctrl_state(uint8_t *ctrl_state) {
	if (ctrl_state == NULL) {
		printf("failed to get fan control state due to parameter is NULL.\n");
		return -1;
	}

	*ctrl_state = ctrl_fan_mode;
	return 0;
}

void pal_set_fan_ctrl_state(uint8_t ctrl_state) {
	ctrl_fan_mode = ctrl_state;
	return;
}

int pal_get_fan_rpm(uint8_t fan_id, uint8_t *rpm) {
	const struct device *fan_dev;
	struct sensor_value sensor_value;
	int ret = 0;

	if (rpm == NULL) {
		return -1;
	}

	fan_dev = device_get_binding(fan_list[fan_id].device_label);
	if (fan_dev == NULL) {
		printf("FAN%d device not found\n", fan_id);
		return -ENODEV;
	}

	ret = sensor_sample_fetch(fan_dev);
	if (ret < 0) {
		printf("Failed to read FAN%d, ret: %d\n", fan_id, ret);
	}

	ret = sensor_channel_get(fan_dev, SENSOR_CHAN_RPM, &sensor_value);
	if (ret < 0) {
		printf("Failed to read FAN%d, ret: %d\n", fan_id, ret);
	}

	printf("%d\n",sensor_value.val1);
	*rpm = sensor_value.val1;
	return ret;
}

int pal_get_fan_duty(uint8_t fan_id, uint8_t *duty) {
	int ret = 0;
	const struct device *pwm_dev, *fan_dev;
	uint64_t cycles_per_sec = 0;

	pwm_dev = device_get_binding(pwm_device_name);
	if (pwm_dev == NULL) {
		printf("PWM device not found\n");
		return -1;
	}
	
	fan_dev = device_get_binding(fan_list[fan_id].device_label);
	if (fan_dev == NULL) {
		printf("FAN%d device not found\n", fan_id);
		return -1;
	}

	ret = pwm_get_cycles_per_sec(pwm_dev, fan_id, &cycles_per_sec);
	*duty = cycles_per_sec;
	printf("cycles_per_sec %lld\n", cycles_per_sec);
	return ret;
}

int pal_set_fan_duty_manual(uint8_t fan_id, uint8_t duty) {
	const struct device *pwm_dev;
	int ret = 0;

	pwm_dev = device_get_binding(pwm_device_name);
	if (pwm_dev == NULL) {
		printf("PWM device not found\n");
		return -1;
	}

	ret = pwm_pin_set_cycles(pwm_dev, fan_id, 100, duty, 0);
	return ret;
}

int pal_set_fan_duty_auto(uint8_t fan_id, uint8_t duty, uint8_t slot_index) {
	const struct device *pwm_dev;
	uint8_t final_duty = 0;
	int ret = 0;

	pwm_dev = device_get_binding(pwm_device_name);
	if (pwm_dev == NULL) {
		printf("PWM device not found\n");
		return -1;
	}

	if (slot_index == 0x01) {
		fan_record[0][fan_id] = duty;
	} else if (slot_index == 0x03) {
		fan_record[1][fan_id] = duty;
	} else {
		printf("Invalid slot index: %d\n", slot_index);
		return -1;
	}

	if (fan_record[0][fan_id] >= fan_record[1][fan_id]) {
		final_duty = fan_record[0][fan_id];
	} else if (fan_record[0][fan_id] <= fan_record[1][fan_id]) {
		final_duty = fan_record[1][fan_id];
	}

	ret = pwm_pin_set_cycles(pwm_dev, fan_id, 100, final_duty, 0);
	return ret;
}
