/*
 * types.h
 *
 *  Created on: Aug 31, 2023
 *      Author: VOM1CLJ
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

typedef union {
	uint16_t button_states;
	struct {
		// @formatter:off
		uint8_t power_on    : 1;
		uint8_t drv_forward : 1;
		uint8_t drv_reverse : 1;
		uint8_t brake_swap  : 1;
		uint8_t brake_state : 1;
		uint8_t cruise_mode : 1;
		uint8_t cruise_up   : 1;
		uint8_t cruise_down : 1;
		uint8_t blink_left  : 1;
		uint8_t blink_right : 1;
		uint8_t fan         : 1;
		uint8_t brake_light : 1;
		uint8_t horn        : 1;
		uint8_t rear_lights : 1;
		uint8_t camera      : 1;
		uint8_t head_lights : 1;
		// @formatter:on
	};
	struct {
		uint8_t optional_state;
		uint8_t aux_state;
	};
} state;

typedef struct {
  	uint8_t counter;
  	uint8_t counter_limit;
  	uint8_t is_released;
} button;

enum State {
	IDLE,
	PRE_CHARGE,
	DRIVE,
	ERR
};

#endif /* INC_BUTTONS_H_ */
