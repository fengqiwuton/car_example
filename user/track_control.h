#ifndef __TRACK_CONTROL_H__
#define __TRACK_CONTROL_H__

#include "stdint.h"

typedef struct
{
	uint8_t raw;
	uint8_t bits;
	uint8_t active_count;
	uint8_t no_frame_count;
	uint8_t frame_count;
	uint8_t d_frame_count;
	uint8_t a_frame_count;
	uint8_t lost_count;
	int error;
	int turn;
	uint16_t analog[8];
} Track_Info_t;

void track_control_init(void);
void track_control_request_data(void);
void track_uart_rx(uint8_t data);
void track_follow_update(void);
void track_car_drive(int left_speed, int right_speed);
void track_car_stop(void);
uint8_t track_has_line(void);
uint8_t track_line_lost(void);
void track_reset_lost_count(void);
Track_Info_t track_get_info(void);

#endif
