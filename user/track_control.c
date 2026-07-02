#include "headfile.h"
#include "track_control.h"

#define TRACK_BASE_SPEED     240
#define TRACK_SEARCH_SPEED   130
#define TRACK_MAX_TURN       260
#define TRACK_KP             16.0f
#define TRACK_KD             5.0f
#define TRACK_LOST_STOP_CNT  8
#define TRACK_FRAME_TIMEOUT  50

static uint8_t track_no_frame_count = 0;
static uint8_t sensor_raw = 0;
static uint8_t sensor_bits = 0;
static uint8_t sensor_active_count = 0;
static uint8_t track_frame_count = 0;
static uint8_t track_d_frame_count = 0;
static uint8_t track_a_frame_count = 0;
static uint8_t track_lost_count = 0;
static int track_error = 0;
static int last_track_error = 0;
static int track_turn = 0;

static uint8_t ir_rx_buf[100];
static uint8_t ir_package[100];
static uint8_t ir_data_number[8] = {1, 1, 1, 1, 1, 1, 1, 1};
static uint16_t ir_data_analog[8] = {0};

static int limit_int(int value, int min_value, int max_value)
{
	if(value < min_value)
	{
		return min_value;
	}
	if(value > max_value)
	{
		return max_value;
	}
	return value;
}

void track_car_stop(void)
{
	control_speed(0, 0, 0, 0);
}

void track_car_drive(int left_speed, int right_speed)
{
	control_speed(left_speed, left_speed, right_speed, right_speed);
}

void track_control_request_data(void)
{
	uart_sendstr(UART_1, "$0,1,1#");
}

void track_control_init(void)
{
	uart_init(UART_1, 115200, 0);
	track_control_request_data();
}

static void deal_track_package(void)
{
	uint8_t i;
	uint8_t index = 0;
	uint16_t value = 0;

	if(ir_package[1] == 'D')
	{
		for(i = 0; i < 8; i++)
		{
			if(ir_package[6 + i * 5] == '0' || ir_package[6 + i * 5] == '1')
			{
				ir_data_number[i] = ir_package[6 + i * 5] - '0';
			}
		}

		if(track_d_frame_count < 255)
		{
			track_d_frame_count++;
		}
	}
	else if(ir_package[1] == 'A')
	{
		for(i = 0; i < sizeof(ir_package); i++)
		{
			if(ir_package[i] == 'x' && ir_package[i + 2] == ':' && ir_package[i + 1] >= '1' && ir_package[i + 1] <= '8')
			{
				index = ir_package[i + 1] - '1';
				value = 0;
				i += 3;
				while(ir_package[i] >= '0' && ir_package[i] <= '9')
				{
					value = value * 10 + (ir_package[i] - '0');
					i++;
				}
				ir_data_analog[index] = value;
			}
		}

		if(track_a_frame_count < 255)
		{
			track_a_frame_count++;
		}
	}
	else
	{
		return;
	}

	if(track_frame_count < 255)
	{
		track_frame_count++;
	}
	track_no_frame_count = 0;
}

void track_uart_rx(uint8_t data)
{
	static uint8_t start = 0;
	static uint8_t step = 0;
	uint8_t i;

	if(data == '$')
	{
		start = 1;
		step = 0;
		ir_rx_buf[step++] = data;
		return;
	}

	if(start == 0)
	{
		return;
	}

	ir_rx_buf[step++] = data;
	if(data == '#')
	{
		for(i = 0; i < sizeof(ir_package); i++)
		{
			ir_package[i] = ir_rx_buf[i];
			ir_rx_buf[i] = 0;
		}
		start = 0;
		step = 0;
		deal_track_package();
		return;
	}

	if(step >= sizeof(ir_rx_buf))
	{
		for(i = 0; i < sizeof(ir_rx_buf); i++)
		{
			ir_rx_buf[i] = 0;
		}
		start = 0;
		step = 0;
	}
}

static uint8_t read_track_sensors(void)
{
	uint8_t bits = 0;
	uint8_t i;

	sensor_raw = 0;
	sensor_active_count = 0;
	for(i = 0; i < 8; i++)
	{
		if(ir_data_number[i])
		{
			sensor_raw |= (1 << i);
		}
		else
		{
			bits |= (1 << i);
			sensor_active_count++;
		}
	}

	return bits;
}

static int calc_track_error(uint8_t bits)
{
	uint8_t x1;
	uint8_t x2;
	uint8_t x3;
	uint8_t x4;
	uint8_t x5;
	uint8_t x6;
	uint8_t x7;
	uint8_t x8;

	if(sensor_active_count == 0)
	{
		return last_track_error;
	}

	x1 = (bits & 0x01) ? 0 : 1;
	x2 = (bits & 0x02) ? 0 : 1;
	x3 = (bits & 0x04) ? 0 : 1;
	x4 = (bits & 0x08) ? 0 : 1;
	x5 = (bits & 0x10) ? 0 : 1;
	x6 = (bits & 0x20) ? 0 : 1;
	x7 = (bits & 0x40) ? 0 : 1;
	x8 = (bits & 0x80) ? 0 : 1;

	if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 0 && x7 == 0 && x8 == 0) return -15;
	if(x1 == 0 && x2 == 0 && x3 == 0 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) return 15;
	if(x1 == 0 && x2 == 0 && x7 == 0 && x8 == 0) return 0;
	if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) return -1;
	if(x1 == 1 && x2 == 1 && x3 == 0 && x4 == 0 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) return -2;
	if(x1 == 1 && x2 == 0 && x3 == 0 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) return -8;
	if(x1 == 0 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 1) return -10;
	if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1) return 1;
	if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 0 && x6 == 0 && x7 == 1 && x8 == 1) return 2;
	if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 0 && x7 == 0 && x8 == 1) return 8;
	if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 1 && x5 == 1 && x6 == 1 && x7 == 1 && x8 == 0) return 10;
	if(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0 && x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1) return 0;

	return last_track_error;
}

void track_follow_update(void)
{
	int left_speed;
	int right_speed;

	sensor_bits = read_track_sensors();
	track_error = calc_track_error(sensor_bits);
	if(track_no_frame_count < 255)
	{
		track_no_frame_count++;
	}

	if(track_no_frame_count > TRACK_FRAME_TIMEOUT)
	{
		track_car_stop();
		return;
	}

	if(sensor_active_count == 0)
	{
		if(track_lost_count < 255)
		{
			track_lost_count++;
		}

		if(track_lost_count > TRACK_LOST_STOP_CNT)
		{
			track_car_stop();
			return;
		}

		track_car_drive(TRACK_BASE_SPEED, TRACK_BASE_SPEED);
		return;
	}

	track_lost_count = 0;
	if(sensor_active_count >= 6)
	{
		track_turn = 0;
		last_track_error = track_error;
		track_car_drive(TRACK_BASE_SPEED, TRACK_BASE_SPEED);
		return;
	}

	track_turn = (int)(TRACK_KP * track_error + TRACK_KD * (track_error - last_track_error));
	track_turn = limit_int(track_turn, -TRACK_MAX_TURN, TRACK_MAX_TURN);
	left_speed = limit_int(TRACK_BASE_SPEED + track_turn, -TRACK_SEARCH_SPEED, TRACK_BASE_SPEED + TRACK_MAX_TURN);
	right_speed = limit_int(TRACK_BASE_SPEED - track_turn, -TRACK_SEARCH_SPEED, TRACK_BASE_SPEED + TRACK_MAX_TURN);
	track_car_drive(left_speed, right_speed);
	last_track_error = track_error;
}

uint8_t track_has_line(void)
{
	sensor_bits = read_track_sensors();
	return sensor_active_count > 0;
}

uint8_t track_line_lost(void)
{
	return track_lost_count > TRACK_LOST_STOP_CNT;
}

Track_Info_t track_get_info(void)
{
	Track_Info_t info;
	uint8_t i;

	info.raw = sensor_raw;
	info.bits = sensor_bits;
	info.active_count = sensor_active_count;
	info.no_frame_count = track_no_frame_count;
	info.frame_count = track_frame_count;
	info.d_frame_count = track_d_frame_count;
	info.a_frame_count = track_a_frame_count;
	info.lost_count = track_lost_count;
	info.error = track_error;
	info.turn = track_turn;
	for(i = 0; i < 8; i++)
	{
		info.analog[i] = ir_data_analog[i];
	}

	return info;
}
