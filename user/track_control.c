#include "headfile.h"
#include "track_control.h"

#define TRACK_BASE_SPEED     150
#define TRACK_EDGE_SPEED     105
#define TRACK_SEARCH_SPEED   80
#define TRACK_MAX_TURN       230
#define TRACK_MIN_EDGE_TURN  145
#define TRACK_KP_DEFAULT     78
#define TRACK_KD_DEFAULT     28
#define TRACK_KP_MIN         0
#define TRACK_KP_MAX         300
#define TRACK_KD_MIN         0
#define TRACK_KD_MAX         200
#define TRACK_LOST_STOP_CNT  60
#define TRACK_FRAME_TIMEOUT  50
#define DRIVE_RAMP_STEP      40
#define EDGE_CONFIRM_COUNT   2

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
static int drive_left_now = 0;
static int drive_right_now = 0;
static uint8_t left_edge_count = 0;
static uint8_t right_edge_count = 0;
static int track_kp_x100 = TRACK_KP_DEFAULT;
static int track_kd_x100 = TRACK_KD_DEFAULT;

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

/* Stop immediately and reset the speed ramp state. */
void track_car_stop(void)
{
	drive_left_now = 0;
	drive_right_now = 0;
	control_speed(0, 0, 0, 0);
}

/* Send differential speed to the motor board with a small ramp to reduce jerk. */
void track_car_drive(int left_speed, int right_speed)
{
	if(left_speed > drive_left_now + DRIVE_RAMP_STEP)
	{
		drive_left_now += DRIVE_RAMP_STEP;
	}
	else if(left_speed < drive_left_now - DRIVE_RAMP_STEP)
	{
		drive_left_now -= DRIVE_RAMP_STEP;
	}
	else
	{
		drive_left_now = left_speed;
	}

	if(right_speed > drive_right_now + DRIVE_RAMP_STEP)
	{
		drive_right_now += DRIVE_RAMP_STEP;
	}
	else if(right_speed < drive_right_now - DRIVE_RAMP_STEP)
	{
		drive_right_now -= DRIVE_RAMP_STEP;
	}
	else
	{
		drive_right_now = right_speed;
	}

	control_speed(drive_left_now, drive_left_now, drive_right_now, drive_right_now);
}

/* Ask the 8-channel tracking module to upload digital and analog data. */
void track_control_request_data(void)
{
	uart_sendstr(UART_1, "$0,1,1#");
}

void track_control_init(void)
{
	uart_init(UART_1, 115200, 0);
	track_control_request_data();
}

Track_PD_t track_pd_get(void)
{
	Track_PD_t pd;

	pd.kp_x100 = track_kp_x100;
	pd.kd_x100 = track_kd_x100;
	return pd;
}

void track_pd_set(int kp_x100, int kd_x100)
{
	track_kp_x100 = limit_int(kp_x100, TRACK_KP_MIN, TRACK_KP_MAX);
	track_kd_x100 = limit_int(kd_x100, TRACK_KD_MIN, TRACK_KD_MAX);
}

void track_pd_adjust(int kp_delta_x100, int kd_delta_x100)
{
	track_pd_set(track_kp_x100 + kp_delta_x100, track_kd_x100 + kd_delta_x100);
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

/* Convert active sensor positions into a signed line error.
 * Negative means the line is on the left side, positive means right side.
 */
static int calc_track_error(uint8_t bits)
{
	static const int sensor_weight[8] = {-350, -250, -150, -50, 50, 150, 250, 350};
	int sum = 0;
	uint8_t i;

	if(sensor_active_count == 0)
	{
		return last_track_error;
	}

	if(sensor_active_count >= 7)
	{
		return 0;
	}

	for(i = 0; i < 8; i++)
	{
		if(bits & (1 << i))
		{
			sum += sensor_weight[i];
		}
	}

	return sum / sensor_active_count;
}

void track_follow_update(void)
{
	int base_speed;
	int left_speed;
	int right_speed;

	sensor_bits = read_track_sensors();
	track_error = calc_track_error(sensor_bits);
	if(sensor_bits & 0x03)
	{
		if(left_edge_count < 255)
		{
			left_edge_count++;
		}
		right_edge_count = 0;
	}
	else if(sensor_bits & 0xC0)
	{
		if(right_edge_count < 255)
		{
			right_edge_count++;
		}
		left_edge_count = 0;
	}
	else
	{
		left_edge_count = 0;
		right_edge_count = 0;
	}

	if(left_edge_count >= EDGE_CONFIRM_COUNT && track_error > -300)
	{
		track_error = -300;
	}
	else if(right_edge_count >= EDGE_CONFIRM_COUNT && track_error < 300)
	{
		track_error = 300;
	}
	track_error = (track_error * 2 + last_track_error) / 3;
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
		left_edge_count = 0;
		right_edge_count = 0;
		if(track_lost_count < 255)
		{
			track_lost_count++;
		}

		if(track_lost_count > TRACK_LOST_STOP_CNT)
		{
			track_car_stop();
			return;
		}

		if(last_track_error > 0)
		{
			track_car_drive(TRACK_SEARCH_SPEED, -TRACK_SEARCH_SPEED);
		}
		else if(last_track_error < 0)
		{
			track_car_drive(-TRACK_SEARCH_SPEED, TRACK_SEARCH_SPEED);
		}
		else
		{
			track_car_drive(TRACK_SEARCH_SPEED, TRACK_SEARCH_SPEED);
		}
		return;
	}

	track_lost_count = 0;
	if(sensor_active_count >= 7)
	{
		/* Full black bar - turn toward last known direction */
		left_edge_count = 0;
		right_edge_count = 0;
		if(last_track_error > 0)
			track_car_drive(-TRACK_EDGE_SPEED, TRACK_EDGE_SPEED);
		else if(last_track_error < 0)
			track_car_drive(TRACK_EDGE_SPEED, -TRACK_EDGE_SPEED);
		else
			track_car_drive(TRACK_SEARCH_SPEED, TRACK_SEARCH_SPEED);
		return;
	}

		track_turn = (track_kp_x100 * track_error + track_kd_x100 * (track_error - last_track_error)) / 100;
	track_turn = limit_int(track_turn, -TRACK_MAX_TURN, TRACK_MAX_TURN);
	if(track_error > 250 && track_turn < TRACK_MIN_EDGE_TURN)
	{
		track_turn = TRACK_MIN_EDGE_TURN;
	}
	else if(track_error < -250 && track_turn > -TRACK_MIN_EDGE_TURN)
	{
		track_turn = -TRACK_MIN_EDGE_TURN;
	}

	if(track_error > 250 || track_error < -250)
	{
		base_speed = TRACK_EDGE_SPEED;
	}
	else
	{
		base_speed = TRACK_BASE_SPEED;
	}

	left_speed = limit_int(base_speed + track_turn, -TRACK_SEARCH_SPEED, base_speed + TRACK_MAX_TURN);
	right_speed = limit_int(base_speed - track_turn, -TRACK_SEARCH_SPEED, base_speed + TRACK_MAX_TURN);
	track_car_drive(left_speed, right_speed);
	last_track_error = track_error;
}

void track_reset_lost_count(void)
{
	track_lost_count = 0;
	track_no_frame_count = 0;
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

/* Read sensors and return the weighted line-position error.
 * Does NOT drive motors — safe to call from seek / gap-drive states. */
int track_read_line_error(void)
{
	sensor_bits = read_track_sensors();
	if(sensor_active_count == 0)
	{
		return 0;
	}
	return calc_track_error(sensor_bits);
}

uint8_t track_read_active_count(void)
{
	return sensor_active_count;
}
