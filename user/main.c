//sjdajd
#include "headfile.h"
#include "track_control.h"

#define LOOP_DT_MS              20
#define OLED_UPDATE_MS          200

#define FIND_LINE_SPEED         160
#define GAP_STRAIGHT_SPEED      190
#define GAP_MIN_TIME_MS         350
#define GAP_MAX_TIME_MS         5000

#define TURN_BASE_SPEED         170
#define TURN_MAX_SPEED          330
#define TURN_STOP_ERR_DEG       4.0f
#define TURN_KP                 2.2f

#define STRAIGHT_YAW_KP         4.0f
#define STRAIGHT_MAX_CORR       95

/* Change these to -1 if the car turns away from the target yaw. */
#define TURN_YAW_DIR            1
#define STRAIGHT_YAW_DIR        1

typedef enum
{
	RUN_FIND_LINE = 0,
	RUN_FOLLOW_LINE,
	RUN_TURN_HEADING,
	RUN_STRAIGHT_GAP
} Run_State_t;

static Run_State_t run_state = RUN_FIND_LINE;
static uint8_t half_round_index = 0;
static uint16_t gap_time_ms = 0;
static float start_yaw = 0.0f;
static float target_yaw = 0.0f;

static int limit_main_int(int value, int min_value, int max_value)
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

static float abs_float(float value)
{
	return value < 0.0f ? -value : value;
}

static float normalize_angle(float angle)
{
	while(angle > 180.0f)
	{
		angle -= 360.0f;
	}
	while(angle < -180.0f)
	{
		angle += 360.0f;
	}
	return angle;
}

static int signed_to_int(float value)
{
	if(value >= 0.0f)
	{
		return (int)(value + 0.5f);
	}
	return (int)(value - 0.5f);
}

static void show_signed_num(uint8_t row, uint8_t col, int value, uint8_t len)
{
	if(value < 0)
	{
		OLED_ShowString(row, col, "-");
		OLED_ShowNum(row, col + 1, -value, len);
	}
	else
	{
		OLED_ShowString(row, col, " ");
		OLED_ShowNum(row, col + 1, value, len);
	}
}

static void imu_init_for_yaw(void)
{
	I2C_Init();
	MPU6050_Init();
	HMC5883L_Init();
	exti_init(EXTI_PA7, RISING, 0);
	delay_ms(300);
	yaw_gyro = 0.0f;
	yaw_Kalman = 0.0f;
	start_yaw = yaw_gyro;
	target_yaw = start_yaw;
}

static void drive_straight_hold(float heading, int speed)
{
	float err = normalize_angle(heading - yaw_gyro);
	int corr = (int)(err * STRAIGHT_YAW_KP * STRAIGHT_YAW_DIR);

	corr = limit_main_int(corr, -STRAIGHT_MAX_CORR, STRAIGHT_MAX_CORR);
	track_car_drive(speed - corr, speed + corr);
}

static uint8_t turn_to_heading(float heading)
{
	float err = normalize_angle(heading - yaw_gyro);
	int speed;

	if(abs_float(err) <= TURN_STOP_ERR_DEG)
	{
		track_car_stop();
		return 1;
	}

	speed = (int)(TURN_BASE_SPEED + abs_float(err) * TURN_KP);
	speed = limit_main_int(speed, TURN_BASE_SPEED, TURN_MAX_SPEED);
	if(err * TURN_YAW_DIR > 0.0f)
	{
		track_car_drive(-speed, speed);
	}
	else
	{
		track_car_drive(speed, -speed);
	}

	return 0;
}

static void set_next_exit_heading(void)
{
	if(half_round_index == 0)
	{
		target_yaw = start_yaw + 180.0f;
		half_round_index = 1;
	}
	else
	{
		target_yaw = start_yaw + 360.0f;
		half_round_index = 0;
	}
}

static void update_stadium_run(void)
{
	switch(run_state)
	{
		case RUN_FIND_LINE:
			if(track_has_line())
			{
				run_state = RUN_FOLLOW_LINE;
			}
			else
			{
				drive_straight_hold(target_yaw, FIND_LINE_SPEED);
			}
			break;

		case RUN_FOLLOW_LINE:
			track_follow_update();
			if(track_line_lost())
			{
				track_car_stop();
				set_next_exit_heading();
				gap_time_ms = 0;
				run_state = RUN_TURN_HEADING;
			}
			break;

		case RUN_TURN_HEADING:
			if(turn_to_heading(target_yaw))
			{
				gap_time_ms = 0;
				run_state = RUN_STRAIGHT_GAP;
			}
			break;

		case RUN_STRAIGHT_GAP:
			gap_time_ms += LOOP_DT_MS;
			drive_straight_hold(target_yaw, GAP_STRAIGHT_SPEED);
			if(gap_time_ms > GAP_MIN_TIME_MS && track_has_line())
			{
				run_state = RUN_FOLLOW_LINE;
			}
			else if(gap_time_ms > GAP_MAX_TIME_MS)
			{
				run_state = RUN_FIND_LINE;
			}
			break;

		default:
			run_state = RUN_FIND_LINE;
			break;
	}
}

static void oled_show_run(void)
{
	Track_Info_t info = track_get_info();

	OLED_ShowString(1, 1, "S");
	OLED_ShowNum(1, 2, run_state, 1);
	OLED_ShowString(1, 4, "H");
	OLED_ShowNum(1, 5, half_round_index, 1);
	OLED_ShowString(1, 7, "F");
	OLED_ShowNum(1, 8, info.frame_count, 3);
	OLED_ShowString(1, 12, "N");
	OLED_ShowNum(1, 13, info.no_frame_count, 3);

	OLED_ShowString(2, 1, "Y");
	show_signed_num(2, 2, signed_to_int(yaw_gyro), 4);
	OLED_ShowString(2, 8, "T");
	show_signed_num(2, 9, signed_to_int(target_yaw), 4);

	OLED_ShowString(3, 1, "Raw");
	OLED_ShowHexNum(3, 5, info.raw, 2);
	OLED_ShowString(3, 9, "Sen");
	OLED_ShowHexNum(3, 13, info.bits, 2);

	OLED_ShowString(4, 1, "E");
	show_signed_num(4, 2, info.error, 2);
	OLED_ShowString(4, 6, "T");
	show_signed_num(4, 7, info.turn, 3);
	OLED_ShowString(4, 12, "L");
	OLED_ShowNum(4, 13, info.lost_count, 3);
}

int main(void)
{
	uint16_t oled_tick = 0;
	uint16_t cmd_tick = 0;

	OLED_Init();
	OLED_Clear();
	OLED_ShowString(1, 1, "Stadium Init");

	track_control_init();
	motor_init();
	track_car_stop();
	OLED_ShowString(2, 1, "Track+Motor OK");

	imu_init_for_yaw();
	OLED_ShowString(3, 1, "IMU OK");
	delay_ms(500);

	OLED_Clear();
	oled_show_run();

	while(1)
	{
		update_stadium_run();

		cmd_tick += LOOP_DT_MS;
		if(cmd_tick >= OLED_UPDATE_MS)
		{
			cmd_tick = 0;
			track_control_request_data();
		}

		oled_tick += LOOP_DT_MS;
		if(oled_tick >= OLED_UPDATE_MS)
		{
			oled_tick = 0;
			oled_show_run();
		}

		delay_ms(LOOP_DT_MS);
	}
}

/* Keep the new tracking module compiled even if the EIDE project file has not
 * been updated to include track_control.c as a separate source file. */
#include "track_control.c"
