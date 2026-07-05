#include "headfile.h"
#include "app_board.h"
#include "track_control.h"

#define LOOP_DT_MS              10
#define OLED_UPDATE_MS          200
#define STAGE_PAUSE_MS          150
#define INIT_STEP_DELAY_MS      500
#define RUN_ENABLE              1
#define START_HOLD_MS           1000

#define FIND_LINE_SPEED         120
#define GAP_STRAIGHT_SPEED      150
#define GAP_MIN_TIME_MS         450
#define GAP_MAX_TIME_MS         5000
#define LINE_ENTER_COUNT        1
#define LINE_EXIT_COUNT         20

#define TURN_BASE_SPEED         90
#define TURN_MAX_SPEED          120
#define TURN_STOP_ERR_DEG       5.0f
#define TURN_KP                 1.4f
#define TURN_MAX_TIME_MS        350
#define STRAIGHT_YAW_KP         0.0f
#define STRAIGHT_MAX_CORR       90
#define TURN_YAW_DIR            1
#define STRAIGHT_YAW_DIR        1

#define TRACK_TUNE_ENABLE       1
#define TRACK_TUNE_DEBOUNCE_MS  20
#define TRACK_TUNE_REPEAT_MS    180
#define TRACK_TUNE_KP_STEP      2
#define TRACK_TUNE_KD_STEP      1
#define TRACK_TUNE_START_HOLD_MS 1200

typedef enum
{
	RUN_SEEK_LINE = 0,
	RUN_FOLLOW_LINE,
	RUN_EXIT_STOP,
	RUN_TURN_180,
	RUN_TURN_STOP,
	RUN_GAP_DRIVE
} Run_State_t;

typedef enum
{
	TUNE_SELECT_KP = 0,
	TUNE_SELECT_KD = 1
} Tune_Select_t;

static Run_State_t run_state = RUN_SEEK_LINE;
static Tune_Select_t tune_select = TUNE_SELECT_KP;
static uint8_t half_round_index = 0;
static uint16_t gap_time_ms = 0;
static uint16_t state_time_ms = 0;
static uint16_t turn_time_ms = 0;
static uint8_t line_enter_count = 0;
static uint8_t line_exit_count = 0;
static float start_yaw = 0.0f;
static float target_yaw = 0.0f;
static int yaw_error_show = 0;
static app_key_t tune_key_up;
static app_key_t tune_key_down;
static uint16_t tune_up_hold_ms = 0;
static uint16_t tune_down_hold_ms = 0;
static uint8_t tune_up_last = 0;
static uint8_t tune_down_last = 0;

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

static void show_fixed_x100(uint8_t row, uint8_t col, int value)
{
	if(value < 0)
	{
		OLED_ShowString(row, col, "-");
		value = -value;
	}
	else
	{
		OLED_ShowString(row, col, " ");
	}

	OLED_ShowNum(row, col + 1, value / 100, 1);
	OLED_ShowString(row, col + 2, ".");
	OLED_ShowNum(row, col + 3, value % 100, 2);
}

static void track_tune_keys_init(void)
{
	app_key_init(&tune_key_up, BOARD_TRACK_KEY_UP_PORT, BOARD_TRACK_KEY_UP_PIN,
	             BOARD_TRACK_KEY_ACTIVE, TRACK_TUNE_DEBOUNCE_MS);
	app_key_init(&tune_key_down, BOARD_TRACK_KEY_DOWN_PORT, BOARD_TRACK_KEY_DOWN_PIN,
	             BOARD_TRACK_KEY_ACTIVE, TRACK_TUNE_DEBOUNCE_MS);
}

static void track_tune_adjust_selected(int direction)
{
	if(tune_select == TUNE_SELECT_KP)
	{
		track_pd_adjust(direction * TRACK_TUNE_KP_STEP, 0);
	}
	else
	{
		track_pd_adjust(0, direction * TRACK_TUNE_KD_STEP);
	}
}

static void track_tune_update_key_level(uint8_t pressed, uint8_t *last_pressed, uint16_t *hold_ms, int direction)
{
	if(pressed)
	{
		if(!*last_pressed)
		{
			*hold_ms = 0;
			track_tune_adjust_selected(direction);
		}
		else
		{
			*hold_ms += LOOP_DT_MS;
			if(*hold_ms >= TRACK_TUNE_REPEAT_MS)
			{
				*hold_ms = 0;
				track_tune_adjust_selected(direction);
			}
		}
	}
	else
	{
		*hold_ms = 0;
	}

	*last_pressed = pressed;
}

static void track_tune_update_adjust_keys(uint8_t up_pressed, uint8_t down_pressed)
{
	if(up_pressed && !down_pressed)
	{
		track_tune_update_key_level(up_pressed, &tune_up_last, &tune_up_hold_ms, 1);
		tune_down_last = 0;
		tune_down_hold_ms = 0;
	}
	else if(down_pressed && !up_pressed)
	{
		track_tune_update_key_level(down_pressed, &tune_down_last, &tune_down_hold_ms, -1);
		tune_up_last = 0;
		tune_up_hold_ms = 0;
	}
	else
	{
		tune_up_last = up_pressed;
		tune_down_last = down_pressed;
		tune_up_hold_ms = 0;
		tune_down_hold_ms = 0;
	}
}

static void oled_show_reset_reason(void)
{
	uint32_t csr = RCC->CSR;

	OLED_ShowString(2, 1, "RST:");
	if(csr & (1UL << 29))
	{
		OLED_ShowString(2, 5, "IWDG");
	}
	else if(csr & (1UL << 30))
	{
		OLED_ShowString(2, 5, "WWDG");
	}
	else if(csr & (1UL << 28))
	{
		OLED_ShowString(2, 5, "SFT");
	}
	else if(csr & (1UL << 27))
	{
		OLED_ShowString(2, 5, "POR");
	}
	else if(csr & (1UL << 26))
	{
		OLED_ShowString(2, 5, "PIN");
	}
	else if(csr & (1UL << 31))
	{
		OLED_ShowString(2, 5, "LPWR");
	}
	else
	{
		OLED_ShowString(2, 5, "NONE");
	}
	OLED_ShowString(2, 10, "C");
	OLED_ShowHexNum(2, 11, (uint16_t)(csr >> 16), 4);
	RCC->CSR |= (1UL << 24);
}

static void motor_stop_on_boot(void)
{
	uint8_t i;

	uart_init(UART_2, 115200, 1);
	USART2->CR1 &= ~(1 << 5);
	for(i = 0; i < 5; i++)
	{
		control_speed(0, 0, 0, 0);
		delay_ms(20);
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

	yaw_error_show = signed_to_int(err);
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
		half_round_index = 1;
	}
	else
	{
		half_round_index = 0;
	}
}

static void update_stadium_run(void)
{
	uint8_t has_line = track_has_line();
	Track_Info_t info;

	switch(run_state)
	{
		case RUN_SEEK_LINE:
			if(has_line)
			{
				line_enter_count++;
				if(line_enter_count >= LINE_ENTER_COUNT)
				{
					line_enter_count = 0;
					line_exit_count = 0;
					run_state = RUN_FOLLOW_LINE;
				}
			}
			else
			{
				line_enter_count = 0;
				track_car_drive(FIND_LINE_SPEED, FIND_LINE_SPEED);
			}
			break;

		case RUN_FOLLOW_LINE:
			track_follow_update();
			info = track_get_info();
			if(info.active_count == 0)
			{
				if(line_exit_count == 0)
				{
					target_yaw = yaw_gyro;
					yaw_error_show = 0;
				}
				if(line_exit_count < 255)
				{
					line_exit_count++;
				}
			}
			else
			{
				line_exit_count = 0;
			}

			if(line_exit_count >= LINE_EXIT_COUNT)
			{
				track_car_stop();
				set_next_exit_heading();
				state_time_ms = 0;
				turn_time_ms = 0;
				line_exit_count = 0;
				run_state = RUN_EXIT_STOP;
			}
			break;

		case RUN_EXIT_STOP:
			track_car_stop();
			state_time_ms += LOOP_DT_MS;
			if(state_time_ms >= STAGE_PAUSE_MS)
			{
				state_time_ms = 0;
				run_state = RUN_TURN_180;
			}
			break;

		case RUN_TURN_180:
			turn_time_ms += LOOP_DT_MS;
			if(turn_to_heading(target_yaw))
			{
				state_time_ms = 0;
				gap_time_ms = 0;
				run_state = RUN_TURN_STOP;
			}
			else if(turn_time_ms >= TURN_MAX_TIME_MS)
			{
				track_car_stop();
				state_time_ms = 0;
				gap_time_ms = 0;
				run_state = RUN_TURN_STOP;
			}
			break;

		case RUN_TURN_STOP:
			track_car_stop();
			state_time_ms += LOOP_DT_MS;
			if(state_time_ms >= STAGE_PAUSE_MS)
			{
				state_time_ms = 0;
				gap_time_ms = 0;
				line_enter_count = 0;
				run_state = RUN_GAP_DRIVE;
			}
			break;

		case RUN_GAP_DRIVE:
			gap_time_ms += LOOP_DT_MS;
			track_car_drive(GAP_STRAIGHT_SPEED, GAP_STRAIGHT_SPEED);
			if(gap_time_ms > GAP_MIN_TIME_MS && has_line)
			{
				line_enter_count++;
				if(line_enter_count >= LINE_ENTER_COUNT)
				{
					line_enter_count = 0;
					line_exit_count = 0;
					run_state = RUN_FOLLOW_LINE;
				}
			}
			else if(gap_time_ms > GAP_MAX_TIME_MS)
			{
				line_enter_count = 0;
				run_state = RUN_SEEK_LINE;
			}
			else if(!has_line)
			{
				line_enter_count = 0;
			}
			break;

		default:
			track_car_stop();
			line_enter_count = 0;
			line_exit_count = 0;
			run_state = RUN_SEEK_LINE;
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
	OLED_ShowString(2, 8, "Er");
	show_signed_num(2, 10, yaw_error_show, 3);

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

static void oled_show_track_tune(uint8_t wait_start)
{
	Track_Info_t info = track_get_info();
	Track_PD_t pd = track_pd_get();

	OLED_ShowString(1, 1, wait_start ? "PD SET  " : "PD RUN  ");
	OLED_ShowString(1, 9, tune_select == TUNE_SELECT_KP ? "Kp" : "Kd");
	OLED_ShowString(1, 12, "Sen");
	OLED_ShowHexNum(1, 15, info.bits, 2);

	OLED_ShowString(2, 1, "Kp");
	show_fixed_x100(2, 3, pd.kp_x100);
	OLED_ShowString(2, 9, "Kd");
	show_fixed_x100(2, 11, pd.kd_x100);

	if(wait_start)
	{
		OLED_ShowString(3, 1, "U");
		OLED_ShowNum(3, 2, app_key_is_pressed(&tune_key_up), 1);
		OLED_ShowString(3, 4, "D");
		OLED_ShowNum(3, 5, app_key_is_pressed(&tune_key_down), 1);
		OLED_ShowString(3, 7, "UD:S/G");
	}
	else
	{
		OLED_ShowString(3, 1, "E");
		show_signed_num(3, 2, info.error, 3);
		OLED_ShowString(3, 8, "T");
		show_signed_num(3, 9, info.turn, 3);
		OLED_ShowString(3, 14, "L");
		OLED_ShowNum(3, 15, info.lost_count, 2);
	}
}

static void track_tune_wait_start(void)
{
	uint16_t oled_tick = 0;
	uint16_t combo_hold_ms = 0;
	uint8_t combo_was_pressed = 0;
	uint8_t up_pressed;
	uint8_t down_pressed;
	uint8_t start_combo;

	tune_up_last = 0;
	tune_down_last = 0;
	tune_up_hold_ms = 0;
	tune_down_hold_ms = 0;
	OLED_Clear();
	oled_show_track_tune(1);

	while(1)
	{
		track_car_stop();
		app_key_update(&tune_key_up, LOOP_DT_MS);
		app_key_update(&tune_key_down, LOOP_DT_MS);

		up_pressed = app_key_is_pressed(&tune_key_up);
		down_pressed = app_key_is_pressed(&tune_key_down);

		start_combo = up_pressed && down_pressed;
		if(start_combo)
		{
			combo_was_pressed = 1;
			combo_hold_ms += LOOP_DT_MS;
			if(combo_hold_ms >= TRACK_TUNE_START_HOLD_MS)
			{
				track_car_stop();
				OLED_Clear();
				OLED_ShowString(1, 1, "Track Start");
				delay_ms(300);
				OLED_Clear();
				return;
			}
		}
		else
		{
			if(combo_was_pressed && combo_hold_ms < TRACK_TUNE_START_HOLD_MS)
			{
				tune_select = (tune_select == TUNE_SELECT_KP) ? TUNE_SELECT_KD : TUNE_SELECT_KP;
			}
			combo_was_pressed = 0;
			combo_hold_ms = 0;
			track_tune_update_adjust_keys(up_pressed, down_pressed);
		}
		oled_tick += LOOP_DT_MS;
		if(oled_tick >= OLED_UPDATE_MS)
		{
			oled_tick = 0;
			oled_show_track_tune(1);
		}

		delay_ms(LOOP_DT_MS);
	}
}

int main(void)
{
	uint16_t oled_tick = 0;

	motor_stop_on_boot();

	OLED_Init();
	OLED_Clear();
	OLED_ShowString(1, 1, "Track Init");
	oled_show_reset_reason();

	OLED_ShowString(3, 1, "UART1 Init");
	track_control_init();
	OLED_ShowString(3, 1, "UART1 OK  ");
	delay_ms(INIT_STEP_DELAY_MS);

	OLED_ShowString(4, 1, "IMU Init");
	imu_init_for_yaw();
	OLED_ShowString(4, 1, "IMU OK  ");
	delay_ms(INIT_STEP_DELAY_MS);

	OLED_Clear();
	OLED_ShowString(1, 1, "Motor Init");
	motor_init();
	track_car_stop();
	OLED_ShowString(1, 1, "Motor OK  ");
	delay_ms(INIT_STEP_DELAY_MS);

#if RUN_ENABLE == 0
	OLED_Clear();
	OLED_ShowString(1, 1, "Hold Stop");
	OLED_ShowString(2, 1, "RUN_ENABLE=0");
	while(1)
	{
		track_car_stop();
		delay_ms(100);
	}
#endif

	OLED_Clear();
	track_tune_keys_init();
	track_car_stop();
#if TRACK_TUNE_ENABLE
	track_tune_wait_start();
	oled_show_track_tune(0);
#else
	OLED_ShowString(1, 1, "Start Hold");
	delay_ms(START_HOLD_MS);
	OLED_Clear();
	oled_show_run();
#endif

	while(1)
	{
		update_stadium_run();

		oled_tick += LOOP_DT_MS;
		if(oled_tick >= OLED_UPDATE_MS)
		{
			oled_tick = 0;
		#if TRACK_TUNE_ENABLE
			oled_show_track_tune(0);
		#else
			oled_show_run();
		#endif
		}

		delay_ms(LOOP_DT_MS);
	}
}

/* Keep the new tracking module compiled even if the EIDE project file has not
 * been updated to include track_control.c as a separate source file. */
#include "track_control.c"
