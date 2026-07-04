#include "headfile.h"
#include "track_control.h"

#define LOOP_DT_MS              10
#define OLED_UPDATE_MS          200
#define STAGE_PAUSE_MS          300
#define INIT_STEP_DELAY_MS      500
#define RUN_ENABLE              1
#define START_HOLD_MS           1000

#define FIND_LINE_SPEED         80
#define GAP_STRAIGHT_SPEED      150
#define GAP_MIN_TIME_MS         450
#define GAP_MAX_TIME_MS         5000
#define LINE_ENTER_COUNT        1
#define LINE_EXIT_COUNT         20

#define TURN_OPEN_SPEED         150
#define TURN_OPEN_TIME_MS       1200
#define TURN_OPEN_DIR           1

typedef enum
{
	RUN_SEEK_LINE = 0,
	RUN_FOLLOW_LINE,
	RUN_EXIT_STOP,
	RUN_TURN_180,
	RUN_TURN_STOP,
	RUN_GAP_DRIVE
} Run_State_t;

static Run_State_t run_state = RUN_SEEK_LINE;
static uint8_t half_round_index = 0;
static uint16_t gap_time_ms = 0;
static uint16_t state_time_ms = 0;
static uint16_t turn_time_ms = 0;
static uint8_t line_enter_count = 0;
static uint8_t line_exit_count = 0;
static float start_yaw = 0.0f;

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
			if(TURN_OPEN_DIR > 0)
			{
				track_car_drive(-TURN_OPEN_SPEED, TURN_OPEN_SPEED);
			}
			else
			{
				track_car_drive(TURN_OPEN_SPEED, -TURN_OPEN_SPEED);
			}
			if(turn_time_ms >= TURN_OPEN_TIME_MS)
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

	OLED_ShowString(1, 1, "Track Only");
	OLED_ShowString(1, 12, "N");
	OLED_ShowNum(1, 13, info.no_frame_count, 3);

	OLED_ShowString(2, 1, "In");
	OLED_ShowNum(2, 3, line_enter_count, 3);
	OLED_ShowString(2, 7, "Out");
	OLED_ShowNum(2, 10, line_exit_count, 3);
	OLED_ShowString(2, 14, "T");
	OLED_ShowNum(2, 15, turn_time_ms / 100, 2);

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

	motor_stop_on_boot();

	OLED_Init();
	OLED_Clear();
	OLED_ShowString(1, 1, "Track Init");
	oled_show_reset_reason();

	OLED_ShowString(3, 1, "UART1 Init");
	track_control_init();
	OLED_ShowString(3, 1, "UART1 OK  ");
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
	OLED_ShowString(1, 1, "Start Hold");
	track_car_stop();
	delay_ms(START_HOLD_MS);
	OLED_Clear();
	oled_show_run();

	while(1)
	{
		track_follow_update();

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
