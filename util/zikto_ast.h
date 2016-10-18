#include "stdint.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "bsp.h"

#pragma anon_unions

#define IS_LEAP_YEAR(a)	((a % 400) == 0 || ((a % 4) == 0 && (a % 100)))

#define APP_TIMER_PRESCALER                              5               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                             6               /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE                       4               /**< Size of timer operation queues. */


struct ast_calv {
	/** Seconds in the range 0 to 59. */
	uint32_t sec   : 6;
	/** Minutes in the range range 0 to 59. */
	uint32_t min   : 6;
	/** Hours in the range 0 to 23. */
	uint32_t hour  : 5;
	/** Day in the range 1 to 31. */
	uint32_t day   : 5;
	/** Month in the range 1 to 12. */
	uint32_t month : 4;
	/** Year in the range 0 to 63. */
	uint32_t year  : 6;
};

/* Input when initializing AST in calendar mode. */
typedef struct  {
	union {
		uint32_t field;
		/** Calendar. */
		struct ast_calv FIELD;
	};
} ast_calendar_t;

uint32_t cal_get_whole();
void print_date_time();
uint32_t get_tick_cnt_();



