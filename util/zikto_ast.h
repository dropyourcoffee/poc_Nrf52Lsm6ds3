#include "stdint.h"

#pragma anon_unions

#define TICK_MILISEC  0x13d8//0x71c8b>>7)
#define IS_LEAP_YEAR(a)	((a % 400) == 0 || ((a % 4) == 0 && (a % 100)))

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

void system_tick();
uint32_t cal_get_whole();
void print_date_time();
uint32_t get_tick_cnt_();



