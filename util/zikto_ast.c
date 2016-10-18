#include "zikto_ast.h"

volatile ast_calendar_t Time = {
    .FIELD.year   = 16,
    .FIELD.month = 1,
    .FIELD.day     = 1,
    .FIELD.hour   = 0,
    .FIELD.min     = 0,
    .FIELD.sec     = 0
};
static app_timer_id_t m_led_a_timer_id;
static uint32_t tck_count;
      
static void timer_a_handler(void * p_context)
{
    if( ++Time.FIELD.sec % 60 == 0 ){
            Time.FIELD.sec = 0;
            Time.FIELD.min++;
            
            if(Time.FIELD.min % 60 == 0 ){
                Time.FIELD.min = 0;
                Time.FIELD.hour++;
                
                if(Time.FIELD.hour % 24 == 0 ){
                    Time.FIELD.hour = 0;
                    Time.FIELD.day++;
                }
            }
        }
}

/**/

// Create timers
static void create_timers()
{   
    uint32_t err_code;
    // Create timers
    err_code = app_timer_create(&m_led_a_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_a_handler);
    APP_ERROR_CHECK(err_code);
}

void ast_init()
{
    uint32_t err_code;
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
    create_timers();
    err_code = app_timer_start(m_led_a_timer_id, APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
    
}

/**/


uint32_t cal_get_whole(){
    return Time.field;
}

void print_date_time(){
    
    printf("%d-%02d-%02d %02d:%02d:%02d\n", Time.FIELD.year+2000,Time.FIELD.month, Time.FIELD.day, Time.FIELD.hour,Time.FIELD.min,Time.FIELD.sec);
    
    
}