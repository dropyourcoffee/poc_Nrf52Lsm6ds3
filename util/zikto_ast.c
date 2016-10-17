#include "zikto_ast.h"

//struct ast_calendar Time = {0};
extern ast_calendar_t Time;
static uint32_t tick_cnt_ = 0;
    
void system_tick(){
    static uint32_t tick_cnt = 0;
    
    if(++tick_cnt % TICK_MILISEC == 0){
        tick_cnt = 0;
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
}

uint32_t get_tick_cnt_(){
    return tick_cnt_;
}

uint32_t cal_get_whole(){
    return Time.field;
}

void print_date_time(){
    char str[50];
    
    sprintf(str,"%2d:%2d:%2d",Time.FIELD.hour,Time.FIELD.min,Time.FIELD.sec);
    if(Time.FIELD.hour/10 == 0) str[0] = '0';
    if(Time.FIELD.min/10   == 0) str[3] = '0';
    if(Time.FIELD.sec/10   == 0) str[6] = '0';
    printf("%s\n",str);
    
    
}