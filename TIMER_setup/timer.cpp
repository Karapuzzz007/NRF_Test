#include "TIMER_setup/timer.hpp"



void timer_conf::set_timer(){

    rcc_periph_clock_enable(RCC_TIM2); 
       uint32_t prescaler = rcc_get_timer_clk_freq(RCC_TIM2) / 10'000;
    timer_set_prescaler(TIM2, prescaler);
    timer_set_counter 	( 	TIM2, 0xffffffff );
    timer_enable_counter(TIM2);
    
}

//data - ms
void timer_conf::timer_delay (uint32_t data){
    
    uint32_t time = (timer_get_counter(TIM2) + data*10);

    while (time > timer_get_counter(TIM2)){
     asm("nop");
    }

}

uint32_t timer_conf::timer_get (){
    
    return (timer_get_counter(TIM2));
}
