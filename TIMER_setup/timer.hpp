#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

class timer_conf{

public: 
    void set_timer();
    void timer_delay(uint32_t);
    uint32_t timer_get();
};

