
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

class spi_master{

public: 
    void set_spi();
    void send_spi(uint8_t);
    uint8_t read_spi();
};

