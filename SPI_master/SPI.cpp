#include "SPI_master/SPI.hpp"

void spi_master::set_spi(){


    /*___LED___*/
    rcc_periph_clock_enable(RCC_GPIOE);
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10|GPIO11|GPIO12);
    
    /*___CSN___*/
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
    
    /*___CE___*/
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

    rcc_periph_clock_enable(RCC_SPI1);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO5 | GPIO6 | GPIO7);

    spi_disable(SPI1);
    spi_set_master_mode(SPI1);
    spi_set_full_duplex_mode(SPI1);
    spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_64);
    spi_set_clock_polarity_0(SPI1);
    spi_set_clock_phase_0(SPI1);
    spi_set_data_size(SPI1, SPI_CR2_DS_8BIT);
    spi_send_msb_first(SPI1);
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);
    spi_enable_ss_output(SPI1);
    spi_fifo_reception_threshold_8bit(SPI1);

    gpio_set(GPIOA,GPIO4);

    spi_enable(SPI1);
    
    // spi_enable_rx_buffer_not_empty_interrupt (SPI1)
    // nvic_enable_irq(NVIC_SPI1_IRQ);

}

void spi_master::send_spi (uint8_t data){
    spi_send8(SPI1, data);
          for (volatile uint32_t i = 0; i < 70; i++) {
        asm("nop");
    }
}

uint8_t spi_master::read_spi (){
  return  spi_read8(SPI1);
      for (volatile uint32_t i = 0; i < 70; i++) {
    asm("nop");
    }
}
