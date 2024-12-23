#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>

#include "NRF24_master/RF24.h"
//#include "SPI_master/SPI.hpp"

//spi_master spi;
RF24 radio;
uint8_t address[5] ={0xB4,0xB5,0xB6,0xB7,0xF1};
char myRxData[50];

void setup_clock(){
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOE);

    // rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);
    // rcc_set_prediv(RCC_CFGR2_PREDIV_DIV12);
}

void setup_gpio (){
   gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9|GPIO10|GPIO11|GPIO12|GPIO13|GPIO14); 
}

void setup_nrf(){

for(volatile uint32_t i=0; i<1000000; i++);

    radio.begin();
    //radio.setAutoAck(1);
    //radio.setRetries(1, 15);
    //radio.enableAckPayload();
    //radio.setPayloadSize(32);
    //radio.openReadingPipe(1, address);
    //radio.setChannel(0x10);
    //radio.setPALevel (RF24_PA_LOW); 
    //radio.setDataRate (RF24_2MBPS);
    //radio.powerUp();
    //radio.startListening();
    gpio_toggle(GPIOE, GPIO9); 
}

void setup(){
    setup_clock();
    setup_gpio();
    setup_nrf();
    
    //spi.set_spi();
}

void loop () {
  //   uint8_t Data[1];
  //   uint8_t ff;
  // while (radio.available(&ff)) {        // слушаем эфир со всех труб

  //   radio.read(Data, 8);  // чиатем входящий сигнал
  //   gpio_toggle(GPIOE, GPIO14);
for(volatile uint32_t i=0; i<1000; i++);
radio.test ();
//radio.setDataRate (RF24_2MBPS);
  //   // if (Data[0] == 55){
  //   //     gpio_toggle(GPIOE, GPIO14);
  //   // }
  // }
//radio.begin();
// radio.setPALevel (RF24_PA_MAX); 
}

int main () {
    setup();
    while (true) {
        loop();
    }   

}
