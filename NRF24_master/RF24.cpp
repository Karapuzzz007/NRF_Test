
#include "NRF24_master/nRF24L01.h"
#include "NRF24_master/RF24.h"

using namespace std;

/****************************************************************************/
spi_master SPI;
timer_conf Timer;
/****************************************************************************/


uint8_t RF24::test()
{
  uint8_t status;

  csn(LOW);

  SPI.send_spi( R_REGISTER | ( REGISTER_MASK & SETUP_AW ) );
  status = SPI.read_spi();
  SPI.send_spi(0xFF);
  uint8_t result = SPI.read_spi();
  // SPI.send_spi(0xFF);
  // uint8_t result = SPI.read_spi();

  csn(HIGH);
  return result;
}



void RF24::csn(int mode)
{
  if (mode == HIGH) {
    gpio_set(GPIOA, GPIO4);
  } else { 
    gpio_clear(GPIOA, GPIO4);
  }
}

/****************************************************************************/

void RF24::ce(int mode)
{
    if (mode) {
    gpio_set(GPIOC, GPIO5);
  } else { 
     gpio_clear(GPIOC, GPIO5);
  }
}

/****************************************************************************/

uint8_t RF24::read_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
  uint8_t status;
  int i = 0;

  csn(LOW);
  SPI.send_spi( R_REGISTER | ( REGISTER_MASK & reg ) );
  status = SPI.read_spi();
  while ( len-- ){
    SPI.send_spi(0xFF);
    buf[i++] = SPI.read_spi();
  }
  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24::read_register(uint8_t reg)
{
  uint8_t status;

  csn(LOW);

  SPI.send_spi( R_REGISTER | ( REGISTER_MASK & reg ) );
  status = SPI.read_spi();
  SPI.send_spi(0xFF);
  uint8_t result = SPI.read_spi();
  // SPI.send_spi(0xFF);
  // uint8_t result = SPI.read_spi();

  csn(HIGH);
  return result;
}

/****************************************************************************/

uint8_t RF24::write_register(uint8_t reg, const uint8_t* buf, uint8_t len)
{
  uint8_t status;
  int i = 0;

  csn(LOW);

  SPI.send_spi( W_REGISTER | ( REGISTER_MASK & reg ) );
  status = SPI.read_spi();

while ( len-- ){
    SPI.send_spi(buf[len]);
}
  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24::write_register(uint8_t reg, uint8_t value)
{
  uint8_t status;

  csn(LOW);
  SPI.send_spi( W_REGISTER | ( REGISTER_MASK & reg ) );
  status = SPI.read_spi();
  SPI.send_spi(value);
  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24::write_payload(const uint8_t* buf, uint8_t len)
{
  uint8_t status;

  uint8_t data_len;
  uint8_t blank_len;
  int i = 0;

  if (payload_size > len){
    data_len = len;
  }else{
    data_len = payload_size;
  }

if (dynamic_payloads_enabled){
  blank_len = 0;
} else {
  blank_len = payload_size - data_len;
}
  
  csn(LOW);
  SPI.send_spi( W_TX_PAYLOAD );
  status = SPI.read_spi();
  while ( data_len-- ){
    SPI.send_spi(buf[i++]);
  }

  while ( blank_len-- )
    SPI.send_spi(0);
  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24::read_payload(uint8_t* buf, uint8_t len)
{
  uint8_t status;
  int i = 0;

    uint8_t data_len;
    uint8_t blank_len;

  if (payload_size > len){
    data_len = len;
  }else{
    data_len = payload_size;
  }

if (dynamic_payloads_enabled){
  blank_len = 0;
} else {
  blank_len = payload_size - data_len;
}
  
  csn(LOW);

    SPI.send_spi( R_RX_PAYLOAD );
    status = SPI.read_spi();

  while ( data_len-- ){
   SPI.send_spi( 0xFF );
   buf[i++] = SPI.read_spi();
}
  while ( blank_len-- ){
       SPI.send_spi(0);
  }

  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24::flush_rx(void)
{
  uint8_t status;

  csn(LOW);

    SPI.send_spi( FLUSH_RX );
    status = SPI.read_spi();  

  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24::flush_tx(void)
{
  uint8_t status;

  csn(LOW);

    SPI.send_spi( FLUSH_TX );
    status = SPI.read_spi(); 

  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24::get_status(void)
{
  uint8_t status;

  csn(LOW);

    SPI.send_spi( NOP );
    status = SPI.read_spi(); 

  csn(HIGH);

  return status;
}

/****************************************************************************/

RF24::RF24():
  wide_band(true), 
  p_variant(false), 
  payload_size(32), 
  ack_payload_available(false), 
  dynamic_payloads_enabled(false),
  pipe0_reading_address{ 0,0,0,0,0}
{
}

/****************************************************************************/

void RF24::setChannel(uint8_t channel)
{

  const uint8_t max_channel = 127;

  if (max_channel < channel){
    channel = max_channel;
  }

  write_register(RF_CH, channel);
}

/****************************************************************************/

void RF24::setPayloadSize(uint8_t size)
{
  const uint8_t max_payload_size = 32;

  if (max_payload_size < size){
    size = max_payload_size;
  }

  payload_size = size;
}

/****************************************************************************/

void RF24::begin(void)
{
  // Initialize SPI bus
  SPI.set_spi();
  Timer.set_timer();

  ce(LOW);
  csn(HIGH);

  // Время для стабилизации 5мс

  Timer.timer_delay(5);

  // Устанрвка времени между попытками достучаться (1500 мкс)
  // установка попыток достучаться (15)
  write_register(SETUP_RETR,(0b0101'0000) | (0b0000'1111));

  // Установка мощности передатчика по умолчанию 
  setPALevel( RF24_PA_MAX ) ;

  // Determine if this is a p or non-p RF24 module and then
  // reset our data rate back to default value. This works
  // because a non-P variant won't allow the data rate to
  // be set to 250Kbps.
  // if( setDataRate( RF24_250KBPS ) )
  // {
  //   p_variant = true ;
  // }
  
  // Установка скорости передчи по умолчанию 
  setDataRate( RF24_2MBPS ) ;

  // Установка CRC на 16 бит
  setCRCLength( RF24_CRC_16 ) ;
  
  // Отключаем динамическую нагрузку
  write_register(DYNPD,0);

  // Сброс текущего состояния
  write_register(STATUS, RX_DR | TX_DS | MAX_RT );

  setChannel(76);

  // Очистка буферов
  flush_rx();
  flush_tx();
}

/****************************************************************************/

void RF24::startListening(void){

  write_register(CONFIG, read_register(CONFIG) | PWR_UP | PRIM_RX);
  write_register(STATUS, RX_DR | TX_DS | MAX_RT );

  // Restore the pipe0 adddress, if exists
  if (pipe0_reading_address)
    write_register(RX_ADDR_P0, pipe0_reading_address, 5);

  flush_rx();
  flush_tx();

  ce(HIGH);

  // дождитесь включения радио 
       for (volatile uint32_t i = 0; i < 260; i++) {
        asm("nop");
    }
}

/****************************************************************************/

void RF24::stopListening(void)
{
  ce(LOW);
  flush_tx();
  flush_rx();
}

/****************************************************************************/

void RF24::powerDown(void)
{
  write_register(CONFIG,read_register(CONFIG) & ~PWR_UP);
}

/****************************************************************************/

void RF24::powerUp(void)
{
  write_register(CONFIG,read_register(CONFIG) | PWR_UP);
}

/******************************************************************/

bool RF24::write( const uint8_t* buf, uint8_t len )
{
  bool result = false;

  startWrite(buf,len);

  // ------------
  // Monitor the send
  uint8_t observe_tx[1];
  uint8_t status;

  uint32_t sent_at = Timer.timer_get();

  const uint32_t timeout = 500; //ms to wait for timeout

  do
  {
    status = read_register(OBSERVE_TX,observe_tx,1);
    //IF_SERIAL_DEBUG(Serial.print(observe_tx,HEX));
  }
    while( ! ( status & ( TX_DS | MAX_RT ) ) && ( sent_at + timeout*10 > Timer.timer_get() ) );

  // The part above is what you could recreate with your own interrupt handler,
  // and then call this when you got an interrupt
  // ------------

  // Call this when you get an interrupt
  // The status tells us three things
  // * The send was successful (TX_DS)
  // * The send failed, too many retries (MAX_RT)
  // * There is an ack packet waiting (RX_DR)
  bool tx_ok, tx_fail;
  whatHappened(tx_ok,tx_fail,ack_payload_available);
  
  //printf("%u%u%u\r\n",tx_ok,tx_fail,ack_payload_available);

  result = tx_ok;
  // IF_SERIAL_DEBUG(Serial.print(result?"...OK.":"...Failed"));

  // Yay, we are done.

  // Power down
  powerDown();

  // Flush buffers (Is this a relic of past experimentation, and not needed anymore??)
  flush_tx();

  return result;
}
/****************************************************************************/

void RF24::startWrite( const uint8_t* buf, uint8_t len)
{
  // Transmitter power-up
  write_register(CONFIG, ( read_register(CONFIG) | PWR_UP ) & ~PRIM_RX );
  // Wait for 150us
     for (volatile uint32_t i = 0; i < 300; i++) {
        asm("nop");
    }
// Timer.timer_delay(0.1);
  // Send the payload
  write_payload( buf, len );

  // Allons!
  ce(HIGH);
  // Wait for 15us
     for (volatile uint32_t i = 0; i < 72; i++) {
        asm("nop");
    }
  ce(LOW);
}

/****************************************************************************/

uint8_t RF24::getDynamicPayloadSize(void)
{
  uint8_t result = 0;

  csn(LOW);
  SPI.send_spi(R_RX_PL_WID);
  SPI.send_spi(0xff);
  result = SPI.read_spi();
  csn(HIGH);

  return result;
}

/****************************************************************************/

bool RF24::available(void)
{
  return available(NULL);
}

/****************************************************************************/

bool RF24::available(uint8_t* pipe_num)
{
  uint8_t status = get_status();

  // Too noisy, enable if you really want lots o data!!
  //IF_SERIAL_DEBUG(print_status(status));

  bool result = ( status & RX_DR );

  if (result)
  {
    // If the caller wants the pipe number, include that
    if ( pipe_num )
      *pipe_num = ( status >> RX_P_NO ) & 0b111;

    // Clear the status bit

    // ??? Should this REALLY be cleared now?  Or wait until we
    // actually READ the payload?

    write_register(STATUS,RX_DR );

    // Handle ack payload receipt
    if ( status & TX_DS )
    {
      write_register(STATUS,TX_DS);
    }
  }

  return result;
}

/****************************************************************************/

bool RF24::read( uint8_t* buf, uint8_t len )
{
  // Fetch the payload
  read_payload( buf, len );

  // was this the last of the data available?
  return read_register(FIFO_STATUS) & RX_EMPTY;
}

/****************************************************************************/

void RF24::whatHappened(bool& tx_ok,bool& tx_fail,bool& rx_ready)
{
  // Read the status & reset the status in one easy call
  // Or is that such a good idea?
  uint8_t status = write_register(STATUS, RX_DR | TX_DS | MAX_RT );

  // Report to the user what happened
  tx_ok = status & TX_DS;
  tx_fail = status & MAX_RT;
  rx_ready = status & RX_DR;
}

/****************************************************************************/

void RF24::openWritingPipe(uint8_t* value)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  write_register(RX_ADDR_P0, value, 5);
  write_register(TX_ADDR, value, 5);

  const uint8_t max_payload_size = 32;

  if (max_payload_size < payload_size){
    payload_size = max_payload_size;
  }
  
  write_register(RX_PW_P0, payload_size);
}

/****************************************************************************/


void RF24::openReadingPipe(uint8_t child, uint8_t* address)
{
  static const uint8_t child_pipe[]  =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[]  =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const uint8_t child_pipe_enable[]  =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0)
    pipe0_reading_address[0] = address[0];
    pipe0_reading_address[1] = address[1];
    pipe0_reading_address[2] = address[2];
    pipe0_reading_address[3] = address[3];
    pipe0_reading_address[4] = address[4];

  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 )
      write_register(child_pipe[child], address, 5);
    else
      write_register(child_pipe[child], address[4]);

    write_register(child_payload_size[child],payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    write_register(EN_RXADDR,read_register(EN_RXADDR) | child_pipe_enable[child]);
  }
}

/****************************************************************************/

void RF24::toggle_features(void)
{
  csn(LOW);
  SPI.send_spi( ACTIVATE );
  SPI.send_spi( 0x73 );
  csn(HIGH);
}

/****************************************************************************/

void RF24::enableDynamicPayloads(void)
{
  // Enable dynamic payload throughout the system
  write_register(FEATURE,read_register(FEATURE) | EN_DPL );

  // If it didn't work, the features are not enabled
  if ( ! read_register(FEATURE) )
  {
    // So enable them and try again
    toggle_features();
    write_register(FEATURE,read_register(FEATURE) | EN_DPL );
  }

  //IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));

  // Enable dynamic payload on all pipes
  //
  // Not sure the use case of only having dynamic payload on certain
  // pipes, so the library does not support it.
  write_register(DYNPD,read_register(DYNPD) | DPL_P5 | DPL_P4 | DPL_P3 | DPL_P2 | DPL_P1 | DPL_P0);

  dynamic_payloads_enabled = true;
}

/****************************************************************************/

void RF24::enableAckPayload(void)
{
  //
  // enable ack payload and dynamic payload features
  //

  write_register(FEATURE,read_register(FEATURE) | EN_ACK_PAY | EN_DPL );

  // If it didn't work, the features are not enabled
  if ( ! read_register(FEATURE) )
  {
    // So enable them and try again
    toggle_features();
    write_register(FEATURE,read_register(FEATURE) | EN_ACK_PAY | EN_DPL );
  }

  //IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));

  //
  // Enable dynamic payload on pipes 0 & 1
  //

  write_register(DYNPD,read_register(DYNPD) | DPL_P1 | DPL_P0);
}

/****************************************************************************/

void RF24::writeAckPayload(uint8_t pipe, const uint8_t* buf, uint8_t len)
{
  int i;
  csn(LOW);
  SPI.send_spi( W_ACK_PAYLOAD | ( pipe & 0b111 ) );
  const uint8_t max_payload_size = 32;
  uint8_t data_len;
  if (payload_size > len){
   data_len = len;
  }else{
   data_len = payload_size;
  }
  
  while ( data_len-- ){
      SPI.send_spi(buf[i++]);
  }
    
  csn(HIGH);
}

/****************************************************************************/

bool RF24::isAckPayloadAvailable(void)
{
  bool result = ack_payload_available;
  ack_payload_available = false;
  return result;
}

/****************************************************************************/

bool RF24::isPVariant(void)
{
  return p_variant ;
}

/****************************************************************************/

void RF24::setAutoAck(bool enable)
{
  if ( enable )
    write_register(EN_AA, 0b0011'1111);
  else
    write_register(EN_AA, 0b0000'0000);
}

/****************************************************************************/

void RF24::setAutoAck( uint8_t pipe, bool enable )
{

  static const uint8_t child_pipe[]  =
{
  P0, P1, P2, P3, P4, P5
};
  if ( pipe <= 6 )
  {
    uint8_t en_aa = read_register( EN_AA ) ;
    if( enable )
    {
      en_aa |= child_pipe[pipe] ;
    }
    else
    {
      en_aa &= ~child_pipe[pipe] ;
    }
    write_register( EN_AA, en_aa ) ;
  }
}

/****************************************************************************/

bool RF24::testCarrier(void)
{
  return ( read_register(CD) & 1 );
}

/****************************************************************************/

// bool RF24::testRPD(void)
// {
//   return ( read_register(RPD) & 1 ) ;
// }

/****************************************************************************/

void RF24::setPALevel(rf24_pa_dbm_e level)
{
  uint8_t setup = read_register( RF_SETUP ) & ~(RF_PWR_LOW | RF_PWR_HIGH);
  // switch uses RAM (evil!)
  if ( level == RF24_PA_MAX )
  {
    setup |= ( RF_PWR_LOW | RF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_HIGH )
  {
    setup |= RF_PWR_HIGH ;
  }
  else if ( level == RF24_PA_LOW )
  {
    setup |= RF_PWR_LOW;
  }
  else if ( level == RF24_PA_MIN )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= ( RF_PWR_LOW | RF_PWR_HIGH) ;
  }

  write_register( RF_SETUP, setup ) ;

  for (volatile uint32_t i = 0; i < 70; i++) {
  asm("nop");
  }

  if ( read_register(RF_SETUP) == setup )
  {
    gpio_set(GPIOE, GPIO11); 
  } 
}

/****************************************************************************/

rf24_pa_dbm_e RF24::getPALevel(void)
{
  rf24_pa_dbm_e result = RF24_PA_ERROR ;
  uint8_t power = read_register(RF_SETUP) & ( RF_PWR_LOW | RF_PWR_HIGH) ;

  // switch uses RAM (evil!)
  if ( power == ( RF_PWR_LOW | RF_PWR_HIGH) )
  {
    result = RF24_PA_MAX ;
  }
  else if ( power == RF_PWR_HIGH )
  {
    result = RF24_PA_HIGH ;
  }
  else if ( power == RF_PWR_LOW )
  {
    result = RF24_PA_LOW ;
  }
  else
  {
    result = RF24_PA_MIN ;
  }

  return result ;
}

/****************************************************************************/

bool RF24::setDataRate(rf24_datarate_e speed)
{
  bool result = false;
  uint8_t setup = read_register(RF_SETUP) & ~( RF_DR_HIGH );

  // HIGH and LOW '00' is 1Mbs - our default
  wide_band = false ;
 

  if( speed == RF24_1MBPS ){
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    wide_band = false ;
    setup = setup  ;
  } else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
      if ( speed == RF24_2MBPS )
    {
       wide_band = true ;
       setup = setup | RF_DR_HIGH;
     }
     else
     {
       // 1Mbs
        wide_band = false ;
      }
  }
  write_register(RF_SETUP,setup);

  // Verify our result

  for (volatile uint32_t i = 0; i < 70; i++) {
  asm("nop");
  }

  if ( read_register(RF_SETUP) == setup )
  {
    result = true;
    gpio_set(GPIOE, GPIO10); 
  } else
  {
    wide_band = false;
  }

  return result;
}

/****************************************************************************/

// rf24_datarate_e RF24::getDataRate( void )
// {
//   rf24_datarate_e result ;
//   uint8_t dr = read_register(RF_SETUP) & ( RF_DR_LOW | RF_DR_HIGH);
  
//   // switch uses RAM (evil!)
//   // Order matters in our case below
//   if ( dr == RF_DR_LOW )
//   {
//     // '10' = 250KBPS
//     result = RF24_250KBPS ;
//   }
//   else if ( dr == RF_DR_HIGH )
//   {
//     // '01' = 2MBPS
//     result = RF24_2MBPS ;
//   }
//   else
//   {
//     // '00' = 1MBPS
//     result = RF24_1MBPS ;
//   }
//   return result ;
// }

/****************************************************************************/

void RF24::setCRCLength(rf24_crclength_e length)
{
  uint8_t config = read_register(CONFIG) ;
  config &= 0b1111'0011; 
  
  // switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above. 
  } else 
  if ( length == RF24_CRC_8 )
  {
    config |= EN_CRC;
  } else
  {
    config |= EN_CRC;
    config |=  CRCO ;
  }
  write_register( CONFIG, config ) ;
}

/****************************************************************************/

// rf24_crclength_e RF24::getCRCLength(void)
// {
//   rf24_crclength_e result = RF24_CRC_DISABLED;
//   uint8_t config = read_register(CONFIG) & ( CRCO | EN_CRC) ;

//   if ( config & EN_CRC  )
//   {
//     if ( config & CRCO )
//       result = RF24_CRC_16;
//     else
//       result = RF24_CRC_8;
//   }

//   return result;
// }

/****************************************************************************/

// void RF24::disableCRC( void )
// {
//   uint8_t disable = read_register(CONFIG) & ~EN_CRC ;
//   write_register( CONFIG, disable ) ;
// }

/****************************************************************************/
void RF24::setRetries(uint8_t delay, uint8_t count)
{

 uint8_t setup;

 setup = ((delay<<ARD) | (count));

 write_register(SETUP_RETR, setup);

  if ( read_register(SETUP_RETR) == setup )
  {
    gpio_set(GPIOE, GPIO12); 
  } 

}

// vim:ai:cin:sts=2 sw=2 ft=cpp

