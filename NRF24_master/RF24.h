/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * @file RF24.h
 *
 * Class declaration for RF24 and helper enums
 */

#ifndef __RF24_H__
#define __RF24_H__

//#include <NRF24_master/RF24_config.h>

/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;

/**
 * CRC Length.  How big (if any) of a CRC is included.
 *
 * For use with setCRCLength()
 */
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;

/**
 * Driver for nRF24L01(+) 2.4GHz Wireless Transceiver
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>

#include "SPI_master/SPI.hpp"
#include "TIMER_setup/timer.hpp"


class RF24
{
private:
  bool wide_band; /* 2Mbs data rate in use? */
  bool p_variant; /* False for RF24L01 and true for RF24L01P */
  uint8_t payload_size; /**< Fixed size of payloads */
  bool ack_payload_available; /**< Whether there is an ack payload waiting */
  bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */ 
  uint8_t pipe0_reading_address[5]; /**< Last address set on pipe 0 for reading. */

protected:

  int _BV(uint8_t);

  void csn(int mode);

  void ce(int level);

  uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len);

  uint8_t read_register(uint8_t reg);

  uint8_t write_register(uint8_t reg, const uint8_t* buf, uint8_t len);

  uint8_t write_register(uint8_t reg, uint8_t value);

  uint8_t write_payload(const uint8_t* buf, uint8_t len);

  uint8_t read_payload(uint8_t* buf, uint8_t len);

  uint8_t flush_rx(void);

  uint8_t flush_tx(void);

  uint8_t get_status(void);

  void toggle_features(void);

public:

  RF24();

  uint8_t test();

  void begin(void);

  void startListening(void);

  void stopListening(void);

  bool write( const uint8_t* buf, uint8_t len );

  bool available(void);

  bool read( uint8_t* buf, uint8_t len );

  void openWritingPipe(uint8_t* address);

  void openReadingPipe(uint8_t number, uint8_t* address);

  void setRetries(uint8_t delay, uint8_t count);

  void setChannel(uint8_t channel);

  void setPayloadSize(uint8_t size);

  uint8_t getDynamicPayloadSize(void);

  void enableAckPayload(void);

  void enableDynamicPayloads(void);

  bool isPVariant(void) ;

  void setAutoAck(bool enable);

  void setAutoAck( uint8_t pipe, bool enable ) ;

  void setPALevel( rf24_pa_dbm_e level ) ;

  rf24_pa_dbm_e getPALevel( void ) ;

  bool setDataRate(rf24_datarate_e speed);

  rf24_datarate_e getDataRate( void ) ;

  void setCRCLength(rf24_crclength_e length);

  rf24_crclength_e getCRCLength(void);

  void disableCRC( void ) ;

  void powerDown(void);

  void powerUp(void) ;

  bool available(uint8_t* pipe_num);

  void startWrite( const uint8_t* buf, uint8_t len );

  void writeAckPayload(uint8_t pipe, const uint8_t* buf, uint8_t len);

  bool isAckPayloadAvailable(void);

  void whatHappened(bool& tx_ok,bool& tx_fail,bool& rx_ready);

  bool testCarrier(void);

  bool testRPD(void) ;

  //bool isValid() { return ce_pin != 0xff && csn_pin != 0xff; } 

};


#endif // __RF24_H__

