/*
 * $Id$
 *
 * Copyright (C) 2009  Martin Mueller
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file main_logger.c
 *  \brief Logger application
 *
 *   This collects telemetry received through a serial port and writes that
 * to a (micro) SD card through the efsl library
 */

  /* XBee-message: ABCDxxxxxxxE
     A XBEE_START (0x7E)
     B LENGTH_MSB (D->D)
     C LENGTH_LSB
     D XBEE_PAYLOAD
       0 XBEE_TX16 (0x01) / XBEE_RX16 (0x81)
       1 FRAME_ID (0)     / SRC_ID_MSB
       2 DEST_ID_MSB      / SRC_ID_LSB
       3 DEST_ID_LSB      / XBEE_RSSI
       4 TX16_OPTIONS (0) / RX16_OPTIONS
       5 PPRZ_DATA
         0 SENDER_ID
         1 MSG_ID
         2 MSG_PAYLOAD
         . DATA (messages.xml)
     E XBEE_CHECKSUM (sum[A->D])

    ID is AC_ID for aircraft, 0x100 for ground station
  */

  /* PPRZ-message: ABCxxxxxxxDE
     A PPRZ_STX (0x99)
     B LENGTH (A->E)
     C PPRZ_DATA
       0 SENDER_ID
       1 MSG_ID
       2 MSG_PAYLOAD
       . DATA (messages.xml)
     D PPRZ_CHECKSUM_A (sum[B->C])
     E PPRZ_CHECKSUM_B (sum[ck_a])
  */

  /* LOG-message: ABCDEFGHxxxxxxxI
     A PPRZ_STX (0x99)
     B LENGTH (H->H)
     C SOURCE (0=uart0, 1=uart1, 2=i2c0, ...)
     D TIMESTAMP_LSB (100 microsec raster)
     E TIMESTAMP
     F TIMESTAMP
     G TIMESTAMP_MSB
     H PPRZ_DATA
       0 SENDER_ID
       1 MSG_ID
       2 MSG_PAYLOAD
       . DATA (messages.xml)
     I CHECKSUM (sum[B->H])
  */

#include "std.h"
#include "mcu.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#include "usb_msc_hw.h"

#include "efs.h"
#include "ls.h"

#ifdef USE_MAX11040
#include "max11040.h"
#endif

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE (!FALSE)
#endif

#ifndef LOG_STOP_KEY
/* BUTTON that stops logging (PPM_IN = P0.6, BUTTON = P0.7, DTR = P0.13, INT1 = P0.14) */
#define LOG_STOP_KEY 7
#endif

/*
#ifndef POWER_DETECT_PIN
// Pin 0.10
#define POWER_DETECT_PIN 6
#endif

#ifndef CARD_DETECT_PIN
// Pin 1.20
#define CARD_DETECT_PIN 20
#endif
*/

#ifndef LED_GREEN
#define LED_GREEN	3
#endif

#ifndef LED_YELLOW
#define LED_YELLOW	2
#endif

#ifndef LED_RED
#define LED_RED		1
#endif


/* USB Vbus (= P0.23) */
#define VBUS_PIN 23

/** Constants for the API protocol */
#define XBEE_START 0x7e
#define XBEE_TX16_ID 0x01
#define TX16_OPTIONS 0x00
#define NO_FRAME_ID 0
#define XBEE_RFDATA_OFFSET 5
#define XBEE_RX16_ID 0x81

/** Status of the API packet receiver automata */
#define XBEE_UNINIT 0
#define XBEE_GOT_START 1
#define XBEE_GOT_LENGTH_MSB 2
#define XBEE_GOT_LENGTH_LSB 3
#define XBEE_GOT_PAYLOAD 4
#define XBEE_PAYLOAD_LEN 256

/** Receiving pprz messages */
#define STX  0x99
#define UNINIT 0
#define GOT_STX 1
#define GOT_LENGTH 2
#define GOT_PAYLOAD 3
#define GOT_CRC1 4
#define PPRZ_PAYLOAD_LEN 256
#define PPRZ_DATA_OFFSET 2

/** Receiving ocean optics messages */
#define OO_SAMPLE 'S'
#define OO_VERSION 'v'
#define OO_INTSET 'I'
#define OO_CHKMODE 'k'
#define OO_INTTIME 10 //integration time in ms
#define OO_ACK 0x06
#define OO_NAK 0x15
#define OO_STX 0x02
#define OO_START 0x98
#define OO_EB1 0xFF
#define OO_EB2 0xFD
#define OO_UNINIT 0
#define OO_INIT 1
#define OO_GOT_ITIME 2
#define OO_ENABLED_CHKSUM 3
#define OO_READY_SAMPLE 4
#define OO_GOT_STX 5
#define OO_SET_TIMESTAMP 6
#define OO_GOT_PAYLOAD 7
#define OO_GOT_CHK_A 8
#define OO_PAYLOAD_LEN 8000
#define OO_MSG_SIZE 8000
#define OO_DATA_OFFSET 2
//increase the UART rx buffer size
// #if LOG_OO_0
// #if UART_RX_BUFFER_SIZE
// #undef UART_RX_BUFFER_SIZE
// #define UART_RX_BUFFER_SIZE 8000
// #warning Increasing UART rx buffer size
// #endif
// #endif

/** logging messages **/
#define LOG_DATA_OFFSET 7
#define MSG_SIZE 256
/* logging frequency in Hz */
#define LOG_FREQ 10000
/* T0_CLK = PCLK / T0_PCLK_DIV (shall be 15MHz)
   frequency = T0_CLK / LOG_FREQ (10kHz = 100micro seconds) */
#define LOG_DIV ((PCLK / T0_PCLK_DIV) / LOG_FREQ)

#define LOG_SOURCE_UART0    0
#define LOG_SOURCE_UART1    1
#define LOG_SOURCE_I2C0     2
#define LOG_SOURCE_I2C1     3

static inline void main_init( void );
static inline void main_periodic_task( void );
int main_log(void);

void set_filename(unsigned int local, char* name);
unsigned char checksum(unsigned char start, unsigned char* data, int length);
unsigned int getclock(void);
void log_payload(int len, unsigned char source, unsigned int timestamp);
void oo_log_payload(int len, unsigned char source, unsigned int timestamp);
void log_xbee(unsigned char c, unsigned char source);
void log_pprz(unsigned char c, unsigned char source);
char log_oo(unsigned char c, unsigned char source);
int do_log(void);

DirList list;
EmbeddedFileSystem efs;
EmbeddedFile filer;
EmbeddedFile filew;

unsigned char xbeel_payload[XBEE_PAYLOAD_LEN];
unsigned char pprzl_payload[PPRZ_PAYLOAD_LEN];
unsigned char oo_payload[OO_PAYLOAD_LEN];
volatile unsigned char xbeel_payload_len;
volatile unsigned char pprzl_payload_len;
volatile unsigned int oo_payload_len;
unsigned char xbeel_error;
unsigned char pprzl_error;
unsigned char oo_error;
unsigned char log_buffer[MSG_SIZE]  __attribute__ ((aligned));
unsigned char oo_log_buffer[OO_MSG_SIZE]  __attribute__ ((aligned));
static unsigned int xbeel_timestamp = 0;
static unsigned int pprzl_timestamp = 0;
static unsigned int oo_timestamp = 0;
unsigned int nb_messages = 0;
unsigned int nb_fail_write = 0;
int bytes = 0;
unsigned int clock_msb = 0;
unsigned int clock_lsb_last = 0;

void set_filename(unsigned int local, char* name)
{
    /* do not use sprintf or similar */
    int i;

    for (i=7; i>=0; i--) {
        name[i] = (local % 10) + '0';
        local /= 10;
    }
    name[8]='.';name[9]='t';name[10]='l';name[11]='m';name[12]=0;
}

unsigned char checksum(unsigned char start, unsigned char* data, int length)
{
    int i;
    unsigned char retval = start;
    for (i=0;i<length;i++) retval += data[i];

    return retval;
}

unsigned int getclock(void)
{
    uint64_t clock;
    uint32_t clock_lsb;

    clock_lsb = T0TC;

    if (clock_lsb < clock_lsb_last) clock_msb++;
    clock_lsb_last = clock_lsb;

    clock = (((uint64_t)clock_msb << 32) | (uint64_t)clock_lsb) / LOG_DIV;

    return(clock & 0xFFFFFFFF);
}

/** Parsing a frame data and copy the payload to the log buffer */
void log_payload(int len, unsigned char source, unsigned int timestamp)
{
  unsigned char chk;

  /* start delimiter */
  log_buffer[0] = STX;

  /* length is just payload */
  log_buffer[1] = len & 0xFF;

  /* source */
  log_buffer[2] = source;

  /* add a 32bit timestamp */
  log_buffer[3] = (timestamp) & 0xFF;       // LSB first
  log_buffer[4] = (timestamp >> 8)  & 0xFF;
  log_buffer[5] = (timestamp >> 16) & 0xFF;
  log_buffer[6] = (timestamp >> 24) & 0xFF;

  /* data is already written */

  /* calculate checksum over start+length+timestamp+data */
  log_buffer[LOG_DATA_OFFSET+len] = checksum(0, &log_buffer[1], LOG_DATA_OFFSET+len-1);

  /* write data, start+length+timestamp+data+checksum */
  chk = file_write(&filew, LOG_DATA_OFFSET+len+1, log_buffer);

  if (len != chk)
  {
    nb_fail_write++;
  }

  bytes += chk;
  nb_messages++;
//  dl_parse_msg();
}

/** Parsing a XBee API frame */
void log_xbee(unsigned char c, unsigned char source)
{
  static unsigned char xbeel_status = XBEE_UNINIT;
  static unsigned char cs, payload_idx, i;

  switch (xbeel_status) {
  case XBEE_UNINIT:
    if (c == XBEE_START)
    {
// serial receive broken with MAX
#ifndef USE_MAX11040
      xbeel_timestamp = getclock();
#endif
      xbeel_status++;
    }
    break;
  case XBEE_GOT_START:
    xbeel_payload_len = c<<8;
    xbeel_status++;
    break;
  case XBEE_GOT_LENGTH_MSB:
    xbeel_payload_len |= c;
    xbeel_status++;
    payload_idx = 0;
    cs=0;
    break;
  case XBEE_GOT_LENGTH_LSB:
    xbeel_payload[payload_idx] = c;
    cs += c;
    payload_idx++;
    if (payload_idx == xbeel_payload_len)
      xbeel_status++;
    break;
  case XBEE_GOT_PAYLOAD:
    if (c + cs != 0xff)
      goto error;
    if ((xbeel_payload[0] != XBEE_RX16_ID) &&
        (xbeel_payload[0] != XBEE_TX16_ID))
      goto error;
    /* copy the XBee message to the logger buffer */
    for (i = 0; i < xbeel_payload_len-XBEE_RFDATA_OFFSET; i++) {
      log_buffer[i+LOG_DATA_OFFSET] = xbeel_payload[i+XBEE_RFDATA_OFFSET];
    }
// serial receive broken with MAX
#ifndef USE_MAX11040
    log_payload(xbeel_payload_len-XBEE_RFDATA_OFFSET, source, xbeel_timestamp);
#endif
    LED_TOGGLE(3);
    goto restart;
  }
  return;
 error:
  xbeel_error++;
 restart:
  xbeel_status = XBEE_UNINIT;
  return;
}

/** Parsing frame data from oo and copy the payload to the log buffer */
void oo_log_payload(int len, unsigned char source, unsigned int timestamp)
{
  unsigned char chk;

  /* start delimiter */
  oo_log_buffer[0] = OO_START;

  /* length is just payload */
  // oo_log_buffer[1] = len & 0xFF; //need two bytes to store length
  oo_log_buffer[1] = OO_START;

  /* source */
  oo_log_buffer[2] = source;

  /* add a 32bit timestamp */
  oo_log_buffer[3] = (timestamp) & 0xFF;       // LSB first
  oo_log_buffer[4] = (timestamp >> 8)  & 0xFF;
  oo_log_buffer[5] = (timestamp >> 16) & 0xFF;
  oo_log_buffer[6] = (timestamp >> 24) & 0xFF;

  /* data is already written */

  /* calculate checksum over start+length+timestamp+data */
  // oo_log_buffer[LOG_DATA_OFFSET+len] = checksum(0, &oo_log_buffer[1], LOG_DATA_OFFSET+len-1);

  /* write data, start+length+timestamp+data+checksum */
  chk = file_write(&filew, LOG_DATA_OFFSET+len+1, oo_log_buffer);

  // if (len != chk)
  // {
  //   nb_fail_write++;
  // }

  bytes += chk;
  nb_messages++;
//  dl_parse_msg();
}

/* logging an Ocean Optics USB4000 */
char log_oo(unsigned char c, unsigned char source)
{
  static unsigned char oo_status = OO_UNINIT;
  static unsigned int payload_idx, i, oo_chk_total, oo_chksum;
  static unsigned char oo_chk_a, oo_chk_b;

  switch (oo_status) {
  case OO_UNINIT:
    if (c == OO_ACK) //wait for ACK from version request
      oo_status++;
    break;
  case OO_INIT:
    if (c == OO_ACK) //wait for ACK from integration time set
      oo_status++;
    else if (c == OO_NAK)
      oo_status = OO_UNINIT;
    break;
  case OO_GOT_ITIME:
    if (c == OO_ACK)
      oo_status++;
    break;
  case OO_ENABLED_CHKSUM:
    if (c == OO_ACK) //wait for ACK from checksum enable
      oo_status++;
    break;
  case OO_READY_SAMPLE: //wait for STX - data will follow
    if (c == OO_STX) {
      oo_payload_len = 0;
      payload_idx = 0;
      oo_chk_total = 0;
      // oo_timestamp = getclock(); //can be large delay between this and trigger
      oo_status++;
    }
    // else
      // oo_status = OO_UNINIT;
    break;
  case OO_GOT_STX:
    oo_timestamp = getclock(); //set timestamp at reception of 1st data bit
    oo_payload[payload_idx] = c;
    payload_idx++;
    oo_status++;
    break;
  case OO_SET_TIMESTAMP: //everything else is data until end bits
    oo_payload[payload_idx] = c;
    if (c == OO_EB2) {
      if (oo_payload[payload_idx-1] == OO_EB1) {
        oo_status++;
        // oo_payload_len = payload_idx;
        // for (i = 0; i < oo_payload_len; i++) {
        //   oo_log_buffer[i+LOG_DATA_OFFSET] = oo_payload[i];
        // }
        // oo_log_payload(oo_payload_len, source, oo_timestamp);
        // LED_TOGGLE(2);
        // goto restart;       
      }
    }
    payload_idx++;
    break;
  case OO_GOT_PAYLOAD:
    // oo_chk_a = c;
    oo_payload[payload_idx] = c;
    payload_idx++;
    oo_status++;
    break;
  case OO_GOT_CHK_A:
    // oo_chk_b = c;
    oo_payload[payload_idx] = c;
    payload_idx++;
    // oo_chksum = (oo_chk_a << 8) + oo_chk_b;
    // if (oo_chksum != oo_chk_total)
    //   goto error;
    /* copy the OO message to the logger buffer */
    oo_payload_len = payload_idx;
    for (i = 0; i < oo_payload_len; i++) {
      oo_log_buffer[i+LOG_DATA_OFFSET] = oo_payload[i];
    }
    oo_log_payload(oo_payload_len, source, oo_timestamp);
    LED_TOGGLE(2);
    goto restart;
  }
  return oo_status;
 error:
  oo_error++;
 restart:
  oo_status = OO_READY_SAMPLE;
  return oo_status;
}

void log_pprz(unsigned char c, unsigned char source)
{
  static unsigned char pprzl_status = UNINIT;
  static unsigned char _ck_a, _ck_b, payload_idx, i;

  switch (pprzl_status) {
  case UNINIT:
    if (c == STX)
// serial receive broken with MAX
#ifndef USE_MAX11040
      pprzl_timestamp = getclock();
#endif
      pprzl_status++;
    break;
  case GOT_STX:
    pprzl_payload_len = c-4; /* Counting STX, LENGTH and CRC1 and CRC2 */
    _ck_a = _ck_b = c;
    pprzl_status++;
    payload_idx = 0;
    break;
  case GOT_LENGTH:
    pprzl_payload[payload_idx] = c;
    _ck_a += c; _ck_b += _ck_a;
    payload_idx++;
    if (payload_idx == pprzl_payload_len)
      pprzl_status++;
    break;
  case GOT_PAYLOAD:
    if (c != _ck_a)
      goto error;
    pprzl_status++;
    break;
  case GOT_CRC1:
    if (c != _ck_b)
      goto error;
    /* copy the pprz message to the logger buffer */
    for (i = 0; i < pprzl_payload_len; i++) {
      log_buffer[i+LOG_DATA_OFFSET] = pprzl_payload[i];
    }
// serial receive broken with MAX
#ifndef USE_MAX11040
    log_payload(pprzl_payload_len, source, pprzl_timestamp);
#endif
    LED_TOGGLE(3);
    goto restart;
  }
  return;
 error:
  pprzl_error++;
 restart:
  pprzl_status = UNINIT;
  return;
}

int do_log(void)
{
    unsigned int count;
    unsigned char name[13];
    unsigned char inc;
    int temp;

	if(efs_init(&efs, 0) != 0) {
		return(-1);
	}

    /* find an unused file number the dumb way */
    for (count = 1; count < 0xFFFFFFF; count++)
    {
        set_filename(count, name);
        if(file_fopen(&filer, &efs.myFs, name,'r')!=0) break;
        file_fclose(&filer);
    }

    if (file_fopen(&filew, &efs.myFs, name, 'w' ) != 0)
    {
		return(-1);
    }

    /* write to SD until key is pressed */
    while ((IO0PIN & (1<<LOG_STOP_KEY))>>LOG_STOP_KEY)
    {

#ifdef USE_MAX11040
      if ((max11040_data == MAX11040_DATA_AVAILABLE) &&
          (max11040_buf_in != max11040_buf_out)) {
//        LED_TOGGLE(3);
        int i;

        max11040_data = MAX11040_IDLE;

        log_buffer[LOG_DATA_OFFSET+0] = AC_ID;  // sender_id
        log_buffer[LOG_DATA_OFFSET+1] = 61;     // message_id (DL_TURB_PRESSURE_RAW)

	while(max11040_buf_in != max11040_buf_out) {
          for (i=0; i<16; i++) {
            log_buffer[LOG_DATA_OFFSET+2 + i*4 + 0] = (max11040_values[max11040_buf_out][i]      ) & 0xFF;
            log_buffer[LOG_DATA_OFFSET+2 + i*4 + 1] = (max11040_values[max11040_buf_out][i] >> 8 ) & 0xFF;
            log_buffer[LOG_DATA_OFFSET+2 + i*4 + 2] = (max11040_values[max11040_buf_out][i] >> 16) & 0xFF;
            log_buffer[LOG_DATA_OFFSET+2 + i*4 + 3] = (max11040_values[max11040_buf_out][i] >> 24) & 0xFF;

          }
          log_payload(2 + 16 * 4, LOG_SOURCE_UART0, max11040_timestamp[max11040_buf_out]);
	  i = max11040_buf_out+1;
	  if (i >= MAX11040_BUF_SIZE) i=0;
          max11040_buf_out = i;
   	}
      }
#endif


#ifdef USE_UART0
  #if LOG_OO_0
      static unsigned char oo_init = UNINIT;
      if (oo_init == OO_UNINIT) {
        Uart0Transmit(OO_VERSION); //will reply with ACK if ready
        oo_init = -1;
      }
      if (oo_init == OO_INIT) {
        Uart0Transmit(OO_INTSET); //set the spectrometer integration time
        unsigned int intTime = OO_INTTIME;
        unsigned char timeLow,timeHigh;
        timeLow = (char)intTime;
        timeHigh = (char)(intTime >> 8);
        Uart0Transmit(timeHigh);
        Uart0Transmit(timeLow);
      }
      if (oo_init == OO_GOT_ITIME) {
        Uart0Transmit(OO_SAMPLE); //tell oo to collect data
        oo_init = -1;
        // LED_TOGGLE(2);
        // sys_time_usleep(1000000);
        // oo_init++;
      }
  #endif
      temp = 0;
      while (Uart0ChAvailable() && (temp++ < 128))
      {
//		LED_TOGGLE(3);
			inc = Uart0Getch();
  #if LOG_OO_0
          // LED_TOGGLE(3);
          oo_init = log_oo(inc, LOG_SOURCE_UART0);
  #else
  #ifdef LOG_XBEE
            log_xbee(inc, LOG_SOURCE_UART0);
  #else
  #ifdef LOG_PPRZ
            log_pprz(inc, LOG_SOURCE_UART0);
  #else
  #error no log transport protocol selected UART0
  #endif
  #endif
  #endif
        }
#endif
#ifdef USE_UART1
  #if LOG_OO_1
        static unsigned char oo_init = OO_UNINIT;
        static unsigned int transmit_timestamp = 0;
        static unsigned int transmit_delta;

        //retry periodically if no response
        switch (oo_init) {
        case OO_UNINIT:
          transmit_delta = getclock() - transmit_timestamp;
          if (transmit_delta > 10000) { //10000 = 1second
            Uart1Transmit(OO_VERSION); //will reply with ACK + 2 bytes if ready
            transmit_timestamp = getclock();
          }
          break;
        case OO_INIT:
          sys_time_usleep(10000); //10000 = 10ms
          Uart1Transmit(OO_INTSET); //set the spectrometer integration time
          unsigned int intTime = OO_INTTIME; //split 16bit int into two 8 bit
          unsigned char timeLow,timeHigh;
          timeLow = (char)intTime;
          timeHigh = (char)(intTime >> 8);
          sys_time_usleep(10000); //10000 = 10ms
          Uart1Transmit(timeHigh);
          Uart1Transmit(timeLow);
          // transmit_timestamp = getclock();
          oo_init = -1;
          sys_time_usleep(10000); //10000 = 10ms
          break;
        case OO_GOT_ITIME:
          Uart1Transmit('k'); //enable transmission of spectrum checksum
          sys_time_usleep(10000);
          Uart1Transmit('!');
          Uart1Transmit(0x00);
          oo_init = -1;
          break;
        case OO_ENABLED_CHKSUM:
          Uart1Transmit('T'); //set trigger mode (see manual)
          sys_time_usleep(10000);
          Uart1Transmit(0x00);
          Uart1Transmit(0x03);
          oo_init = -1;
          break;
        case OO_READY_SAMPLE:
          Uart1Transmit(OO_SAMPLE); //tell oo to collect data
          transmit_timestamp = getclock();
          oo_init = -1;
        }
  #endif
        temp = 0;
        while (Uart1ChAvailable() && (temp++ < 128))
        {
//			LED_TOGGLE(3);
			inc = Uart1Getch();
  #if LOG_OO_1
            oo_init = log_oo(inc, LOG_SOURCE_UART1); //update oo_init for switch
  #else
  #ifdef LOG_XBEE
            log_xbee(inc, LOG_SOURCE_UART1);
  #else
  #ifdef LOG_PPRZ
            log_pprz(inc, LOG_SOURCE_UART1);
  #else
  #error no log transport protocol selected UART1
  #endif
  #endif
  #endif
        }
#endif
    }
    LED_OFF(3);

    file_fclose( &filew );
    fs_umount( &efs.myFs ) ;

    return 0;
}

int main(void)
{
  int waitloop, ledcount, logstatus;
  main_init();

#ifdef _DEBUG_BOARD_
  while(1)
  {
    if (IO0PIN & (1 << LOG_STOP_KEY))
    {
      LED_ON(LED_YELLOW);
    }
    else
    {
      LED_OFF(LED_YELLOW);
    }

    if (IO1PIN & (1 << CARD_DETECT_PIN))
    {
      LED_OFF(LED_GREEN);
    }
    else
    {
      LED_ON(LED_GREEN);
    }

    if (IO0PIN & (1 << POWER_DETECT_PIN))
//    if (IO0PIN & (1 << VBUS_PIN))
    {
      LED_ON(LED_RED);
    }
    else
    {
      LED_OFF(LED_RED);
    }
  }
#endif


  while(1)
  {
    LED_ON(2);
    logstatus = do_log();
    LED_OFF(2);

    /* if there is an error initializing the file system (ex. No SD card)
    flash the LEDs angrily*/
    if (logstatus != 0) {
      LED_ON(2);
      LED_OFF(3);
      while(1) {
        sys_time_usleep(100000);
        LED_TOGGLE(3);
        LED_TOGGLE(2);
      }
    }

    waitloop = 0;
    ledcount = 0;

    while (waitloop < 20)
    {
      sys_time_usleep(100000);

      {
        if (ledcount++ > 9) {
          ledcount=0;
          LED_ON(2);
        } else {
          LED_OFF(2);
        }
        if (((IO0PIN & _BV(LOG_STOP_KEY))>>LOG_STOP_KEY) == 1) {
          waitloop=0;
        } else {
          waitloop++;
        }
      }

      if ((IO0PIN & _BV(VBUS_PIN))>>VBUS_PIN)
      {
        LED_OFF(2);
        LED_ON(1);
        main_mass_storage();
      }
    }
    LED_ON(2);
    while (((IO0PIN & _BV(LOG_STOP_KEY))>>LOG_STOP_KEY) == 0);
  }

  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_init();
  led_init();

#ifdef USE_MAX11040
  max11040_init_ssp();
  max11040_init();
#endif

  mcu_int_enable();
}

static inline void main_periodic_task( void ) {
}
