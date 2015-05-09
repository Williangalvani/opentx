/*
 * Authors (alphabetical order)
 * - Andre Bernet <bernet.andre@gmail.com>
 * - Andreas Weitl
 * - Bertrand Songis <bsongis@gmail.com>
 * - Bryan J. Rentoul (Gruvin) <gruvin@gmail.com>
 * - Cameron Weeks <th9xer@gmail.com>
 * - Erez Raviv
 * - Gabriel Birkus
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michael Blandford
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini <romolo.manfredini@gmail.com>
 * - Thomas Husterer
 *
 * opentx is based on code named
 * gruvin9x by Bryan J. Rentoul: http://code.google.com/p/gruvin9x/,
 * er9x by Erez Raviv: http://code.google.com/p/er9x/,
 * and the original (and ongoing) project by
 * Thomas Husterer, th9x: http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "opentx.h"

#ifdef SSD1306
 #define ALWAYS_INLINE   __attribute__((always_inline))

 #define ROTATE_SCREEN 1
 #define OUT_C_LCD_SCL OUT_C_LCD_RnW     // PC4
 #define OUT_C_LCD_SI  OUT_C_LCD_E       // PC5

 static void lcdSendBit(uint8_t b, uint8_t v0, uint8_t v1) ALWAYS_INLINE;
 static void lcdSend8bits(uint8_t val, uint8_t v0, uint8_t v1) ALWAYS_INLINE;
 static void lcdSendDataBits(uint8_t *p, uint8_t COLUMN_START_LO) ALWAYS_INLINE;
 static void lcdSendCtlBits(uint8_t val);
 inline void lcdSendCtl(uint8_t val) { lcdSendCtlBits(val); }
#endif

#define delay_1us() _delay_us(1)
#define delay_2us() _delay_us(2)
void delay_1_5us(uint16_t ms)
{
  for (uint16_t i=0; i<ms; i++) delay_1us();
}

#ifndef SSD1306
  void lcdSendCtl(uint8_t val)
  {
    PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_CS1);
  #ifdef LCD_MULTIPLEX
    DDRA = 0xFF; // set LCD_DAT pins to output
  #endif
    PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_A0);
    PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_RnW);
    PORTA_LCD_DAT = val;
    PORTC_LCD_CTRL |=  (1<<OUT_C_LCD_E);
    PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_E);
    PORTC_LCD_CTRL |=  (1<<OUT_C_LCD_A0);
  #ifdef LCD_MULTIPLEX
    DDRA = 0x00; // set LCD_DAT pins to input
  #endif
    PORTC_LCD_CTRL |=  (1<<OUT_C_LCD_CS1);
  }
#endif

#if defined(PCBSTD) && defined(VOICE)
volatile uint8_t LcdLock ;
#define LCD_LOCK() LcdLock = 1
#define LCD_UNLOCK() LcdLock = 0
#else
#define LCD_LOCK()
#define LCD_UNLOCK()
#endif

const static pm_uchar lcdInitSequence[] PROGMEM =
{
   #ifdef SSD1306
     0xAE,         // DON = 0: display OFF
     0xD5, 0x80,   // set display clock 100 frames/sec
     0xA8, 0x3F,   // set multiplex ratio 1/64 duty
     0xD3, 0x00,   // set display offset 0
     0x8D, 0x14,   // enable embedded DC/DC conveter
     0xD9, 0xF1,   // set precharge 15 clocks, discharge 1 clock
     0xDA, 0x12,   // set COM pins hardware configuration
     0xDB, 0x40,   // set VCOMH deselect level -undocumented
     #if ROTATE_SCREEN
      0xA1,         // ADC = 1: reverse direction(SEG128->SEG1)
      0xC8,         // SHL = 1: reverse direction (COM64->COM1)
     #else
      0xA0,         // ADC = 0: normal direction(SEG1->SEG128)
      0xC0,         // SHL = 0: normal direction (COM1->COM64)
     #endif

   #else
     0xe2, //Initialize the internal functions
     0xae, //DON = 0: display OFF
     0xa1, //ADC = 1: reverse direction(SEG132->SEG1)
     0xA6, //REV = 0: non-reverse display
     0xA4, //EON = 0: normal display. non-entire
     0xA2, // Select LCD bias=0
     0xC0, //SHL = 0: normal direction (COM1->COM64)
     0x2F, //Control power circuit operation VC=VR=VF=1
     0x25, //Select int resistance ratio R2 R1 R0 =5
     0x81, //Set reference voltage Mode
     0x22, // 24 SV5 SV4 SV3 SV2 SV1 SV0 = 0x18
   #endif
   0xAF  //DON = 1: display ON
};

inline void lcdInit()
{
  // /home/thus/txt/datasheets/lcd/KS0713.pdf
  // ~/txt/flieger/ST7565RV17.pdf  from http://www.glyn.de/content.asp?wdid=132&sid=

  LCD_LOCK();
  PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_RES);  //LCD_RES
  delay_2us();
  PORTC_LCD_CTRL |= (1<<OUT_C_LCD_RES); //  f524  sbi 0x15, 2 IOADR-PORTC_LCD_CTRL; 21           1
  delay_1_5us(1500);
  for (uint8_t i=0; i<DIM(lcdInitSequence); i++) {
    lcdSendCtl(pgm_read_byte(&lcdInitSequence[i])) ;
  }
  g_eeGeneral.contrast = 0x22;
  lcdSetContrast(); //necessary?
  LCD_UNLOCK();
}

void lcdSetRefVolt(uint8_t val)
{
  LCD_LOCK();
  lcdSendCtl(0x81);
  #ifdef SSD1306
     lcdSendCtl((val << 2) + 3);  // [3-255]
  #else
   lcdSendCtl(val);             // [0-63]
  #endif
LCD_UNLOCK();
}


#ifdef SSD1306
// NOTE: ST7565 SCLK min period is 50ns (100ns?)
// single bit write takes 5 cycles = 312.5ns @16MHz clock
static void lcdSendBit(uint8_t b, uint8_t v0, uint8_t v1)
{
  PORTC_LCD_CTRL = v0;                  // out 0x15, r19  ; 1 cycle
  if (b != 0)                           // sbrc r24, 7    ; 1 cycle
    PORTC_LCD_CTRL = v1;                // out 0x15, r18  ; 1 cycle
  PORTC_LCD_CTRL |= (1<<OUT_C_LCD_SCL); // sbi 0x15, 4    ; 2 cycles
}

static void lcdSendCtlBits(uint8_t val)
{
  uint8_t v0c = 0xC5; // PC7=1,PC6=1,SI=0,SCL=0,A0=0,RES=1,CS1=0,PC0=1
  uint8_t v1c = 0xE5; // PC7=1,PC6=1,SI=1,SCL=0,A0=0,RES=1,CS1=0,PC0=1
  for (uint8_t n = 8; n > 0; n--) {
    lcdSendBit((val & 0x80), v0c, v1c);
    val <<= 1;
  }
  PORTC_LCD_CTRL |= (1<<OUT_C_LCD_CS1);   // disable chip select
}


static void lcdSend8bits(uint8_t val, uint8_t v0, uint8_t v1)
{
  lcdSendBit((val & 0x80), v0, v1);
  lcdSendBit((val & 0x40), v0, v1);
  lcdSendBit((val & 0x20), v0, v1);
  lcdSendBit((val & 0x10), v0, v1);
  lcdSendBit((val & 0x08), v0, v1);
  lcdSendBit((val & 0x04), v0, v1);
  lcdSendBit((val & 0x02), v0, v1);
  lcdSendBit((val & 0x01), v0, v1);
}

static void lcdSendDataBits(uint8_t *p, uint8_t COLUMN_START_LO)
{
  uint8_t v0c = 0xC5; // PC7=1,PC6=1,SI=0,SCL=0,A0=0,RES=1,CS1=0,PC0=1
  uint8_t v1c = 0xE5; // PC7=1,PC6=1,SI=1,SCL=0,A0=0,RES=1,CS1=0,PC0=1
  uint8_t v0d = 0xCD; // PC7=1,PC6=1,SI=0,SCL=0,A0=1,RES=1,CS1=0,PC0=1
  uint8_t v1d = 0xED; // PC7=1,PC6=1,SI=1,SCL=0,A0=1,RES=1,CS1=0,PC0=1
  for(uint8_t y=0xB0; y < 0xB8; y++) {
    lcdSend8bits(COLUMN_START_LO, v0c, v1c);
    lcdSend8bits(0x10, v0c, v1c);  //column addr 0
    lcdSend8bits(y, v0c, v1c);     //page addr y

    for(uint8_t x=32; x>0; x--){
       lcdSend8bits(*p++, v0d, v1d);
       lcdSend8bits(*p++, v0d, v1d);
       lcdSend8bits(*p++, v0d, v1d);
       lcdSend8bits(*p++, v0d, v1d);
    }
  }
}
#endif

void lcdRefresh()
{
  LCD_LOCK();
  uint8_t *p=displayBuf;
  #ifdef SSD1306
    lcdSendDataBits(p, 0x00);
    PORTC_LCD_CTRL |= (1<<OUT_C_LCD_CS1);   // disable chip select
  #else
    for(uint8_t y=0; y < 8; y++) {
      lcdSendCtl(0x04);
      lcdSendCtl(0x10); //column addr 0
      lcdSendCtl( y | 0xB0); //page addr y
      PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_CS1);
  #ifdef LCD_MULTIPLEX
      DDRA = 0xFF; // set LCD_DAT pins to output
  #endif
      PORTC_LCD_CTRL |=  (1<<OUT_C_LCD_A0);
      PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_RnW);
      for (xcoord_t x=LCD_W; x>0; --x) {
        PORTA_LCD_DAT = *p++;
        PORTC_LCD_CTRL |= (1<<OUT_C_LCD_E);
        PORTC_LCD_CTRL &= ~(1<<OUT_C_LCD_E);
      }
      PORTC_LCD_CTRL |=  (1<<OUT_C_LCD_A0);
      PORTC_LCD_CTRL |=  (1<<OUT_C_LCD_CS1);
    }
  #endif
  LCD_UNLOCK();
}

