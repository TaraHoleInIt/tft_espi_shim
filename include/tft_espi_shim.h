// Copyright 2025 Tara Keeling. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef _TFT_ESPI_SHIM_H_
#define _TFT_ESPI_SHIM_H_

#ifndef WEAK
#define WEAK __attribute__( ( weak ) )
#endif

#ifndef PROGMEM
#define PROGMEM
#endif

#ifndef pgm_read_byte
#define pgm_read_byte( addr ) ( *addr )
#endif

#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xD69A      /* 211, 211, 211 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFE19      /* 255, 192, 203 */ //Lighter pink, was 0xFC9F
#define TFT_BROWN       0x9A60      /* 150,  75,   0 */
#define TFT_GOLD        0xFEA0      /* 255, 215,   0 */
#define TFT_SILVER      0xC618      /* 192, 192, 192 */
#define TFT_SKYBLUE     0x867D      /* 135, 206, 235 */
#define TFT_VIOLET      0x915C      /* 180,  46, 226 */

#ifndef TAB_COLOUR
  #define TAB_COLOUR 0
#endif

WEAK void _tftBusInit( uint32_t freq );
WEAK void _tftPinSetOutput( int pin );
WEAK void _tftPinSetInput( int pin );
WEAK int _tftPinRead( int pin );
WEAK void _tftPinWrite( int pin, int level );
WEAK uint8_t _tftBusWrite8( uint8_t data );
WEAK uint16_t _tftBusWrite16( uint16_t data );
WEAK void _tftDelayMSec( uint32_t msec );

void tftShimInit( void );
void tftRotate( uint8_t m );
void tftSetAddressWindow( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 );
void tftBeginPixels( void );
void tftEndPixels( void );
void tftFillScreen( uint16_t color );

WEAK void tftPushColor( uint16_t color, int count );

#endif
