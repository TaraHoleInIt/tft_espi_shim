// Copyright 2025 Tara Keeling. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef _TFT_ESPI_SHIM_H_
#define _TFT_ESPI_SHIM_H_

#ifndef TFT_WIDTH
#error TFT_WIDTH Must be defined!
#endif

#ifndef TFT_HEIGHT
#error TFT_HEIGHT Must be defined!
#endif

#ifndef TFT_DC
#error TFT_DC Must be defined!
#endif

#ifndef TFT_CS
#error TFT_CS Must be defined!
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

typedef struct {
    void ( *setPinOutput ) ( int pin );
    void ( *setPinInput ) ( int pin );
    uint8_t ( *pinRead ) ( int pin );
    void ( *pinWrite ) ( int pin, int level );
    uint8_t ( *busWrite8 ) ( uint8_t data );
    uint16_t ( *busWrite16 ) ( uint16_t data );
    void ( *msecDelay ) ( int32_t msec );
} ShimHWAPI;

void tftShimInit( ShimHWAPI* api );
void tftRotate( uint8_t m );
void tftSetAddressWindow( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 );
void tftBeginPixels( void );
void tftEndPixels( void );

#endif
