/**
 * Copyright 2025 Tara Keeling. All rights reserved.
 * Use of this source code is governed by a BSD-style
 * license that can be found in the LICENSE file.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "tft_espi_shim.h"

#define DC_C _tftPinWrite( TFT_DC, 0 )
#define DC_D _tftPinWrite( TFT_DC, 1 )

#if defined GC9A01_DRIVER
#include "../TFT_eSPI/TFT_Drivers/GC9A01_Defines.h"
#endif

#if defined ILI9341_DRIVER
#include "../TFT_eSPI/TFT_Drivers/ILI9341_Defines.h"
#endif

#if defined SSD1351_DRIVER
#include "../TFT_eSPI/TFT_Drivers/SSD1351_Defines.h"
#endif

#if defined ST7735_DRIVER
#include "../TFT_eSPI/TFT_Drivers/ST7735_Defines.h"
#endif

#if defined ST7789_DRIVER
#include "../TFT_eSPI/TFT_Drivers/ST7789_Defines.h"
#endif

#if defined ST7796_DRIVER
#include "../TFT_eSPI/TFT_Drivers/ST7796_Defines.h"
#endif

static void delay( int ms );
static void begin_tft_write( void );
static void end_tft_write( void );
static void writecommand( uint8_t cmd );
static void writedata( uint8_t data );
static void fillScreen( uint16_t color );
static void commandList( const uint8_t* l );

static int _width, _init_width = TFT_WIDTH;
static int _height, _init_height = TFT_HEIGHT;

static int colstart = 0;
static int rowstart = 0;

static int tabcolor = 0;

static void commandList( const uint8_t* l ) {
    uint8_t delayTime = 0;
    uint8_t cmdCount = 0;
    uint8_t argCount = 0;
    int i = 0;
    int j = 0;

    cmdCount = pgm_read_byte( l++ );

    for ( i = 0; i < cmdCount; i++ ) {
        writecommand( pgm_read_byte( l++ ) );

        argCount = pgm_read_byte( l++ );
        delayTime = argCount & TFT_INIT_DELAY;
        argCount &= ~TFT_INIT_DELAY;

        for ( j = 0; j < argCount; j++ ) {
            writedata( pgm_read_byte( l++ ) );
        }

        if ( delayTime ) {
            delayTime = pgm_read_byte( l++ );
            _tftDelayMSec( ( delayTime == 255 ) ? 500 : delayTime );
        }
    }
}

static void delay( int ms ) {
    _tftDelayMSec( ms );
}

static void begin_tft_write( void ) {
    _tftPinWrite( TFT_CS, 0 );
}

static void end_tft_write( void ) {
    _tftPinWrite( TFT_CS, 1 );
}

static void writecommand( uint8_t cmd ) {
    begin_tft_write( );
        DC_C;
            _tftBusWrite8( cmd );
        DC_D;
    end_tft_write( );
}

static void writedata( uint8_t data ) {
    begin_tft_write( );
        DC_D;
        _tftBusWrite8( data );
        _tftPinWrite( TFT_CS, 0 );
    end_tft_write( );
}

static void fillScreen( uint16_t color ) {
    tftSetAddressWindow( 0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1 );

    for ( int i = 0; i < TFT_WIDTH * TFT_HEIGHT; i++ ) {
        _tftBusWrite16( color );
    }
}

void tftShimInit( void ) {
    _tftBusInit( 1000000 );

    if ( TFT_CS > -1 ) {
        _tftPinWrite( TFT_CS, 1 );        
        _tftPinSetOutput( TFT_CS );
    }

    if ( TFT_DC > -1 ) {
        _tftPinWrite( TFT_DC, 0 );        
        _tftPinSetOutput( TFT_DC );
    }

#if defined TFT_RST
    if ( TFT_RST > -1 ) {
        writecommand( 0x00 );

        _tftPinWrite( TFT_RST, 1 );
        _tftPinSetOutput( TFT_RST );

        _tftDelayMSec( 5 );

        _tftPinWrite( TFT_RST, 0 );
        _tftDelayMSec( 20 );

        _tftPinWrite( TFT_RST, 1 );
    } else {
#else
    {
#endif
        writecommand( TFT_SWRST );
    }

    _tftDelayMSec( 150 );

    begin_tft_write( );

#if defined GC9A01_DRIVER
#include "../TFT_eSPI/TFT_Drivers/GC9A01_Init.h"
#endif

#if defined ILI9341_DRIVER
#include "../TFT_eSPI/TFT_Drivers/ILI9341_Init.h"
#endif

#if defined SSD1351_DRIVER
#include "../TFT_eSPI/TFT_Drivers/SSD1351_Init.h"
#endif

#if defined ST7735_DRIVER
#include "../TFT_eSPI/TFT_Drivers/ST7735_Init.h"
tabcolor = TAB_COLOUR;
#endif

#if defined ST7789_DRIVER
#include "../TFT_eSPI/TFT_Drivers/ST7789_Init.h"
#endif

#if defined ST7796_DRIVER
        #include "../TFT_eSPI/TFT_Drivers/ST7796_Init.h"
#endif

    end_tft_write( );

    tftRotate( 0 );
}

void tftRotate( uint8_t m ) {
    uint8_t rotation = 0;

    begin_tft_write( );

#if defined GC9A01_DRIVER
#include "../TFT_eSPI/TFT_Drivers/GC9A01_Rotation.h"
#endif

#if defined ILI9341_DRIVER
    #include "../TFT_eSPI/TFT_Drivers/ILI9341_Rotation.h"
#endif

#if defined SSD1351_DRIVER
#include "../TFT_eSPI/TFT_Drivers/SSD1351_Rotation.h"
#endif

#if defined ST7735_DRIVER
#include "../TFT_eSPI/TFT_Drivers/ST7735_Rotation.h"
#endif

#if defined ST7796_DRIVER
        #include "../TFT_eSPI/TFT_Drivers/ST7796_Rotation.h"
#endif

#if defined ST7789_DRIVER
#include "../TFT_eSPI/TFT_Drivers/ST7789_Rotation.h"
#endif

    _tftDelayMSec( 1 );
    end_tft_write( );
}

void tftSetAddressWindow( uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1 ) {
    begin_tft_write( );
#if defined SSD1351_DRIVER
        // Col address
        DC_C;
        _tftBusWrite8( TFT_CASET );

        DC_D;
        _tftBusWrite8( x0 );
        _tftBusWrite8( x1 );

        // Row address
        DC_C;
        ( *hw.busWrite8 )( TFT_PASET );

        DC_D;
        _tftBusWrite8( y0 );
        _tftBusWrite8( y1 );

        DC_C;
        _tftBusWrite8( TFT_RAMWR );
        DC_D;
#else
        // Col address
        DC_C;
        _tftBusWrite8( TFT_CASET );

        DC_D;
        _tftBusWrite16( x0 + colstart );
        _tftBusWrite16( x1 + colstart );

        // Row address
        DC_C;
        _tftBusWrite8( TFT_PASET );

        DC_D;
        _tftBusWrite16( y0 + rowstart );
        _tftBusWrite16( y1 + rowstart );

        DC_C;
        _tftBusWrite8( TFT_RAMWR );
        DC_D;
#endif
    //end_tft_write( );
}

void tftBeginPixels( void ) {
    begin_tft_write( );
    DC_D;
}

void tftEndPixels( void ) {
    end_tft_write( );
}

void tftFillScreen( uint16_t color ) {
    tftSetAddressWindow( 0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1 );

    tftBeginPixels( );
        //fillScreen( color );
        tftPushColor( color, TFT_WIDTH * TFT_HEIGHT );
    tftEndPixels( );
}

WEAK void tftPushColor( uint16_t color, int count ) {
    while ( count-- )
        _tftBusWrite16( color );
}
