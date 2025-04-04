#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "tft_espi_shim.h"

static void drawCatAt( int32_t x, int32_t y );

extern const unsigned char CATIMAGE_pixel_data[ 64 * 48 * 2 ] PROGMEM;

int main( void ) {
    int x = 0;
    int y = 0;

    tftShimInit( );

    while ( 1 ) {
        tftFillScreen( TFT_RED );
        _delay_ms( 1000 );

        tftFillScreen( TFT_GREEN );
        _delay_ms( 1000 );

        tftFillScreen( TFT_BLUE );
        _delay_ms( 1000 );

        for ( y = 0; y < 240; y+= 48 ) {
            for ( x = 0; x < 240; x+= 48 ) {
                drawCatAt( x, y );
            }
        }

        _delay_ms( 5000 );
    }
}

static void drawCatAt( int32_t x, int32_t y ) {
    int32_t x2 = x + 63;
    int32_t y2 = y + 47;

    tftSetAddressWindow( x, y, x2, y2 );
    tftPushPixels( ( const uint16_t* ) CATIMAGE_pixel_data, sizeof( CATIMAGE_pixel_data ) / 2 );
}
