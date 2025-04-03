#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "tft_espi_shim.h"

int main( void ) {
    // Run at 8MHz
    CLKPR = _BV( CLKPCE );
    CLKPR = 0;

    _delay_ms( 125 );

    tftShimInit( );
    tftFillScreen( TFT_GREEN );

    while ( 1 ) {
    }
}
