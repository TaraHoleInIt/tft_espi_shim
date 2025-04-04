#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "tft_espi_shim.h"

#define pulseUSI( ) do { USICR = usiClockLO; USICR = usiClockHI; } while ( 0 )

static const uint8_t usiClockHI = _BV( USIWM0 ) | ( 0 << USICS0 ) | _BV( USITC ) | _BV( USICLK );
static const uint8_t usiClockLO = _BV( USIWM0 ) | ( 0 << USICS0 ) | _BV( USITC );

uint8_t _tftBusWrite8( uint8_t data ) {
    uint8_t hi = _BV( USIWM0 ) | ( 0 << USICS0 ) | _BV( USITC ) | _BV( USICLK );
    uint8_t lo = _BV( USIWM0 ) | ( 0 << USICS0 ) | _BV( USITC );

    USIDR = data;

    USICR = lo; // MSB
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo; // LSB
    USICR = hi;

    return USIDR;
}

uint16_t _tftBusWrite16( uint16_t data ) {
    uint8_t hi = _BV( USIWM0 ) | ( 0 << USICS0 ) | _BV( USITC ) | _BV( USICLK );
    uint8_t lo = _BV( USIWM0 ) | ( 0 << USICS0 ) | _BV( USITC );
    uint16_t res = 0;

    USIDR = ( data >> 8 ) & 0xFF;

    USICR = lo; // MSB
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo; // LSB
    USICR = hi;

    res = ( USIDR << 8 );

    USIDR = data & 0xFF;

    USICR = lo; // MSB
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo;
    USICR = hi;

    USICR = lo; // LSB
    USICR = hi;

    return res | USIDR;
}

void _tftDelayMSec( uint32_t msec ) {
    for ( int i = 0; i < msec; i++ )
        _delay_ms( 1 );
}

void _tftDCHigh( void ) {
    DC_OUT |= _BV( DC_PIN );
}

void _tftDCLow( void ) {
    DC_OUT &= ~_BV( DC_PIN );
}

void _tftCSHigh( void ) {
    CS_OUT |= _BV( CS_PIN );
}

void _tftCSLow( void ) {
    CS_OUT &= ~_BV( CS_PIN );
}

void _tftRSTHigh( void ) {
    RST_OUT |= _BV( RST_PIN );
}

void _tftRSTLow( void ) {
    RST_OUT &= ~_BV( RST_PIN );
}

void _tftPinSetup( void ) {
    DC_OUT |= _BV( DC_PIN );
    DC_DIR |= _BV( DC_PIN );

    CS_OUT |= _BV( CS_PIN );
    CS_DIR |= _BV( CS_PIN );

    RST_OUT |= _BV( RST_PIN );
    RST_DIR |= _BV( RST_PIN );

    SPI_DIR |= _BV( SPI_MOSI ) | _BV( SPI_SCLK );
}

void tftPushPixels( const uint16_t* buf, int32_t lenWords ) {
    uint16_t data = 0;

    tftBeginPixels( );
        while ( lenWords-- ) {
            data = pgm_read_word( buf++ );

            USIDR = ( data >> 8 ) & 0xFF;

            pulseUSI( );    // MSB
            pulseUSI( );
            pulseUSI( );
            pulseUSI( );
            pulseUSI( );
            pulseUSI( );
            pulseUSI( );
            pulseUSI( );    // LSB

            ( void ) USIDR;
            USIDR = data & 0xFF;

            pulseUSI( );    // MSB
            pulseUSI( );
            pulseUSI( );
            pulseUSI( );
            pulseUSI( );
            pulseUSI( );
            pulseUSI( );
            pulseUSI( );    // LSB

            ( void ) USIDR;
        }
    tftEndPixels( );
}
