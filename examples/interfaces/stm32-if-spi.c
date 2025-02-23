#include "stm32u0xx_hal.h"
#include "tft_espi_shim.h"

GPIO_TypeDef* getPortFromIOPin( uint8_t ioPortPin );
int getPinFromIOPin( uint8_t ioPortPin );

GPIO_TypeDef* getPortFromIOPin( uint8_t ioPortPin ) {
    ioPortPin = ( ioPortPin >> 4 ) & 0x0F;

    switch ( ioPortPin ) {
        case 0: return GPIOA;
        case 1: return GPIOB;
        case 2: return GPIOC;
        case 3: return GPIOD;
        case 4: return GPIOE;
        case 5: return GPIOF;
        default: break;
    };

    return GPIOA;
}

int getPinFromIOPin( uint8_t ioPortPin ) {
    return ( ioPortPin & 0x0F );
}

void gpioSetMode( GPIO_TypeDef* PORT, int pinNo, int mode, int af, int speed, int pupd, int initialLevel ) {
    int r = 0;

	mode &= 0x03;
	speed &= 0x03;
	pupd &= 0x03;
	initialLevel &= 0x01;

	PORT->MODER &= ~( 0b11 << ( pinNo * 2 ) );		// Clear out mode bits
	PORT->BSRR |= ( initialLevel << pinNo ) | ( ( ! initialLevel ) << ( pinNo + 16 ) );	// Set/reset initial value
	PORT->MODER |= ( mode << ( pinNo * 2 ) );		// Set GPIO mode

	PORT->OSPEEDR &= ( 0b11 << ( pinNo * 2 ) );		// Clear out speed bits
	PORT->OSPEEDR |= ( speed << ( pinNo * 2 ) );	// Set new speed

	PORT->PUPDR &= ( 0b11 << ( pinNo * 2 ) );		// Clear out PUPD bits
	PORT->PUPDR |= ( pupd << ( pinNo * 2 ) );		// Set pull-up/pull-down mode

    if ( af ) {
        if ( pinNo > 7 ) {
            pinNo-= 8;
            r++;
        }

        PORT->AFR[ r ] &= ~( 0b1111 << ( pinNo * 4 ) );
        PORT->AFR[ r ] |= ( af << ( pinNo * 4 ) );
    }
}

void _tftBusInit( uint32_t freq ) {
    RCC->APBENR2 |= RCC_APBENR2_SPI1EN;
    asm volatile( "nop" );

    RCC->APBRSTR2 |= RCC_APBRSTR2_SPI1RST;
    asm volatile( "nop" );

    RCC->APBRSTR2 &= ~RCC_APBRSTR2_SPI1RST;
    asm volatile( "nop" );

    if ( TFT_CS >= 0 ) {
        gpioSetMode(
            getPortFromIOPin( TFT_MOSI ),
            getPinFromIOPin( TFT_MOSI ),
            GPIO_MODE_OUTPUT_PP,
            TFT_CS,
            GPIO_SPEED_FREQ_VERY_HIGH,
            GPIO_NOPULL,
            1
        );
    }

    gpioSetMode(
        getPortFromIOPin( TFT_MOSI ),
        getPinFromIOPin( TFT_MOSI ),
        GPIO_MODE_AF_PP,
        5,
        GPIO_SPEED_FREQ_VERY_HIGH,
        GPIO_NOPULL,
        0
    );

    gpioSetMode(
        getPortFromIOPin( TFT_SCLK ),
        getPinFromIOPin( TFT_SCLK ),
        GPIO_MODE_AF_PP,
        5,
        GPIO_SPEED_FREQ_VERY_HIGH,
        GPIO_NOPULL,
        0
    );

    gpioSetMode(
        getPortFromIOPin( TFT_MISO ),
        getPinFromIOPin( TFT_MISO ),
        GPIO_MODE_INPUT,
        5,
        GPIO_SPEED_FREQ_VERY_HIGH,
        GPIO_NOPULL,
        0
    );

    TFT_SPI_PORT->CR1 = \
        SPI_CR1_SSM | \
        SPI_CR1_MSTR | \
        0 | \
        SPI_CR1_SSI
    ;
    TFT_SPI_PORT->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_FRXTH;
    TFT_SPI_PORT->CR1 |= SPI_CR1_SPE;
}

void _tftPinSetOutput( int pin ) {
    gpioSetMode(
        getPortFromIOPin( pin ),
        getPinFromIOPin( pin ),
        GPIO_MODE_OUTPUT_PP,
        0,
        GPIO_SPEED_FREQ_VERY_HIGH,
        GPIO_NOPULL,
        0
    );
}

void _tftPinSetInput( int pin ) {
    gpioSetMode(
        getPortFromIOPin( pin ),
        getPinFromIOPin( pin ),
        GPIO_MODE_INPUT,
        0,
        GPIO_SPEED_FREQ_VERY_HIGH,
        GPIO_PULLUP,
        0
    );
}

int _tftPinRead( int pin ) {
    return ( getPortFromIOPin( pin )->IDR & ( 1 << getPinFromIOPin( pin ) ) ) ? 1 : 0;
}

void _tftPinWrite( int pin, int level ) {
    GPIO_TypeDef* p = NULL;

    p = getPortFromIOPin( pin );
    pin = getPinFromIOPin( pin );
    level &= 0x01;

    p->BSRR |= ( level << pin ) | ( ( ! level ) << ( pin + 16 ) );
}


uint8_t _tftBusWrite8( uint8_t data ) {
    while ( ! ( TFT_SPI_PORT->SR & SPI_SR_TXE ) )
    ;

    *( ( __IO uint8_t* ) &TFT_SPI_PORT->DR ) = data;

    while ( ! ( TFT_SPI_PORT->SR & SPI_SR_RXNE ) )
    ;

    return *( ( __IO uint8_t* ) &TFT_SPI_PORT->DR );
}

uint16_t _tftBusWrite16( uint16_t data ) {
    uint16_t res = 0;

    res = _tftBusWrite8( ( data >> 8 ) & 0xFF ) << 8;
    res |= _tftBusWrite8( data & 0xFF );

    return res;
}

void _tftDelayMSec( uint32_t msec ) {
    HAL_Delay( msec );
}
