#include <stdio.h>
#include <stdlib.h>
#include "stm32u0xx_hal.h"
#include "tft_espi_shim.h"

#define SPI_TX_DMA DMA1_Channel1

static void setupSPIDMA16( const void* src, uint16_t count, uint32_t flags );
static GPIO_TypeDef* getPortFromIOPin( uint8_t ioPortPin );
static int getPinFromIOPin( uint8_t ioPortPin );
static int spiSetFreq( int newFreq );

volatile int isInTXDMA = 0;

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

    // TODO
    // Fix this
    return GPIOA;
}

static int getPinFromIOPin( uint8_t ioPortPin ) {
    return ( ioPortPin & 0x0F );
}

static void gpioSetMode( GPIO_TypeDef* PORT, int pinNo, int mode, int af, int speed, int pupd, int initialLevel ) {
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
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
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
            GPIO_PULLUP,
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
        SPI_CR1_SSI
    ;
    TFT_SPI_PORT->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_FRXTH | SPI_CR2_TXDMAEN;
    TFT_SPI_PORT->CR1 |= SPI_CR1_SPE;

    printf( "SPI Frequency: %uMHz\n", ( spiSetFreq( freq ) ) / 1000000 );
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

void spiWaitTX( void ) {
    while ( ! ( TFT_SPI_PORT->SR & SPI_SR_TXE ) )
    ;
}

void spiWaitRX( void ) {
    while ( ! ( TFT_SPI_PORT->SR & SPI_SR_RXNE ) )
    ;
}

void spiWaitBusy( void ) {
    while ( TFT_SPI_PORT->SR & SPI_SR_BSY )
    ;
}

uint8_t _tftBusWrite8( uint8_t data ) {
    spiWaitTX( );
        *( ( __IO uint8_t* ) &TFT_SPI_PORT->DR ) = data;
    spiWaitRX( );

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

int spiSetFreq( int newFreq ) {
    static const uint8_t prescalers[ 8 ] = {
        0 << SPI_CR1_BR_Pos,    // CLK / 2
        1 << SPI_CR1_BR_Pos,    // CLK / 4
        2 << SPI_CR1_BR_Pos,    // CLK / 8
        3 << SPI_CR1_BR_Pos,    // CLK / 16
        4 << SPI_CR1_BR_Pos,    // CLK / 32
        5 << SPI_CR1_BR_Pos,    // CLK / 64
        6 << SPI_CR1_BR_Pos,    // CLK / 128
        7 << SPI_CR1_BR_Pos,    // CLK / 256
    };
    uint32_t periphClk = 0;
    int freqActual = 0;
    int freqClosest = 0;
    int freqDiff = 0;
    int smallestDiff = 0;
    int pscShift = 0;
    int pscIdx = 0;
    int i = 0;
    int wasEnabled = 0;

    spiWaitBusy( );

    wasEnabled = ( TFT_SPI_PORT->CR1 & SPI_CR1_SPE );
    periphClk = HAL_RCC_GetPCLK1Freq( );
    freqClosest = periphClk >> 1;
    freqDiff = freqClosest;
    smallestDiff = freqDiff;

    for ( i = 0; i < 8; i++ ) {
        pscShift = i + 1;

        freqActual = periphClk >> pscShift;
        freqDiff = abs( freqActual - newFreq );

        if ( freqDiff < smallestDiff ) {
            smallestDiff = freqDiff;
            freqClosest = freqActual;
            pscIdx = i;
        }
    }

    // Clamp the prescaler index to be extra sure...
    pscIdx &= 0x07;

    TFT_SPI_PORT->CR1 &= ~SPI_CR1_SPE;
        TFT_SPI_PORT->CR1 &= ~SPI_CR1_BR_Msk;
        TFT_SPI_PORT->CR1 |= prescalers[ pscIdx ];
    TFT_SPI_PORT->CR1 |= wasEnabled;

    return freqClosest;
}

static void setupSPIDMA16( const void* src, uint16_t count, uint32_t flags ) {
    while ( SPI_TX_DMA->CNDTR > 0 )
    ;

    SPI_TX_DMA->CMAR = ( uint32_t ) src;
    SPI_TX_DMA->CPAR = ( uint32_t ) ( ( __IO uint16_t* ) &TFT_SPI_PORT->DR );
    SPI_TX_DMA->CNDTR = count;
    SPI_TX_DMA->CCR = \
        DMA_CCR_PL_0 | DMA_CCR_PL_1 |   // Very high priority
        DMA_CCR_MSIZE_0 |               // 16 Bit memory
        DMA_CCR_PSIZE_0 |               // 16 Bit peripheral
        DMA_CCR_DIR               |
        0
    ;

    DMAMUX1_Channel0->CCR = ( 37 << DMAMUX_CxCR_DMAREQ_ID_Pos );
    SPI_TX_DMA->CCR |= DMA_CCR_EN;
}

void tftPushColor( uint16_t color, int count ) {
    while ( SPI_TX_DMA->CNDTR )
    ;

    spiWaitBusy( );
    spiWaitTX( );

    // TARA:
    // CS MUST BE HIGH WHEN CHANGING SPI MODE
    _tftPinWrite( TFT_CS, 1 );
        TFT_SPI_PORT->CR1 &= ~SPI_CR1_SPE;
            TFT_SPI_PORT->CR2 &= ~SPI_CR2_DS_Msk;
            TFT_SPI_PORT->CR2 |= ( 0x0F << SPI_CR2_DS_Pos );
        TFT_SPI_PORT->CR1 |= SPI_CR1_SPE;
    _tftPinWrite( TFT_CS, 0 );

    setupSPIDMA16( 
        ( uint16_t* ) &color, 
        count,
        0
    );

    // TODO
    // Prooobably shouldn't block...
    // Figure out what to do
    while ( SPI_TX_DMA->CNDTR )
    ;

    // Make sure CS is HIGH before changing SPI mode again
    _tftPinWrite( TFT_CS, 1 );

    // Turn off DMA
    SPI_TX_DMA->CCR &= ~DMA_CCR_EN;

    TFT_SPI_PORT->CR1 &= ~SPI_CR1_SPE;
        TFT_SPI_PORT->CR2 &= ~SPI_CR2_DS_Msk;
        TFT_SPI_PORT->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
    TFT_SPI_PORT->CR1 |= SPI_CR1_SPE;
}
