#include <stdio.h>
#include <stdlib.h>
#include "ch32fun.h"
#include "ch32v003_GPIO_branchless.h"
#include "ch32v003_SPI.h"
#include "tft_espi_shim.h"

#define SPI_TX_DMA DMA1_Channel3
#define DMA_MAX_LEN 65535

void DMA1_Channel3_IRQHandler( void ) __attribute__( ( interrupt ) );

int spiSetFreq( int newFreq );
uint8_t* spiRW( const uint8_t* src, uint8_t* dest, int len );
uint8_t spiWrite8( const uint8_t data );
uint16_t spiWrite16( const uint16_t data );
void spiInit( void );
void spiWaitBusy( void );
void spiWaitRX( void );

static volatile uint8_t isTX = 0;

void _tftPinSetOutput( int pin ) {
    GPIO_pinMode( pin, GPIO_pinMode_O_pushPull, GPIO_Speed_50MHz );
}

void _tftPinSetInput( int pin ) {
    GPIO_pinMode( pin, GPIO_pinMode_I_pullUp, GPIO_Speed_50MHz );
}

int _tftPinRead( int pin ) {
    return GPIO_digitalRead( pin );
}

void _tftPinWrite( int pin, int level ) {
   if ( level ) GPIO_digitalWrite_hi( pin );
   else GPIO_digitalWrite_lo( pin );
}

uint8_t _tftBusWrite8( uint8_t data ) {
    return spiWrite8( data );
}

uint16_t _tftBusWrite16( uint16_t data ) {
    return spiWrite16( data );
}

void _tftDelayMSec( uint32_t msec ) {
    Delay_Ms( msec );
}

void _tftBusInit( uint32_t freq ) {
    spiInit( );
}

// Base freq:   48000000 Hz  48   MHz
// SPI DIV 2:   24000000 Hz  24   MHz 0
// SPI DIV 4:   12000000 Hz  12   MHz 1 
// SPI DIV 8:    6000000 Hz   6   MHz 2
// SPI DIV 16:   3000000 Hz   3   MHz 3
// SPI DIV 32:   1500000 Hz   1.5 MHz 4 
// SPI DIV 64:    750000 Hz 750   KHz 5
// SPI DIV 128:   375000 Hz 375   KHz 6
// SPI DIV 256:   187500 Hz 187.5 KHz 7

int spiSetFreq( int newFreq ) {
    static const uint8_t prescalers[ 8 ] = {
        SPI_BaudRatePrescaler_2,
        SPI_BaudRatePrescaler_4,
        SPI_BaudRatePrescaler_8,
        SPI_BaudRatePrescaler_16,
        SPI_BaudRatePrescaler_32,
        SPI_BaudRatePrescaler_64,
        SPI_BaudRatePrescaler_128,
        SPI_BaudRatePrescaler_256
    };
    int freqActual = 0;
    int freqClosest = 0;
    int freqDiff = 0;
    int smallestDiff = 0;
    int pscShift = 0;
    int pscIdx = 0;
    int i = 0;
    int wasEnabled = 0;

    wasEnabled = ( SPI1->CTLR1 & SPI_CTLR1_SPE );
    freqClosest = FUNCONF_SYSTEM_CORE_CLOCK >> 1;
    freqDiff = freqClosest;
    smallestDiff = freqDiff;

    for ( i = 0; i < 8; i++ ) {
        pscShift = i + 1;

        freqActual = ( FUNCONF_SYSTEM_CORE_CLOCK ) >> pscShift;
        freqDiff = abs( freqActual - newFreq );

        if ( freqDiff < smallestDiff ) {
            smallestDiff = freqDiff;
            freqClosest = freqActual;
            pscIdx = i;
        }
    }

    // Clamp the prescaler index to be extra sure...
    pscIdx &= 0x07;

    SPI1->CTLR1 &= ~SPI_CTLR1_SPE;      // Disable SPI
        SPI1->CTLR1 &= ~SPI_CTLR1_BR;   // Clear baud rate bits
        SPI1->CTLR1 |= prescalers[ pscIdx ];
    SPI1->CTLR1 |= wasEnabled;          // Re-enable SPI (if enabled before)

    return freqClosest;
}

uint8_t* spiRW( const uint8_t* src, uint8_t* dest, int len ) {
    uint8_t* result = dest;
    uint8_t tx = 0;
    uint8_t rx = 0;

    while ( len-- ) {
        tx = ( src ) ? *src++ : 0;
        rx = spiWrite8( tx );

        if ( dest ) {
            *dest++ = rx;
        }
    }

    return result;
}

uint8_t spiWrite8( const uint8_t data ) {
    spiWaitBusy( );
        SPI1->DATAR = data;
    spiWaitRX( );
    
    return SPI1->DATAR;
}

uint16_t spiWrite16( const uint16_t data ) {
    uint16_t res = 0;

    res |= ( spiWrite8( ( data >> 8 ) & 0xFF ) << 8 );
    res |= ( spiWrite8( data & 0xFF ) );

    return res;
}

void spiInit( void ) {
    RCC->APB2PCENR |= RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO;
    RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

    GPIO_pinMode( TFT_MOSI, GPIO_pinMode_O_pushPullMux, GPIO_Speed_50MHz );
    GPIO_pinMode( TFT_SCLK, GPIO_pinMode_O_pushPullMux, GPIO_Speed_50MHz );
    GPIO_pinMode( TFT_MISO, GPIO_pinMode_I_pullUp, GPIO_Speed_50MHz );

    SPI1->CTLR1 = \
        SPI_Mode_Master | \
        SPI_Direction_2Lines_FullDuplex | \
        SPI_DataSize_8b | \
        SPI_CPOL_Low | \
        SPI_CPHA_1Edge | \
        SPI_NSS_Soft | \
        SPI_FirstBit_MSB
    ;
    SPI1->CTLR2 = SPI_CTLR2_TXDMAEN;

    spiSetFreq( SPI_FREQUENCY );
    SPI1->CTLR1 |= SPI_CTLR1_SPE;
}

void spiWaitBusy( void ) {
    while ( ( SPI1->STATR & SPI_STATR_BSY ) || isTX )
    ;  
}

void spiWaitRX( void ) {
    while ( ! ( SPI1->STATR & SPI_STATR_RXNE ) )
    ;
}

// 240x240x2 = 115200

void tftPushColor( uint16_t color, int count ) {
    int len = 0;

    spiWaitBusy( );

    SPI1->CTLR1 &= ~SPI_CTLR1_SPE;      // Disable SPI
        SPI1->CTLR1 |= SPI_CTLR1_DFF;   // Set 16-bit mode
    SPI1->CTLR1 |= SPI_CTLR1_SPE;       // Re-enable SPI

    tftBeginPixels( );
        while ( count ) {
            len = ( count > DMA_MAX_LEN ) ? DMA_MAX_LEN : count;

            SPI_TX_DMA->CFGR &= ~DMA_CFGR3_EN;
            SPI_TX_DMA->MADDR = ( uint32_t ) &color;
            SPI_TX_DMA->PADDR = ( uint32_t ) &SPI1->DATAR;
            SPI_TX_DMA->CNTR = len;
            SPI_TX_DMA->CFGR = \
                DMA_Priority_VeryHigh | \
                DMA_Mode_Normal | \
                DMA_M2M_Disable | \
                DMA_MemoryInc_Disable | \
                DMA_PeripheralInc_Disable | \
                DMA_PeripheralDataSize_HalfWord | \
                DMA_MemoryDataSize_HalfWord | \
                DMA_DIR_PeripheralDST | 
                DMA_IT_TC | \
                DMA_CFGR1_EN
            ;

            spiWaitBusy( );
            count-= len;
        }
    tftEndPixels( );

    SPI1->CTLR1 &= ~SPI_CTLR1_SPE;      // Disable SPI
        SPI1->CTLR1 &= ~SPI_CTLR1_DFF;  // Clear 16-bit mode
    SPI1->CTLR1 |= SPI_CTLR1_SPE;       // Re-enable SPI
}

// ref: https://github.com/cnlohr/ch32v003fun/blob/master/examples/dma_spi/dma_spi.c
void DMA1_Channel3_IRQHandler( void ) {
    volatile int intfr = DMA1->INTFR;

    do {
        DMA1->INTFCR = 0xFFFFFFFF;

        if ( intfr & DMA1_IT_TC3 )
            isTX = 0;

        intfr = DMA1->INTFR;
    } while ( intfr );
}
