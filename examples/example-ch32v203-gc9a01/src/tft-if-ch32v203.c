#include <stdio.h>
#include "ch32v20x.h"
#include "debug.h"
#include "tft_espi_shim.h"

static void _gpioSetModeHi( GPIO_TypeDef* port, int pinNo, int mode_speed, int initialLevel );
static void _gpioSetModeLo( GPIO_TypeDef* port, int pinNo, int mode_speed, int initialLevel );
static void _gpioEnablePort( GPIO_TypeDef* port );

static uint32_t _spiFreqFromPSC( uint32_t psc );

static void _spiWaitTXE( void );
static void _spiWaitBusy( void );
static void _spiWaitRXNE( void );

static void _spiSet8Bit( void );
static void _spiSet16Bit( void );

static void _waitDMA( DMA_Channel_TypeDef* dma, uint32_t tcFlag );

static void _gpioSetModeHi( GPIO_TypeDef* port, int pinNo, int mode_speed, int initialLevel ) {
    pinNo-= 8;

    port->CFGHR &= ~( 0b1111 << ( pinNo * 4 ) );       // Clear out existing config
    port->CFGHR |= ( mode_speed << ( pinNo * 4 ) );    // Set new config
}

static void _gpioSetModeLo( GPIO_TypeDef* port, int pinNo, int mode_speed, int initialLevel ) {
    port->CFGLR &= ~( 0b1111 << ( pinNo * 4 ) );       // Clear out existing config
    port->CFGLR |= ( mode_speed << ( pinNo * 4 ) );    // Set new config
}

static void _gpioEnablePort( GPIO_TypeDef* port ) {
    if ( port == GPIOA ) RCC->APB2PCENR |= RCC_APB2Periph_GPIOA;
    else if ( port == GPIOB ) RCC->APB2PCENR |= RCC_APB2Periph_GPIOB;
    else if ( port == GPIOC ) RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
    else if ( port == GPIOD ) RCC->APB2PCENR |= RCC_APB2Periph_GPIOD;
    else if ( port == GPIOE ) RCC->APB2PCENR |= RCC_APB2Periph_GPIOE;
}

void gpioSetMode( GPIO_TypeDef* port, int pinNo, int mode_speed, int initialLevel ) {
    mode_speed &= 0x0F;
    initialLevel &= 0x01;

    _gpioEnablePort( port );

    if ( initialLevel ) port->BSHR = ( 1 << pinNo );
    else port->BSHR = ( 1 << ( pinNo + 16 ) );
    
    if ( pinNo >= 8 ) _gpioSetModeHi( port, pinNo, mode_speed, initialLevel );
    else _gpioSetModeLo( port, pinNo, mode_speed, initialLevel );
}

void gpioWrite( GPIO_TypeDef* port, int pinNo, int level ) {
    if ( level ) port->BSHR = ( 1 << pinNo );
    else port->BSHR = ( 1 << ( pinNo + 16 ) );  
}

int gpioRead( GPIO_TypeDef* port, int pinNo ) {
    return ( port->INDR & ( 1 << pinNo ) ) ? 1 : 0;
}

uint32_t _spiFreqFromPSC( uint32_t psc ) {
    uint32_t res = 1;

    psc >>= 3;
    psc++;

    // Hacky way to compute 2^n
    while ( psc-- )
        res *= 2;
    
    return res;
}

void _tftPinSetup( void ) {
    RCC->APB2PCENR |= RCC_APB2Periph_AFIO;
    RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;
    SPI_POWER_PORT |= SPI_POWER_BIT;

    gpioSetMode( CS_PORT, CS_PIN, GPIO_Mode_Out_PP | GPIO_Speed_50MHz, 1 );
    gpioSetMode( DC_PORT, DC_PIN, GPIO_Mode_Out_PP | GPIO_Speed_10MHz, 1 );
    gpioSetMode( RST_PORT, RST_PIN, GPIO_Mode_Out_PP | GPIO_Speed_10MHz, 1 );

    gpioSetMode( SPI_GPIO_PORT, SPI_MOSI, GPIO_Mode_AF_PP | GPIO_Speed_50MHz, 0 );
    gpioSetMode( SPI_GPIO_PORT, SPI_SCLK, GPIO_Mode_AF_PP | GPIO_Speed_50MHz, 0 );
    gpioSetMode( SPI_GPIO_PORT, SPI_MISO, GPIO_Mode_IN_FLOATING, 0 );

    SPI_PORT->CTLR1 = 0;
    SPI_PORT->CTLR2 = 0;

    SPI_PORT->CTLR2 = SPI_CTLR2_RXDMAEN | SPI_CTLR2_TXDMAEN;
    SPI_PORT->CTLR1 = \
        SPI_Mode_Master | \
        SPI_Direction_2Lines_FullDuplex | \
        SPI_DataSize_8b | \
        SPI_CPOL_Low | \
        SPI_CPHA_1Edge | \
        SPI_NSS_Soft | \
        SPI_FirstBit_MSB | \
        SPI_CLOCK_PSC | \
        SPI_CTLR1_SPE
    ;

    printf( "SYSCLK: %luHz\n", SystemCoreClock );
    printf( "SPI Frequency: %luHz\n", SystemCoreClock / _spiFreqFromPSC( SPI_CLOCK_PSC ) );
}

uint8_t _tftBusWrite8( uint8_t data ) {
    _spiWaitTXE( );
    _spiWaitBusy( );

    SPI_PORT->DATAR = data;

    _spiWaitBusy( );
    _spiWaitRXNE( );

    return SPI_PORT->DATAR;
}

uint16_t _tftBusWrite16( uint16_t data ) {
    uint16_t res = 0;

    _spiWaitTXE( );
    _spiWaitBusy( );
    
    _spiSet16Bit( );
        SPI_PORT->DATAR = data;

        _spiWaitBusy( );
        _spiWaitRXNE( );

        res = SPI1->DATAR;
    _spiSet8Bit( );

    return res;
}

void _tftDelayMSec( uint32_t msec ) {
    Delay_Ms( msec );
}

void _tftDCHigh( void ) {
    gpioWrite( DC_PORT, DC_PIN, 1 );
}

void _tftDCLow( void ) {
    gpioWrite( DC_PORT, DC_PIN, 0 );
}

void _tftCSHigh( void ) {
    gpioWrite( CS_PORT, CS_PIN, 1 );
}

void _tftCSLow( void ) {
    gpioWrite( CS_PORT, CS_PIN, 0 );
}

void _tftRSTHigh( void ) {
    gpioWrite( RST_PORT, RST_PIN, 1 );
}

void _tftRSTLow( void ) {
    gpioWrite( RST_PORT, RST_PIN, 0 );
}

static void _spiWaitTXE( void ) {
    while ( ! ( SPI_PORT->STATR & SPI_STATR_TXE ) );
}

static void _spiWaitBusy( void ) {
    while ( SPI_PORT->STATR & SPI_STATR_BSY );
}

static void _spiWaitRXNE( void ) {
    while ( ! ( SPI_PORT->STATR & SPI_STATR_RXNE ) );
}

static void _spiSet8Bit( void ) {
    SPI_PORT->CTLR1 &= ~SPI_CTLR1_SPE;
        SPI_PORT->CTLR1 &= ~SPI_CTLR1_DFF;
    SPI_PORT->CTLR1 |= SPI_CTLR1_SPE;
}

static void _spiSet16Bit( void ) {
    SPI_PORT->CTLR1 &= ~SPI_CTLR1_SPE;
        SPI_PORT->CTLR1 |= SPI_CTLR1_DFF;
    SPI_PORT->CTLR1 |= SPI_CTLR1_SPE;
}

static void _waitDMA( DMA_Channel_TypeDef* dma, uint32_t tcFlag ) {
    if ( dma->CFGR & DMA_CFGR1_EN ) {
        while ( ! DMA_GetFlagStatus( tcFlag ) )
        ;

        DMA1->INTFCR |= tcFlag;
    }
}

void tftPushColor( uint16_t color, int count ) {
    static uint16_t dummy = 0xFFFF;
    int len = 0;

    _waitDMA( SPI_TX_DMA, SPI_TXTC );
    _waitDMA( SPI_RX_DMA, SPI_RXTC );

    _spiWaitTXE( );
    _spiWaitBusy( );

    tftEndPixels( );
    _spiSet16Bit( );
        tftBeginPixels( );
            while ( count ) {
                len = ( count > 65535 ) ? 65535 : count;

                SPI_TX_DMA->CFGR = 0;
                SPI_TX_DMA->CNTR = len;
                SPI_TX_DMA->MADDR = ( uint32_t ) &color;
                SPI_TX_DMA->PADDR = ( uint32_t ) &SPI_PORT->DATAR;

                SPI_RX_DMA->CFGR = 0;
                SPI_RX_DMA->CNTR = len;
                SPI_RX_DMA->MADDR = ( uint32_t ) &dummy;
                SPI_RX_DMA->PADDR = ( uint32_t ) &SPI_PORT->DATAR;

                SPI_RX_DMA->CFGR = 
                    DMA_Priority_VeryHigh | \
                    DMA_MemoryDataSize_HalfWord | \
                    DMA_PeripheralDataSize_HalfWord | \
                    DMA_Mode_Normal | \
                    DMA_DIR_PeripheralSRC | \
                    DMA_M2M_Disable | \
                    DMA_PeripheralInc_Disable | \
                    DMA_MemoryInc_Disable | \
                    DMA_IT_TC | \
                    DMA_CFGR1_EN
                ;

                SPI_TX_DMA->CFGR = 
                    DMA_Priority_VeryHigh | \
                    DMA_MemoryDataSize_HalfWord | \
                    DMA_PeripheralDataSize_HalfWord | \
                    DMA_Mode_Normal | \
                    DMA_DIR_PeripheralDST | \
                    DMA_M2M_Disable | \
                    DMA_PeripheralInc_Disable | \
                    DMA_MemoryInc_Disable | \
                    DMA_IT_TC | \
                    DMA_CFGR1_EN
                ;

                _waitDMA( SPI_RX_DMA, SPI_RXTC );
                _waitDMA( SPI_TX_DMA, SPI_TXTC );

                SPI_TX_DMA->CFGR = 0;
                SPI_RX_DMA->CFGR = 0;

                count-= len;
            }
        tftEndPixels( );
    _spiSet8Bit( );
}

void tftPushPixels( const uint16_t* buf, int lenWords ) {
    static uint16_t dummy = 0xFFFF;
    int len = 0;

    _waitDMA( SPI_TX_DMA, SPI_TXTC );
    _waitDMA( SPI_RX_DMA, SPI_RXTC );

    _spiWaitTXE( );
    _spiWaitBusy( );

    tftEndPixels( );
    _spiSet16Bit( );
        tftBeginPixels( );
            while ( lenWords ) {
                len = ( lenWords > 65535 ) ? 65535 : lenWords;

                SPI_TX_DMA->CFGR = 0;
                SPI_TX_DMA->CNTR = len;
                SPI_TX_DMA->MADDR = ( uint32_t ) buf;
                SPI_TX_DMA->PADDR = ( uint32_t ) &SPI_PORT->DATAR;

                SPI_RX_DMA->CFGR = 0;
                SPI_RX_DMA->CNTR = len;
                SPI_RX_DMA->MADDR = ( uint32_t ) &dummy;
                SPI_RX_DMA->PADDR = ( uint32_t ) &SPI_PORT->DATAR;   
                
                SPI_RX_DMA->CFGR = 
                    DMA_Priority_VeryHigh | \
                    DMA_MemoryDataSize_HalfWord | \
                    DMA_PeripheralDataSize_HalfWord | \
                    DMA_Mode_Normal | \
                    DMA_DIR_PeripheralSRC | \
                    DMA_M2M_Disable | \
                    DMA_PeripheralInc_Disable | \
                    DMA_MemoryInc_Disable | \
                    DMA_IT_TC | \
                    DMA_CFGR1_EN
                ;

                SPI_TX_DMA->CFGR = 
                    DMA_Priority_VeryHigh | \
                    DMA_MemoryDataSize_HalfWord | \
                    DMA_PeripheralDataSize_HalfWord | \
                    DMA_Mode_Normal | \
                    DMA_DIR_PeripheralDST | \
                    DMA_M2M_Disable | \
                    DMA_PeripheralInc_Disable | \
                    DMA_MemoryInc_Enable | \
                    DMA_IT_TC | \
                    DMA_CFGR1_EN
                ;

                _waitDMA( SPI_RX_DMA, SPI_RXTC );
                _waitDMA( SPI_TX_DMA, SPI_TXTC );

                SPI_TX_DMA->CFGR = 0;
                SPI_RX_DMA->CFGR = 0;

                lenWords-= len;
                buf+= len;
            }
        tftEndPixels( );
    _spiSet8Bit( );
}
