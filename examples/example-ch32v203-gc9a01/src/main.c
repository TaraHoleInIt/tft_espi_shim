#include "debug.h"
#include "tft_espi_shim.h"

extern const unsigned char catImage[ 240 * 108 * 2 ];

void drawCat( void );

int main( void ) {
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_1 );
    SystemCoreClockUpdate( );
    Delay_Init( );
    USART_Printf_Init( 115200 );

    printf( "\x1b[2J\x1b[0;0H" );
    printf( "Built %s at %s.\nCore clock: %ldMHz.\nReady...\n\n", __DATE__, __TIME__, SystemCoreClock / 1000000 );   
    fflush( stdout );

    tftShimInit( );

    while( 1 ) {
        tftFillScreen( TFT_RED );
        drawCat( );
        Delay_Ms( 1000 );

        tftFillScreen( TFT_GREEN );
        drawCat( );
        Delay_Ms( 1000 );

        tftFillScreen( TFT_BLUE );
        drawCat( );
        Delay_Ms( 1000 );
    }
}

void drawCat( void ) {
    tftSetAddressWindow( 0, 63, 239, 171 );

    tftBeginPixels( );
        tftPushPixels( ( const uint16_t* ) catImage, sizeof( catImage ) / 2 );
    tftEndPixels( );
}
