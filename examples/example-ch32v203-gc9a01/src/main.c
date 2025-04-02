#include "debug.h"
#include "tft_espi_shim.h"

int main( void ) {
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_1 );
    SystemCoreClockUpdate( );
    Delay_Init( );
    USART_Printf_Init( 115200 );

    printf( "\x1b[2J\x1b[0;0H" );
    printf( "Built %s at %s.\nCore clock: %ldMHz.\nReady...\n\n", __DATE__, __TIME__, SystemCoreClock / 1000000 );   
    fflush( stdout );

    tftShimInit( );
    tftFillScreen( TFT_GREEN );

    while( 1 ) {
    }
}
