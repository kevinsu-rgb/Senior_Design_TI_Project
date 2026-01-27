

#include <stdlib.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

void testSensor();

int main(void)
{
    System_init();
    Board_init();

    Drivers_open(); 
    Board_driversOpen();

    testSensor();

    Board_deinit();
    System_deinit();

    return 0;
}
