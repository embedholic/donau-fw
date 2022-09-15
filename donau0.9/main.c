/*
 *  ======== main.c ========
 */

#include <xdc/std.h>

#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>

#include "MCU_BSP_Init.h"
#include "MCU_SW_Init.h"
#include "FastRms.h"
/*
 *  ======== main ========
 */
void main()
{
    /*
	 * Create BSP Software
	 */
   MBI_Create();
#if DEBUG_MODE == 1
    System_printf("MBI_Created\n");
    System_flush();
#endif

	/*
	 * Create Application Software
	 */
	MSI_Create();
#if DEBUG_MODE == 1
	System_printf("MSI_Created\n");
	System_flush();
#endif

    /*
     * use ROV->SysMin to view the characters in the circular buffer
     *
     * Start BIOS Operation, User Task and Periodic functions.
     */
    BIOS_start();        /* enable interrupts and start SYS/BIOS */
}
