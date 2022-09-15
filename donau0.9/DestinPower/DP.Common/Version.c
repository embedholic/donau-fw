/*
 * Version.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */


#include <string.h>
#include "Version.h"
#include "Parameter.h"
#include "NvSRAM.h"
#include "System.h"

static char sVersion[VER_MAXLEN] = {0};

void VER_create(void)
{
	//VER_VERSION "x.xx - .  - .   "
	Uns Pos;
	Uns ParaVer;

	memset(sVersion, ' ', VER_MAXLEN);

	/*
	 * copy XCU67 version
	 */
	Pos = 0;
	strncpy(sVersion, VER_VERSION, strlen(VER_VERSION));
	Pos += strlen(VER_VERSION);	// This counts '\n' too

	/*
	 * mark compile mode
	 */
	#if DBUG_MODE==1
	sVersion[Pos++]   = 'D';
	sVersion[Pos++]   = '1';
	#elif DBUG_MODE==2
	sVersion[Pos++]   = 'D';
	sVersion[Pos++]   = '2';
	#elif DBUG_MODE==3
	sVersion[Pos++]   = 'D';
	sVersion[Pos++]   = '3';
	#else
	sVersion[Pos++]   = 'R';
	#endif


	/*
	 * copy version suffix if exsist
	 */
	sVersion[Pos++] = '/';	// make dash


	/*
	 * Parameter version
	 */
	ParaVer = PARA_getParaVersion();

	sVersion[Pos++] = '0'+((ParaVer & 0xF000) >> 12);
	sVersion[Pos++] = '0'+((ParaVer & 0x0F00) >> 8);
	sVersion[Pos++] = '.';
	sVersion[Pos++] = '0'+((ParaVer & 0x00F0) >> 4);
	sVersion[Pos++] = '0'+( ParaVer & 0x000F);




	sVersion[Pos] = NULL; //make null
}

/*******************************************************************/
/*FUNCTION:													       */
/*PARAMETERS:													   */
/*RETURN:		                                                   */
/*NOTE:				                                               */
/*******************************************************************/

String VER_getString()
{
	return sVersion;
}



