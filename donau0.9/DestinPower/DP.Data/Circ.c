/*
 * Circ.c
 *
 *  Created on: 2012. 11. 12.
 *      Author: destinPower
 */

#include <xdc/std.h>
#include "circ.h"

/*
 *  ======== CIRC_new ========
 *
 *  Initializes the circular buffer structure
 */
Void CIRC_new(CIRC_Handle circ,Char* buf,Uns uiBufSize)
{
    circ->writeIndex = 0;
    circ->readIndex = 0;
    circ->charCount = 0;
    circ->size = uiBufSize;
	circ->buf=buf;
}

/*
 *  ======== CIRC_readChar ========
 *
 *  Reads a character from the circular buffer.
 */
Char CIRC_readChar(CIRC_Handle circ)
{
    Char c;

    if( circ->charCount <= 0)
    {
#if DEBUG_MODE == 1
		System_printf("read buffer underflow");
		System_flush();
#endif

    }
    /* read character and increment the character count */
    c = circ->buf[circ->readIndex];
    circ->readIndex = CIRC_nextIndex(circ,circ->readIndex);
//    ATM_decu(&circ->charCount);
// Disable CPU interrupts
//	DINT;
	(circ->charCount)--;
//	EINT;
    return (c);
}

/*
 *  ======== CIRC_writeChar ========
 *
 *  Writes a character into the circular buffer
 */
Bool CIRC_writeChar(CIRC_Handle circ, Char c)
{
	Bool res=TRUE;
	if(circ->charCount==circ->size)
	{
		CIRC_readChar(circ);
#if DEBUG_MODE == 1
		System_printf("write buffer overflow");
		System_flush();
#endif
		res=FALSE;
	}
    /* write character and decrement the character count */
    circ->buf[circ->writeIndex] = c;
    circ->writeIndex = CIRC_nextIndex(circ,circ->writeIndex);
    //ATM_incu(&circ->charCount);
	(circ->charCount)++;
	return res;
}

Bool CIRC_read(CIRC_Handle circ,char* string, unsigned short* psize)
{
    Bool c=TRUE;
    unsigned short i;

	if (CIRC_fullCount(circ) < *psize){
		*psize=0;
		return FALSE;
	}


    if (CIRC_isEmpty(circ) == 1)
    {
        *psize=0;
        c = FALSE;
    }
    else
    {
        for(i=0;i<(unsigned short)*psize;i++)
        {
            string[i] = circ->buf[circ->readIndex];
            circ->readIndex = CIRC_nextIndex(circ,circ->readIndex);
            (circ->charCount)--;
        }
    }
    return c;
}
Bool CIRC_write(CIRC_Handle circ, const char* string, unsigned short* psize)
{
	Bool res=TRUE;
	unsigned short i;

	if((circ->charCount + (unsigned short)*psize) >=circ->size )
	{
#if DEBUG_MODE == 1
		System_printf("write buffer overflow");
		System_flush();
#endif
		res=FALSE;
	}
    else
    {
        for(i=0;i<(unsigned short)*psize;i++)
        {
            circ->buf[circ->writeIndex] = string[i];
            circ->writeIndex = CIRC_nextIndex(circ,circ->writeIndex);
            (circ->charCount)++;
        }
    }
	return res;
}

