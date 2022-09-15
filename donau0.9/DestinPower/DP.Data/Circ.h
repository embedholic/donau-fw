/*
 * Circ.h
 *
 *  Created on: 2012. 11. 12.
 *      Author: destinPower
 */

#ifndef CIRC_H_
#define CIRC_H_

#include "LGS_Common.h"

typedef struct CIRC_Obj {
    Uns         writeIndex;     /* write pointer for the buffer */
    Uns         readIndex;      /* read pointer fro the buffer */
    Uns         charCount;      /* buffer character count */
    Uns         size;
    char*       buf;      /* circular buffer */
} CIRC_Obj, *CIRC_Handle;

void CIRC_new(CIRC_Handle circ,Char* buf,Uns uiBufSize);
Char CIRC_readChar(CIRC_Handle circ);
Bool CIRC_writeChar(CIRC_Handle circ, Char c);
Bool CIRC_read(CIRC_Handle circ,char* string, unsigned short* psize);
Bool CIRC_write(CIRC_Handle circ, const char* string, unsigned short* psize);

#define CIRC_isEmpty(circ)            ((circ)->charCount==0)
#define CIRC_fullCount(circ)            ((circ)->charCount)
#define CIRC_emptyCount(circ)           ((circ)->size - (circ)->charCount)
#define CIRC_nextIndex(circ,index)      (((index) + 1) & ((circ)->size - 1))
#define CIRC_prevIndex(circ,index)      (((index) - 1) & ((circ)->size - 1))
#define CIRC_reset(circ)            {(circ)->charCount = 0; (circ)->readIndex = (circ)->writeIndex; }
#define CIRC_isFull(circ)           ((circ)->charCount == circ->size)

#ifdef __cplusplus
}
#endif /* extern "C" */


#endif /* CIRC_H_ */
