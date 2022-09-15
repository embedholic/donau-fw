/*
 * Util.h
 *
 *  Created on: 2012. 11. 13.
 *      Author: destinPower
 */

#ifndef UTIL_H_
#define UTIL_H_

#include "LGS_Common.h"
void MmiUtil_Bcd2Asc(char *Bcd, char *Asc, unsigned int AscLen);
void MmiUtil_Bcd2Len(char *Bcd, unsigned int *Len, unsigned int BcdLen);
void MmiUtil_Len2Bcd(unsigned int Len, char *Bcd, char BcdLen);
void MmiUtil_Asc2Bcd(char *AscBuf, char *BcdBuf, unsigned int Len, unsigned int iFieldType);
void MmiUtil_Int2Asc( Int32 iVal, char *Asc, unsigned int* AscLen );

char Uns2Hex( Uns u );
unsigned int Hex2Uns( char c );

int utilCharToInt(char ch);
Bool utilCharBuffer2Int( char *Start, int Length, int *Value );
Void utilTxLenToStr(int TxLen, char *cTxLenStr);
void utilLtoa(int iN, char *pCopyStr, int iStartPos, int *pEndPos, Bool bCommaAdd);

void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);

#endif /* UTIL_H_ */
