/*
 * Util.c
 *
 *  Created on: 2012. 11. 13.
 *      Author: destinPower
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "util.h"

#define ISO8583type_Cnst			0x00 /* type Const                  */
#define ISO8583type_LEFT_JUSTIFY	0x04
#define ISO8583type_Var				0x08 /* type Variable - 99/999      */
#define ISO8583type_BIN				0x10 /* type Binary - 'b','h'       */
#define ISO8583type_ASC				0x20 /* type ASCII  - 'a','an','ans'*/
#define ISO8583type_BCD				0x40 /* type BCD    - 'n','z'       */
#define ISO8583BufLen				300 /* number of bytes for a message */

#define BITFIELD_ORDER_2			0x40
#define BITFIELD_ORDER_14			0x04
#define BITFIELD_ORDER_35			0x20

#define	STX_CHAR					0xEE

/* --------------------------------------------------------------------------
 * FUNCTION NAME: MmiUtil_Int2Asc
 */
void MmiUtil_Int2Asc( Int32 iSrcVal,/*OUT*/ char *cDestAsc,/*OUT*/ unsigned int* AscLen )
{
	unsigned long	lValue;
	unsigned int	pos=0,i=0;
	char			temp;

	if(iSrcVal<0)
	{
		lValue=-iSrcVal;
		cDestAsc[pos++]='-';
	}
	else
		lValue=iSrcVal;

	do
	{
		cDestAsc[pos++]='0'+lValue%10;
		lValue=lValue/10;
	}while(lValue>0);

	*AscLen=pos;

	if(iSrcVal<0)
	{
		for(i=1;i<(pos+1)/2;i++)
		{
			temp=cDestAsc[i];
			cDestAsc[i]=cDestAsc[pos-i];
			cDestAsc[pos-i]=temp;;
		}
	}
	else
	{
		for(i=0;i<pos/2;i++)
		{
			temp=cDestAsc[i];
			cDestAsc[i]=cDestAsc[pos-1-i];
			cDestAsc[pos-1-i]=temp;;
		}
	}
}


/* --------------------------------------------------------------------------
 * FUNCTION NAME: MmiUtil_BCD2ASC
 */
void MmiUtil_Bcd2Asc( char *Bcd, char *Asc, unsigned int AscLen )
{
	int	i;

	for(i = 0; i < AscLen; i++ )
	{
		if(i%2==0)
			Asc[i] = ((Bcd[i/2]&0xF0)>>4)+48;
		else
			Asc[i] = (Bcd[i/2]&0x0F)+48;
	}
}

/* --------------------------------------------------------------------------
 * FUNCTION NAME: MmiUtil_Bcd2Len
 * DESCRIPTION:   Convert BcdLen chars BCD length to unsigned int.
 * PARAMETERS:    Bcd - source data,
 *                Len - destination data,
 *                BcdLen - Bcd chars(1 or 2)
 * RETURN:        T - success / F - failed.
 * ------------------------------------------------------------------------ */
void MmiUtil_Bcd2Len( char *Bcd, unsigned int *Len, unsigned int BcdLen)
{
	unsigned int	i;

    *Len = 0;
	for(i = 0; i < BcdLen; i++)
        *Len = *Len * 100 + (((*(Bcd+i))&0xF0) >> 4) * 10 + ((*(Bcd+i))&0xF);

}

/* --------------------------------------------------------------------------
 * FUNCTION NAME: MmiUtil_Len2Bcd
 * DESCRIPTION:   Convert unsigned int to n chars BCD
 * PARAMETERS:    Len - source data
 *                Bcd - destination data
 *                BcdLen - Length of BCD code
 * RETURN:        T - success / F - failed.
 * June : Fixed for 28x MCU
 * ------------------------------------------------------------------------ */
void MmiUtil_Len2Bcd(unsigned int Len, char *Bcd, char BcdLen )
{
	char	str[30];
	 int i=0;
	 int numlen=0;
	 int divider = 1;

	// 자릿수 구하기.
	i = Len;
	while(1)
	{
		if(i!=0)
 		{
			numlen++;
			i=i/10;
		}
		else
			break;
	}

	for(i=0; i<numlen-1; i++)
		divider *= 10;

	for(i=0;i<BcdLen*2;i++)
	{
		// 자릿수가 남을 경우 좌측 공백.
		if( i < (BcdLen*2)-numlen )
		{
			str[i] = '0';
			continue;
		}

		str[i] = (Len / divider) + '0';
		Len = Len % divider;
		divider /= 10;

	}
	str[i++] = 0;

    MmiUtil_Asc2Bcd((char*)str, Bcd, BcdLen*2, ISO8583type_BCD);
}


/* --------------------------------------------------------------------------
 * FUNCTION NAME: MmiUtil_Asc2Bcd.
 * DESCRIPTION:   Convert ASCII code to BCD code.
 * PARAMETERS:    AscBuf - Ascii input buffer, must ended by '\0'
 *                BcdBuf - converted result
 *                Len - double length of BCD code, should be even.
 * RETURN:        T - success / F - failed(illegal char in ASCII buffer).
 * NOTES:         support 'A'-'F' convertion.
 * ------------------------------------------------------------------------ */
void MmiUtil_Asc2Bcd(char *AscBuf, char *BcdBuf, unsigned int Len, unsigned int iFieldType)
{
    unsigned int    i;
	char	str[2],cBuf[100];

    if(Len % 2 != 0)
    {
        Len++;

		if(iFieldType & ISO8583type_LEFT_JUSTIFY)
		{
			strncpy((char *)cBuf, (char *)AscBuf, Len-1);
			cBuf[Len-1] = 'F';
		}
		else
		{
		    cBuf[0] = '0';
		    strncpy((char *)cBuf+1, (char *)AscBuf, Len-1);
		}
    }
    else
	{
        strncpy((char *)cBuf, (char *)AscBuf, Len);
	}

	memset( str, 0, sizeof( str ) );

    for(i = 0; i < Len; i += 2)
    {
		if( ( cBuf[i] >= 'a' ) && ( cBuf[i] <= 'f' ) )
			str[0] = cBuf[i] - 'a' + 0x0A;
		else if( ( cBuf[i] >= 'A' ) && ( cBuf[i] <= 'F' ) )
			str[0] = cBuf[i] - 'A' + 0x0A;
		else if( cBuf[i] >= '0' )
			str[0] = cBuf[i] - '0';
		else
			str[0] = 0;

		if( ( cBuf[i+1] >= 'a' ) && ( cBuf[i+1] <= 'f' ) )
			str[1] = cBuf[i+1] - 'a' + 0x0A;
		else if( ( cBuf[i+1] >= 'A' ) && ( cBuf[i+1] <= 'F' ) )
			str[1] = cBuf[i+1] - 'A' + 0x0A;
		else if( cBuf[1] >= '0' )
			str[1] = cBuf[i+1] - '0';
		else
			str[1] = 0;

		BcdBuf[i/2] = (str[0]<<4) | (str[1]&0x0F);
	}
}

/*******************************************************************/
/* Uns2Hex - Uns를 Hex로 변환                                      */
/* Parameters : Uns u => 변환 대상                                 */
/* Returns : char => 변환한 값                                     */
/*******************************************************************/
char Uns2Hex(Uns u)
{
	if ( u < 10 ) return ( u + '0' );
	else return ( u - 10 + 'A' );
}

unsigned int Hex2Uns( char c )
{
	unsigned int temp;

	if ( c >= 'A' ) temp = c - 'A' + 10;
	else temp = c - '0';

	return temp;
}

/*******************************************************************/
/*FUNCTION:													       */
/*PARAMETERS:													   */
/*RETURN:		                                                   */
/*NOTE:				                                               */
/*******************************************************************/
int utilCharToInt(char ch)
{
	if ( ch < '0' || ch > '9' ) return -1;
	else return (ch - '0');
}
/*******************************************************************/
/*FUNCTION:													       */
/*PARAMETERS:													   */
/*RETURN:		                                                   */
/*NOTE:				                                               */
/*******************************************************************/
void utilLtoa(int iN, char *pCopyStr, int iStartPos, int *pEndPos, Bool bCommaAdd)
{
	int iByteNum;
	char cTxDataStr[10];
	int iLast, i;

	iByteNum = ltoa(iN, cTxDataStr,16); // by isjeon
	for ( i = 0; i < iByteNum; i++ )
	{
		pCopyStr[iStartPos+i] = cTxDataStr[i];
	}
	iLast = iStartPos + i;

	if ( bCommaAdd == TRUE )
	{
		pCopyStr[iLast] = ',';
		*pEndPos = iLast+1;
	}
	else
	{
		pCopyStr[iLast] = 0;
		*pEndPos = iLast;
	}
}
/*******************************************************************/
/*FUNCTION:													       */
/*PARAMETERS:													   */
/*RETURN:		                                                   */
/*NOTE:				                                               */
/*******************************************************************/
Bool utilCharBuffer2Int( char *Start, int Length, int *Value )
{
	int iTemp, iVal = {0};
	Bool bResult;

	bResult = TRUE;

	switch ( Length )
	{
		case 1:
			iTemp = utilCharToInt( *Start );
			if ( iTemp == -1 )
				bResult = FALSE;
			else
				iVal = iTemp;
			break;
		case 2:
			iTemp = utilCharToInt( *Start );
			if ( iTemp == -1 )
				bResult = FALSE; // -1이면 숫자아닌 다른 문자임
			else
				iVal = iTemp * 10;
			iTemp = utilCharToInt( *(Start+1) );
			if ( iTemp == -1 )
				bResult = FALSE; // -1이면 숫자아닌 다른 문자임
			else
				iVal += iTemp;
			break;
		case 3:
			iTemp = utilCharToInt( *Start );
			if ( iTemp == -1 )
				bResult = FALSE; // -1이면 숫자아닌 다른 문자임
			else
				iVal = iTemp*100;
			iTemp = utilCharToInt( *(Start+1) );
			if ( iTemp == -1 )
				bResult = FALSE; // -1이면 숫자아닌 摸?문자임
			else
				iVal += iTemp*10;
			iTemp = utilCharToInt( *(Start+2) );
			if ( iTemp == -1 )
				bResult = FALSE; // -1이면 숫자아닌 다른 문자임
			else
				iVal += iTemp;
			break;
		default:
			bResult = FALSE;
	}

		if ( bResult )
		{
			*Value = iVal;
			return TRUE;
		}
		else
			return FALSE;

}
/*******************************************************************/
/*FUNCTION:													       */
/*PARAMETERS:													   */
/*RETURN:		                                                   */
/*NOTE:				                                               */
/*******************************************************************/
Void utilTxLenToStr(int TxLen, char *cTxLenStr)
{
	if ( TxLen < 10 )
	{
		cTxLenStr[0] = '0';
		cTxLenStr[1] = '0';
		cTxLenStr[2] = '0'+TxLen;
	}
	else if ( TxLen < 100 )
	{
		cTxLenStr[0] = '0';
		cTxLenStr[1] = '0' + (TxLen/10);
		cTxLenStr[2] = '0' + (TxLen%10);
	}
	else if ( TxLen < 1000 )
	{
		cTxLenStr[0] = '0' + (TxLen/100);
		cTxLenStr[1] = '0' + ((TxLen%100)/10);
		cTxLenStr[2] = '0' + ((TxLen%100)%10);
	}
	else
	{
		cTxLenStr[0] = '0';
		cTxLenStr[1] = '0';
		cTxLenStr[2] = '0';
	}
}

void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    {
       *DestAddr++ = *SourceAddr++;
    }
    return;
}



