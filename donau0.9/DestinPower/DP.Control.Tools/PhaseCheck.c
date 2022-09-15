/*
 * PhaseCheck.c
 *
 *  Created on: 2012. 11. 16.
 *      Author: destinPower
 */


#include "phasecheck.h"
#include "fault.h"

#define PHASECHECK_CHECK_NUM	50

PhaseCheck GPC={0,0.};
PhaseCheck IPC={0,0.};

PhaseCheck BYP={0, 0,};

/********************************************************************************************************
* PHS_Check - 상순에 이상이 있는지를 파악하여 경보로 알려준다
*             A상이 +에서 -로 바뀔 때 B상 전압은 항상 C상 보다 크다.
*             상순이 바뀐 경우는 그 반대임을 이용하여 상순을 검사한다.
* Parameters :
*       PhaseCheck *this
*       Va ==> a상 전압
*       Vb ==> b상 전압
*       Vc ==> c상 전압
*       num ==>   상순이상 고장 번호
********************************************************************************************************/
//-TODO DATA REGION: #pragma CODE_SECTION(PHS_Check, ".iram")
void PHS_Check( PhaseCheck *this , float Va, float Vb, float Vc, int num )
{
	if ( (this->fVaPrev >= 0)&&(Va < 0) )
	{
		if(Vb<Vc)    // b상이 c상보다 큰지 비교
			this->iFcnt++;   // Fault 카운트 증가
		else
			this->iFcnt = 0;
	}
	this->fVaPrev = Va;

	if(this->iFcnt >= PHASECHECK_CHECK_NUM) // 연속해서  PHASECHECK_CHECK_NUM번(주기) 발생
	{
		FLT_Raise( num );
		this->iFcnt = 0;
	}
}



