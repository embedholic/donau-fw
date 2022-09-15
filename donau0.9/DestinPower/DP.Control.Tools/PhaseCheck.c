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
* PHS_Check - ����� �̻��� �ִ����� �ľ��Ͽ� �溸�� �˷��ش�
*             A���� +���� -�� �ٲ� �� B�� ������ �׻� C�� ���� ũ��.
*             ����� �ٲ� ���� �� �ݴ����� �̿��Ͽ� ����� �˻��Ѵ�.
* Parameters :
*       PhaseCheck *this
*       Va ==> a�� ����
*       Vb ==> b�� ����
*       Vc ==> c�� ����
*       num ==>   ����̻� ���� ��ȣ
********************************************************************************************************/
//-TODO DATA REGION: #pragma CODE_SECTION(PHS_Check, ".iram")
void PHS_Check( PhaseCheck *this , float Va, float Vb, float Vc, int num )
{
	if ( (this->fVaPrev >= 0)&&(Va < 0) )
	{
		if(Vb<Vc)    // b���� c�󺸴� ū�� ��
			this->iFcnt++;   // Fault ī��Ʈ ����
		else
			this->iFcnt = 0;
	}
	this->fVaPrev = Va;

	if(this->iFcnt >= PHASECHECK_CHECK_NUM) // �����ؼ�  PHASECHECK_CHECK_NUM��(�ֱ�) �߻�
	{
		FLT_Raise( num );
		this->iFcnt = 0;
	}
}



