/*
 * TEST_delfino.h
 *
 *  Created on: 2012. 11. 1.
 *      Author: destinPower
 */

#ifndef TEST_DELFINO_H_
#define TEST_DELFINO_H_

//#if DBUG_MODE
typedef struct
{
	float Theta;
	float Theta_TR;
	float SinTheta;
	float CosTheta;
    float SinTheta_TR;
    float CosTheta_TR;
	float Vds;
	float Vqs;
    float Vds_TR;
    float Vqs_TR;
    float Va;
    float Vb;
    float Vc;
	float Va_TR;
	float Vb_TR;
	float Vc_TR;
	float Vinv_a;
	float Vinv_b;
	float Vinv_c;
	//float Ids; //+130624@ 인버터 전류도 각을 줌.
	//float Iqs; //+130624@ 인버터 전류도 각을 줌.
	float Iinv_a;
	float Iinv_b;
	float Iinv_c;
	float Vg_a;
	float Vg_b;
	float Vg_c;
	float Vdc;
	float Vdc_batt;
	float Ipv;

	float Vbyp_a;
	float Vbyp_b;
	float Vbyp_c;
	float Ibyp_a;
	float Ibyp_b;
	float Ibyp_c;

	float Itr_a;
	float Itr_b;
	float Itr_c;

	float Iqe;
	float Ide;
} Debug;
//#endif

void TEST_Create();
void TEST_ThreePhaseSine(void);
void TEST_UpdateEVAConvResult(void);
//#if DBUG_MODE
extern Debug DBUG;
//#endif

void TEST_LedToggle(int ledNum);
void TEST_Led(int ledNum, int onoff);

#endif /* TEST_DELFINO_H_ */
