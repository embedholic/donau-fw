/*
 * MathConst.h
 *
 *  Created on: 2012. 11. 8.
 *      Author: destinPower
 */

#ifndef MATHCONST_H_
#define MATHCONST_H_

#define PWM_CLK		150	/* 150 MHz */

/* Inverse Factorial */
#define     f2      		(1./2.)
#define		f3				(f2/3.)
#define		f4				(f3/4.)
#define		f5				(f4/5.)
#define		f6				(f5/6.)
#define		f7				(f6/7.)
#define		f8				(f7/8.)
#define		f9				(f8/9.)
#define		f10				(f9/10.)
#define		f11				(f10/11.)
#define		f12				(f11/12.)
#define		f13				(f12/13.)
#define		f14				(f13/14.)
#define		f15				(f14/15.)

/* PI */
#ifndef PI
#define		PI				3.1415926536
#endif
/* 2 * PI */
#define		TWO_PI			6.2831853072
/* 1 / (2*PI) */
#define		INV_TWO_PI		0.15915494309189533
/* PI/6, 30도 */
#define		PI_6			0.5235987756

/* SQRT */
#define		SQRT2		1.4142135624
#define		SQRT3		1.7320508076

// 1/SQRT2
#define		INV_SQRT2	(0.7071067812)

// 1/SQRT3
#define		INV_SQRT3	(0.5773502692)

// 1/3
#define		INV_3   	(0.3333333333)

// 2/15
#define		TAN3rd		(0.1333333333)

// 90/0.7
//#define		APS_LINE_COEF2 (128.57142857) // degree
#define		APS_LINE_COEF2 (2.243994752)

//#define		APS_LINE_COEF1 (3.4336) // degree. 6%
//#define		APS_LINE_COEF1 0.059927625196
#define 	APS_LINE_COEF1 (0.0996687) /* degree 10% = 5.7106 */

#define 	DEGREE_90_RADIAN 1.57079633


// SQRT2/3
#define		SQRT2_3		0.81649658093

/* etc */
/*#define		OP_BR		0x60000000*/
#define 	SIN_60 		0.866025404

#define 	COS_30 		0.866025404
#define 	SIN_30 		0.5

#define		DEG_2_RAD	0.0174532925199
#define		RAD_2_DEG	57.295779513082
#define		TR_RATIO_320_208	1.5384615384615384615384615384615
#define     TR_RATIO_320_220    1.4545454545454545454545454545455
#define     TR_RATIO_320_208_INVERSE    0.65
#define		TR_RATIO_320_220_INVERSE	0.68749999999999999999999999999998
#define     TR_RATIO_320_480    0.6666666666666666666
#define     TR_RATIO_320_480_INVERSE    1.5
#define		TR_RATIO_310_400	0.775	 /* 310/400 */
#define		TR_RATIO_290_400	0.725	 /* 290/400 */
#define		TR_RATIO_190_380	0.5		 /* 190/380 */
#define 	TR_RATIO_19_380		0.05	 /* 19/380 */

#define		TR_RATIO_380_400	0.95
#define		TR_RATIO_380_400_INVERSE 1.052631

#define		TR_RATIO_400_380	1.05263
#define		TR_RATIO_400_380_INVERSE 0.95

#define		TR_RATIO_350_380	0.92105263
#define		TR_RATIO_350_380_INVERSE 1.085714
#define		TR_RATIO_370_380	0.97368421
#define		TR_RATIO_370_380_INVERSE 1.027027
#define 	TR_RATIO_350_350 1.
#define		TR_RATIO_350_350_INVERSE 1.
#define 	TR_RATIO_380_380 1.
#define		TR_RATIO_380_380_INVERSE 1.
#define		TR_RATIO_350_440		 0.7954
#define		TR_RATIO_350_440_INVERSE 1.25714

#define		TR_RATIO_400_208		 1.923077
#define		TR_RATIO_400_208_INVERSE 0.52


#define 	COSPHI_Qcf_75kW 		(28.072450682582880)	//sqrt(6)* 2*pi*freq*40uF*2*380	??
#define 	COSPHI_Qcf_100kW 		(22.9012097673702)		//sqrt(6)* 2*pi*freq*40uF*2*310
#define 	COSPHI_Qcf_200kW 		(53.559280907559440)	//sqrt(6)* 2*pi*freq*40uF*5*290
#define 	COSPHI_Qcf_250kW 		 0 // 더해줄 필요 없어서, 0으로 둠. 13.10.29 (27.33370198049845)		//sqrt(6)* 2*pi*freq*40uF*2*370
#define 	COSPHI_Qcf_350kW 		(53.559280907559440)	//sqrt(6)* 2*pi*freq*40uF*5*290	??

#define		COSPHI_EX_RP_100kW	(-0.07)
#define		COSPHI_EX_RP_200kW	(-0.045)
#define		COSPHI_EX_RP_250kW	(-0.08)

#define 	COSPHI_MAX_APP_POWER 1.05263157//894737 // 1/0.95
//#define		COSPHI_EXPERIMENTAL_RP	(-0.07)

#define 	COSPHI_Q_PU_MAX		0.999//0.7
#define		COSPHI_PF_MAX		(1.0)
#define 	COSPHI_PF_MIN		(0.8)
#define		COSPHI_RATED_PF_MIN	(0.95)
#define		COSPHI_RATED_Q_PU_MAX	(0.31)

/* Macro functions */

#define TAN(x)( (x) + (INV_3*(x)*(x)*(x)) + (TAN3rd*(x)*(x)*(x)*(x)*(x)) )

#define BOUND_PI(x) ((x) + (((x)> PI) ? (-2.*PI) : ((x)< (-PI)) ? (2.*PI) : 0.))
#define SINm(x,x2)   ((x)*(1.-(x2)*(f3-(x2)*(f5-(x2)*(f7-(x2)*(f9-(x2)*(f11-(x2)*(f13-(x2)*f15))))))))
#define COSm(x2)     (1.-(x2)*(f2-(x2)*(f4-(x2)*(f6-(x2)*(f8-(x2)*(f10-(x2)*(f12-(x2)*f14)))))))
#define SIN_INV_X(x2)   ((1.-(x2)*(f3-(x2)*(f5-(x2)*(f7-(x2)*(f9-(x2)*(f11-(x2)*(f13-(x2)*f15))))))))
#define EXP(x)	    (1.+(x)*(1.+(x)*(f2+(x)*(f3+(x)*(f4+(x)*(f5+(x)*(f6+(x)*f7)))))))

// 3상 좌표계 ==> d-q 정지좌표계, 영상분이 0인 경우
// 인자 설명
//   Input
//     a ==> Phase a
//     b ==> Phase b
//     c ==> Phase c
//   Output
//     ds ==> 정지좌표계 d축
//     qs ==> 정지좌표계 q축
#define TRANSFORM_abc_dq(a, b, c, ds, qs) \
	ds = (a); \
	qs = ((b)-(c)) * INV_SQRT3;

// 3상 좌표계 ==> d-q 정지좌표계, 영상분이 0이 아닌 경우
// 인자 설명
//   Input
//     a ==> Phase a
//     b ==> Phase b
//     c ==> Phase c
//   Output
//     ds ==> 정지좌표계 d축
//     qs ==> 정지좌표계 q축
//     ns ==> 정지좌표계 n축
#define TRANSFORM_abc_dqn(a, b, c, ds, qs, ns) \
	ds = (2*(a) - (b) - (c)) * INV_3; \
	qs = ((b)-(c)) * INV_SQRT3; \
	ns = ((a) + (b) + (c)) * SQRT2 * INV_3;

// d-q 정지좌표계 ==> 3상 좌표계, 영상분이 0인 경우
// 인자 설명
//   Input
//     ds ==> 정지좌표계 d축
//     qs ==> 정지좌표계 q축
//   Output
//     a ==> Phase a
//     b ==> Phase b
//     c ==> Phase c
#define TRANSFORM_dq_abc(ds, qs, a, b, c) \
	a = (ds); \
	b = -0.5 * ( (ds) - ( SQRT3 * (qs) ) ); \
	c = - (a + b);

// d-q 정지좌표계 ==> 3상 좌표계, 영상분이 0이 아닌 경우
// 인자 설명
//   Input
//     ds ==> 정지좌표계 d축
//     qs ==> 정지좌표계 q축
//     ns ==> 정지좌표계 n축
//   Output
//     a ==> Phase a
//     b ==> Phase b
//     c ==> Phase c
#define TRANSFORM_dqn_abc(ds, qs, ns, a, b, c) \
	a = (ds) + ( INV_SQRT2 * (ns) ); \
	b = -0.5 * ( (ds) - ( SQRT3 * (qs) ) - ( SQRT2 * (ns) ) ); \
	c = -0.5 * ( (ds) + ( SQRT3 * (qs) ) - ( SQRT2 * (ns) ) );

// d-q 정지 좌표계 ==> d-q 동기 좌표계
// 인자 설명
//   Input
//     ds ==> 정지좌표계 d축
//     qs ==> 정지좌표계 q축
//     CosTheta ==>
//     SinTheta ==>
//   Output
//     de ==> 동기좌표계 d축
//     qe ==> 동기좌표계 q축
#define TRANSFORM_ROTATE_THETA(ds, qs, CosTheta, SinTheta, de, qe) \
	de = (CosTheta) * (ds) + (SinTheta) * (qs); \
	qe = -(SinTheta) * (ds) + (CosTheta) * (qs);

// d-q 정지 좌표계 ==> d-q 동기 좌표계
// 인자 설명
//   Input
//     ds ==> 정지좌표계 d축
//     qs ==> 정지좌표계 q축
//     ns ==> 정지좌표계 n축
//     CosTheta ==>
//     SinTheta ==>
//   Output
//     de ==> 동기좌표계 d축
//     qe ==> 동기좌표계 q축
//     ne ==> 동기좌표계 n축
#define TRANSFORM_ROTATE_THETA_n(ds, qs, ns, CosTheta, SinTheta, de, qe, ne) \
	de = (CosTheta) * (ds) + (SinTheta) * (qs); \
	qe = -(SinTheta) * (ds) + (CosTheta) * (qs); \
	ne = (ns);

// d-q 동기 좌표계 ==> d-q 정지 좌표계
// 인자 설명
//   Input
//     de ==> 동기좌표계 d축
//     qe ==> 동기좌표계 q축
//     CosTheta ==>
//     SinTheta ==>
//   Output
//     ds ==> 정지좌표계 d축
//     qs ==> 정지좌표계 q축
#define TRANSFORM_ROTATE_THETA_INVERSE(de, qe, CosTheta, SinTheta, ds, qs) \
	ds = (CosTheta) * (de) - (SinTheta) * (qe); \
	qs = (SinTheta) * (de) + (CosTheta) * (qe);

// d-q 동기 좌표계 ==> d-q 정지 좌표계
// 인자 설명
//   Input
//     de ==> 동기좌표계 d축
//     qe ==> 동기좌표계 q축
//     ne ==> 동기좌표계 n축
//     CosTheta ==>
//     SinTheta ==>
//   Output
//     ds ==> 정지좌표계 d축
//     qs ==> 정지좌표계 q축
//     ns ==> 정지좌표계 n축
#define TRANSFORM_ROTATE_THETA_INVERSE_n(de, qe, ne, CosTheta, SinTheta, ds, qs, ns) \
	ds = (CosTheta) * (de) - (SinTheta) * (qe); \
	qs = (SinTheta) * (de) + (CosTheta) * (qe); \
	ns = (ne);

#define		RUN_sub		0x555550fa
#define		STOP_sub	0xaaaaa0f4



#endif /* MATHCONST_H_ */
