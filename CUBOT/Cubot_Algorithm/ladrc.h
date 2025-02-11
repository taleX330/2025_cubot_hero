#ifndef _LADRC_H
#define _LADRC_H
#include "stdint.h"
#include "stm32h7xx_hal.h"

#define square(a) (a)*(a)
#define cube(a) (a)*(a)*(a)
/**
   *@Brief  ����ΪLADRCϵͳ����
   *@WangShun  2022-07-03  ע��
   */
typedef struct
{
	double v1;            //�������ֵ
	double v2;            //�������ֵ
	double h;             //���ֲ���
	double r;             //�ٶ�����
	double kp;
	double kd;
	double b0;            //ϵͳ����
	double beta;
	double z1;            //�۲������
	double z2;			 //�۲������
	double z3;		 	 //�۲������
	double u;             //���������
	double u0;
}Ladrc_t;

extern Ladrc_t heroLadrc;
/**
   *@Brief  ����ΪLADRC��غ���
   *@WangShun  2022-07-03  ע��
   */
double LADRC_Loop(double Expect, double Measure/*����*/, double DevMeasure ,Ladrc_t *heroLadrc);
void Ladrc_Init_all(void);
#endif
