#ifndef _LADRC_H
#define _LADRC_H
#include "stdint.h"
#include "stm32h7xx_hal.h"

#define square(a) (a)*(a)
#define cube(a) (a)*(a)*(a)
/**
   *@Brief  以下为LADRC系统参数
   *@WangShun  2022-07-03  注释
   */
typedef struct
{
	double v1;            //最速输出值
	double v2;            //最速输出值
	double h;             //积分步长
	double r;             //速度因子
	double kp;
	double kd;
	double b0;            //系统参数
	double beta;
	double z1;            //观测器输出
	double z2;			 //观测器输出
	double z3;		 	 //观测器输出
	double u;             //控制器输出
	double u0;
}Ladrc_t;

extern Ladrc_t heroLadrc;
/**
   *@Brief  以下为LADRC相关函数
   *@WangShun  2022-07-03  注释
   */
double LADRC_Loop(double Expect, double Measure/*反馈*/, double DevMeasure ,Ladrc_t *heroLadrc);
void Ladrc_Init_all(void);
#endif
