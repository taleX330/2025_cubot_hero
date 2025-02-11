#include "ladrc.h"

/**
* ����˵����LADRC��ʼ��
* 		YuTong��202-09-28����
*/	
static void Ladrc_Init(Ladrc_t *ladrc, float H, float R, float Kp, float Kd, float B0, float Beta)
{
	
	ladrc->h  = H;//��ʱʱ�估ʱ�䲽��
	ladrc->r  = R;//�����ٶȲ���
	ladrc->kp = Kp;//�۲�������
	ladrc->kd = Kd;//�۲�������
	ladrc->b0 = B0;//ϵͳ����
	ladrc->beta = Beta;//״̬�����ʴ���
	ladrc->z1 = 0;
	ladrc->z2 = 0;
	ladrc->z3 = 0;
	ladrc->u  = 0;
	ladrc->v1 = 0;
	ladrc->v2 = 0;
	ladrc->u0 = 0;
}
/**
* ��������void Ladrc_TD(Ladrc_t *ladrc,float Target)
* ����˵����LADRC����΢�ֲ���
* @param[in]	��ڲ���������ֵExpect(v0)���ֵv1,v2
* @par �޸���־
* 		WangShun��2022-05-28����
*         YuTong��202-09-28�޸�
*/
static void Ladrc_TD(Ladrc_t *ladrc,float Target)
{
    double fh= -ladrc->r * ladrc->r * (ladrc->v1-Target)-2 * ladrc->r * ladrc->v2;
    ladrc->v1 += ladrc->v2 * ladrc->h;
    ladrc->v2 += fh * ladrc->h;//΢����
}
/**
* ��������LADRC_ESO_ZOH(Ladrc_t *ladrc,double Measure,double RealTimeOut)
* ����˵����LADRC����״̬�۲���
* @param[in]
* @par �޸���־
* 		WangShun��2022-07-03����
*         YuTong��202-09-28�޸�
*/
static void LADRC_ESO_ZOH(Ladrc_t *ladrc,double Measure)
{	

	double z1k,z2k,z3k;
	z1k = ladrc->z1;
	z2k = ladrc->z2;
	z3k = ladrc->z3;
	
	ladrc->z1 = ladrc->h * z2k + square(ladrc->h) * z3k/2 - Measure * (3 * ladrc->beta - 3) + z1k * (3 * ladrc->beta - 2) + square(ladrc->b0 * ladrc->h) * ladrc->u / 2;
	ladrc->z2 = z2k + ladrc->h * z3k + ladrc->b0 * ladrc->h * ladrc->u + (Measure * square(ladrc->beta - 1)*(ladrc->beta + 5))/(2*ladrc->h) - (z1k*square(ladrc->beta - 1)*(ladrc->beta + 5))/(2*ladrc->h);
	ladrc->z3 = z3k - (Measure*cube(ladrc->beta - 1))/square(ladrc->h) + (z1k*cube(ladrc->beta - 1))/square(ladrc->h);
}
/**
*@Brief  LADRC_LSEF
*@Date   ���Կ�����
*		WangShun��2022-07-03����
*         YuTong��202-09-28�޸�
*/
static void LADRC_LF(Ladrc_t *ladrc,double Measure,double DevMeasure)
{
//    float Kp=ladrc->wc * ladrc->wc;
//    float Kd=2 * ladrc->wc;
/**
  *@Brief  ���Կ�����������kd = 2wc
  *@Before Kd=3*ladrc->wc;
  *@Now    Kd=2*ladrc->wc;
  *@WangShun  2022-04-27  ע��
  */
    double e1=ladrc->v1 - Measure;
    double e2= - ladrc->z2;
//	double e2= - DevMeasure;
    double u0=ladrc->kp * e1 + ladrc->kd * e2;
		ladrc->u0 = u0;
    ladrc->u = (u0-ladrc->z3) / ladrc->b0;
	if(ladrc->u > 16000)
		ladrc->u = 16000;
	else if(ladrc->u < -16000)
		ladrc->u = -16000;
}
/**
  * LADRC���ƺ��� .
  * ������������ѭ���м���
  * @par ����
  * @par �޸���־
  */
double LADRC_Loop(double Target, double Measure/*����*/, double DevMeasure ,Ladrc_t *ladrc)
{
    Ladrc_TD(ladrc,Target);
    LADRC_ESO_ZOH(ladrc,Measure); 
    LADRC_LF(ladrc, Measure, DevMeasure);
	return ladrc->u;
}

void Ladrc_Init_all(void)
{
}
