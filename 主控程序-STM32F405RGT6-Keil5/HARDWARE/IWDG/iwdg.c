#include "iwdg.h"
/*��ʼ���������Ź�*/
//prer:��Ƶ��:0~7(ֻ�е�3λ��Ч!)
//rlr:�Զ���װ��ֵ,0~0XFFF.
//��Ƶ����=4*2^prer.�����ֵֻ����256!
//rlr:��װ�ؼĴ���ֵ:��11λ��Ч.
//ʱ�����(���):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(u8 prer,u16 rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ʹ�ܶ�IWDG->PR IWDG->RLR��д
	IWDG_SetPrescaler(prer); //����IWDG��Ƶϵ��
	IWDG_SetReload(rlr);   //����IWDGװ��ֵ
	IWDG_ReloadCounter(); //reload
	IWDG_Enable();       //ʹ�ܿ��Ź�
}
//ι�������Ź�
void IWDG_Feed(void)
{
	IWDG_ReloadCounter();//reload
}
//������������ʱ�䣬��λS��
void My_IWDG_Init(u8 time)
{
	u16 rlr;
	rlr=time*500;
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ʹ�ܶ�IWDG->PR IWDG->RLR��д
	IWDG_SetPrescaler(4); //����IWDG��Ƶϵ��=4
	IWDG_SetReload(rlr);   //����IWDGװ��ֵ
	IWDG_ReloadCounter(); //reload
	IWDG_Enable();       //ʹ�ܿ��Ź�
}
