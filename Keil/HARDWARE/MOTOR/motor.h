#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
#define PWMC   TIM8->CCR3  
#define PWMB   TIM8->CCR2 
#define PWMA   TIM8->CCR1 

#define INC   PCout(5)  
#define INB   PBout(0)  
#define INA   PCout(4)  
#define ST    PCout(3) 
void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
void Set_PWM_Final(u16 arr1,u16 arr2,u16 arr3);
#endif
