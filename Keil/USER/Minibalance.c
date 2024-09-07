#include "sys.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/ 
u8 Flag_Left,Flag_Right,Flag_Direction=0;   //����ң����صı���
u8 Flag_Stop=1,Flag_Show=0;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_A,Encoder_B,Encoder_C;          //���������������
float Position_A,Position_B,Position_C,Rate_A,Rate_B,Rate_C; //PID������ر���                      
long int Motor_A,Motor_B,Motor_C;        //���PWM����
long int Target_A,Target_B,Target_C;     //���Ŀ��ֵ
int Voltage;                             //��ص�ѹ������صı���
float Show_Data_Mb;                      //ȫ����ʾ������������ʾ��Ҫ�鿴������                         
u8 delay_50,delay_flag;                          //��ʱ��ر���
u8 Run_Flag=0;  //����ң����ر���������״̬��־λ
u8 rxbuf[11],Urxbuf[11],CAN_ON_Flag=0,Usart_ON_Flag=0,Usart_Flag,PID_Send,Flash_Send;  //CAN�ʹ��ڿ�����ر���
u8 txbuf[24],txbuf2[24],Turn_Flag;             //CAN������ر���
float Pitch,Roll,Yaw,Move_X,Move_Y,Move_Z;   //����ǶȺ�XYZ��Ŀ���ٶ�
u16 PID_Parameter[10],Flash_Parameter[10];  //Flash�������
float	Position_KP=20,Position_KI=0,Position_KD=20;  //λ�ÿ���PID����
int RC_Velocity=800,Max_Velocity,RC_Position=5000;         //����ң�ص��ٶȺ�λ��ֵ
int main(void)
{ 
	Stm32_Clock_Init(9);            //=====ϵͳʱ������
	delay_init(72);                 //=====��ʱ��ʼ��
	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
	LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
	KEY_Init();                     //=====������ʼ��
	Run_Flag=0;          						//=====�ٶ�ģʽ
	OLED_Init();                    //=====OLED��ʼ��
	uart2_init(36,9600);            //=====����2����A2��A3��ʼ��
  uart3_init(36,115200);          //=====����3ROSC10��C11��ʼ�� 
	Adc_Init();                     //=====adc��ʼ��
	IIC_Init();                     //=====IIC��ʼ��
  MPU6050_initialize();           //=====MPU6050��ʼ��	
  DMP_Init();                     //=====��ʼ��DMP     
	if(KEY==0) Flash_Read();        //=====��ȡFlash����Ĳ���
	delay_ms(1000);                 //=====��ʱ�ȴ���ʼ���ȶ�
  EXTI_Init();                    //=====MPU6050 5ms��ʱ�жϳ�ʼ��
  CAN1_Mode_Init(1,2,3,6,0);      //=====CAN��ʼ��
	MiniBalance_PWM_Init(7199,14);  //=====��ʼ��PWM �����������
	delay_ms(1000);
	while(1)
		{		
	  	if(Flash_Send==1)          //д��PID������Flash,��app���Ƹ�ָ��
			{
				Flash_Write();	
				Flash_Send=0;	
			}	
			APP_Show();
			USART3_SEND();
	    if(Flag_Show==0)oled_show1();             //===��ʾ����
			else oled_show2();
			delay_flag=1;	
			delay_50=0;
			while(delay_flag);	       //ͨ��MPU6050��INT�ж�ʵ�ֵ�50ms��׼��ʱ				
		} 
}
