#include "show.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
float Vol;
/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show1(void)
{
		OLED_ShowString(120,0,">");
	  //=============��1����ʾ3��Ƕ�===============//	
  	OLED_ShowString(0,0,"X:");
		if(Pitch<0)		OLED_ShowNumber(15,0,Pitch+360,3,12);
		else					OLED_ShowNumber(15,0,Pitch,3,12);	
       
  	OLED_ShowString(40,0,"Y:");
		if(Roll<0)		OLED_ShowNumber(55,0,Roll+360,3,12);
		else					OLED_ShowNumber(55,0,Roll,3,12);	
	
	   OLED_ShowString(80,0,"Z:");
		if(Yaw<0)		OLED_ShowNumber(95,0,Yaw+360,3,12);
		else					OLED_ShowNumber(95,0,Yaw,3,12);		
	  //=============�ڶ���Z�������Ǻ�Ŀ���ٶ�===============//	
			                    OLED_ShowString(00,10,"GZ");
		if( gyro[2]<0)      	OLED_ShowString(20,10,"-"),
		                      OLED_ShowNumber(30,10,-gyro[2],5,12);
		else                 	OLED_ShowString(20,10,"+"),
		                      OLED_ShowNumber(30,10, gyro[2],5,12);			
		
					                OLED_ShowString(70,10,"V");
		if( RC_Velocity<0)	  OLED_ShowString(80,10,"-"),
		                      OLED_ShowNumber(90,10,-RC_Velocity,5,12);
		else                 	OLED_ShowString(80,10,"+"),
		                      OLED_ShowNumber(90,10, RC_Velocity,5,12);	
  if(Run_Flag==0)
  {		
		//=============��3����ʾ���A��״̬=======================//	
		  if( Target_A<0)		  OLED_ShowString(00,20,"-"),
		                      OLED_ShowNumber(15,20,-Target_A,5,12);
		else                 	OLED_ShowString(0,20,"+"),
		                      OLED_ShowNumber(15,20, Target_A,5,12); 
		
		if( Encoder_A<0)		OLED_ShowString(80,20,"-"),
		                      OLED_ShowNumber(95,20,-Encoder_A,4,12);
		else                 	OLED_ShowString(80,20,"+"),
		                      OLED_ShowNumber(95,20, Encoder_A,4,12);
 		//=============��4����ʾ���B��״̬=======================//	
		  if( Target_B<0)		OLED_ShowString(00,30,"-"),
		                      OLED_ShowNumber(15,30,-Target_B,5,12);
		else                 	OLED_ShowString(0,30,"+"),
		                      OLED_ShowNumber(15,30, Target_B,5,12); 
		
		if( Encoder_B<0)		OLED_ShowString(80,30,"-"),
		                      OLED_ShowNumber(95,30,-Encoder_B,4,12);
		else                 	OLED_ShowString(80,30,"+"),
		                      OLED_ShowNumber(95,30, Encoder_B,4,12);	
		//=============��5����ʾ���C��״̬=======================//	
		  if( Target_C<0)		OLED_ShowString(00,40,"-"),
		                      OLED_ShowNumber(15,40,-Target_C,5,12);
		else                 	OLED_ShowString(0,40,"+"),
		                      OLED_ShowNumber(15,40, Target_C,5,12); 
		
		if( Encoder_C<0)		OLED_ShowString(80,40,"-"),
		                      OLED_ShowNumber(95,40,-Encoder_C,4,12);
		else                 	OLED_ShowString(80,40,"+"),
		                      OLED_ShowNumber(95,40, Encoder_C,4,12);
	}
	else if(Run_Flag==1)
  {		
		//=============��3����ʾ���A��״̬=======================//	
		  if( Target_A<0)		  OLED_ShowString(00,20,"-"),
		                      OLED_ShowNumber(15,20,-Target_A,6,12);
		else                 	OLED_ShowString(0,20,"+"),
		                      OLED_ShowNumber(15,20, Target_A,6,12); 
		
		if( Position_A<0)		  OLED_ShowString(70,20,"-"),
		                      OLED_ShowNumber(90,20,-Position_A,6,12);
		else                 	OLED_ShowString(70,20,"+"),
		                      OLED_ShowNumber(90,20, Position_A,6,12);
 			//=============��4����ʾ���B��״̬=======================//	
		  if( Target_B<0)		OLED_ShowString(00,30,"-"),
		                      OLED_ShowNumber(15,30,-Target_B,6,12);
		else                 	OLED_ShowString(0,30,"+"),
		                      OLED_ShowNumber(15,30, Target_B,6,12); 
		
		if( Position_B<0)		OLED_ShowString(70,30,"-"),
		                      OLED_ShowNumber(90,30,-Position_B,6,12);
		else                 	OLED_ShowString(70,30,"+"),
		                      OLED_ShowNumber(90,30, Position_B,6,12);	
			//=============��5����ʾ���C��״̬=======================//	
		  if( Target_C<0)		OLED_ShowString(00,40,"-"),
		                      OLED_ShowNumber(15,40,-Target_C,6,12);
		else                 	OLED_ShowString(0,40,"+"),
		                      OLED_ShowNumber(15,40, Target_C,6,12); 
		
		if( Position_C<0)		OLED_ShowString(70,40,"-"),
		                      OLED_ShowNumber(90,40,-Position_C,6,12);
		else                 	OLED_ShowString(70,40,"+"),
		                      OLED_ShowNumber(90,40, Position_C,6,12);
	}
		//=============��6����ʾģʽ�͵�ѹ=======================//
		if(ST)       OLED_ShowString(30,50," RUN");
	else OLED_ShowString(30,50,"STOP");
													
	if(Usart_ON_Flag) OLED_ShowString(00,50,"ROS");
	else OLED_ShowString(00,50,"APP");
		                      OLED_ShowString(88,50,".");
		                      OLED_ShowString(110,50,"V");
		                      OLED_ShowNumber(75,50,Voltage/100,2,12);
		                      OLED_ShowNumber(98,50,Voltage%100,2,12);
		 if(Voltage%100<10) 	OLED_ShowNumber(92,50,0,2,12);
		//=============ˢ��=======================//
		OLED_Refresh_Gram();	
	}

void oled_show2(void){
	OLED_ShowString(120,0,"<");
	OLED_ShowNumber(15,20,Move_X,6,12);
	OLED_ShowNumber(15,30,Move_Y,6,12);
	OLED_ShowNumber(15,40,Move_Z,6,12);
	OLED_ShowChar(30, 50 ,rxbuf[10], 12, 1);
	OLED_Refresh_Gram();
}
/**************************************************************************
�������ܣ���APP��������
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void APP_Show(void)
{    
	  static u8 flag;
	  int app_2,app_3,app_4;
	  app_4=(Voltage-1110)*2/3;	
	  if(app_4>100)app_4=100;   //�Ե�ѹ���ݽ��д���
	  app_2=Move_X*0.2;  if(app_2<0)app_2=-app_2;			                   //�Ա��������ݾ������ݴ������ͼ�λ�
		app_3=Move_Y*0.2;  if(app_3<0)app_3=-app_3;
	   if(Run_Flag==1)app_2=0,app_3=0;
	  flag=!flag;
	 	if(PID_Send==1)//����PID����
	{
		printf("{C%d:%d:%d:%d:%d}$",(int)RC_Velocity,(int)RC_Position,(int)Position_KP,(int)Position_KI,(int)Position_KD);//��ӡ��APP����	
		PID_Send=0;	
	}	
   else	if(flag==0)// 
   printf("{A%d:%d:%d:%d}$",(u8)app_2,(u8)app_3,app_4,0); //��ӡ��APP����
	 else
	 printf("{B%d:%d:%d:%d}$",(int)Pitch,(int)Roll,(int)Yaw,app_4);//��ӡ��APP���� ��ʾ����
}
