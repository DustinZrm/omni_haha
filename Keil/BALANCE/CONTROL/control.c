#include "control.h"	
#include "filter.h"	
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/

u8 Flag_Target,Flag_Change;                             //��ر�־λ
u8 temp1;                                               //��ʱ����
float Voltage_Count,Voltage_All;  //��ѹ������ر���
float Gyro_K=0.004;       //�����Ǳ���ϵ��
int j;
int Flag_Jiasu;
#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER            (1.0f)  
/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ�����X Y Z �����ٶȻ���λ��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
        Target_A   = Vx + L_PARAMETER*Vz+gyro[2]*Gyro_K;
        Target_B   = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz+gyro[2]*Gyro_K;
	      Target_C   = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz+gyro[2]*Gyro_K;
}
/**************************************************************************
�������ܣ���ȡλ�ÿ��ƹ����ٶ�ֵ
��ڲ�����X Y Z ����λ�ñ仯��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis2(float Vx,float Vy,float Vz)
{
	      Rate_A   = Vx + L_PARAMETER*Vz;
        Rate_B   =-X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
	      Rate_C   =-X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;
}
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	 if(INT==0)		
	{     
		  EXTI->PR=1<<15;                                                      //���LINE5�ϵ��жϱ�־λ  		
		  Flag_Target=!Flag_Target;
		  if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //���������ṩ50ms�ľ�׼��ʱ
			 }
		  if(Flag_Target==1)                                                  //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ
			{
				  CAN1_SEND();                                                          //CAN����
					if(Usart_Flag==0&&Usart_ON_Flag==1)  memcpy(rxbuf,Urxbuf,11*sizeof(u8));	//��������˴��ڿ��Ʊ�־λ�����봮�ڿ���ģʽ
					Read_DMP();                                                           //===������̬		
			  	Key();//ɨ�谴���仯	
			return 0;	                                               
			}                            					 //===10ms����һ��
			Read_DMP();                  				 //===������̬	
			Encoder_A=Motor_A;            	 		//===ͨ��Ƶ�ʽ����ٶ�
	  	Encoder_B=Motor_B;           		 		//===ͨ��Ƶ�ʽ����ٶ�
	  	Encoder_C=Motor_C;            	 	 //===ͨ��Ƶ�ʽ����ٶ�
			Position_A+=4.8*Encoder_A*0.01;     //===ͨ��Ƶ�ʻ��ּ���λ��
	  	Position_B+=4.8*Encoder_B*0.01;    //===ͨ��Ƶ�ʻ��ּ���λ��
		  Position_C+=4.8*Encoder_C*0.01;    //===ͨ��Ƶ�ʻ��ּ���λ��
  		Led_Flash(100);                     //===LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬	
			Voltage_All+=Get_battery_volt();     //��β����ۻ�
			if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ ��ȡ��ص�ѹ	       
		  if(CAN_ON_Flag==1||Usart_ON_Flag==1) CAN_N_Usart_Control();       //�ӵ����ڻ���CANң�ؽ���ָ��֮��ʹ��CAN�ʹ��ڿ�������
		 if(Turn_Off(Voltage)==0)               //===�����ص�ѹ�������쳣
		 { 			 
				if(CAN_ON_Flag==0&&Usart_ON_Flag==0)  Get_RC(Run_Flag);  //===���ں�CAN���ƶ�δʹ�ܣ����������ң��ָ
				Motor_A=Target_A;//ֱ�ӵ���PWMƵ��
				Motor_B=Target_B;//ֱ�ӵ���PWMƵ��
				Motor_C=Target_C;//ֱ�ӵ���PWMƵ��
				Xianfu_Pwm(5000);  //===PWMƵ���޷�
				Set_Pwm(Motor_A,Motor_B,Motor_C);    //��ֵ��PWM�Ĵ���
		 }
	 }
	 return 0;	 
} 


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c)
{
			int Final_Motor_A,Final_Motor_B,Final_Motor_C;
	    static int flag,count;
	   	if(motor_a>0)			    INA=0;   //���A�������
			else 	             	  INA=1;
	   	if(motor_b>0)			    INB=0;   //���B�������
			else 	             	  INB=1;
			if(motor_c>0)			    INC=0;   //���C�������
			else 	                INC=1;
			//���ʧ�ܼ��
			if(flag==0)
			{
					if(0==Target_A&&0==Target_B&&0==Target_C)  count++;
					else count=0;
					if(count>200)
					{
						flag=1;
						count=0;
					}
					ST	=1;	
			}
				if(flag==1)
				{
						ST	=0;
						if(Target_A!=0||Target_B!=0||Target_C!=0)
						flag=0;	
				}
	    Final_Motor_A=Linear_Conversion(motor_a);  //���Ի�
    	Final_Motor_B=Linear_Conversion(motor_b);
			Final_Motor_C=Linear_Conversion(motor_c);
			Set_PWM_Final(Final_Motor_A,Final_Motor_B,Final_Motor_C);  
}
/**************************************************************************
�������ܣ��Կ��������PWM���Ի�,���ڸ�ϵͳ�Ĵ�����ֵ
��ڲ�����PWM
����  ֵ�����Ի����PWM
**************************************************************************/
u16  Linear_Conversion(int motor)
{ 
	 u32 temp;
   u16 Linear_Moto;
   temp=1000000/myabs(motor);   //1000000�Ǿ���ֵ
	 if(temp>65535) Linear_Moto=65535;
	 else Linear_Moto=(u16)temp;
	 return Linear_Moto;
}	

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_A<-amplitude) Motor_A=-amplitude;	
		if(Motor_A>amplitude)  Motor_A=amplitude;	
	  if(Motor_B<-amplitude) Motor_B=-amplitude;	
		if(Motor_B>amplitude)  Motor_B=amplitude;		
	  if(Motor_C<-amplitude) Motor_C=-amplitude;	
		if(Motor_C>amplitude)  Motor_C=amplitude;		
}
/**************************************************************************
�������ܣ�λ��PID���ƹ������ٶȵ�����
��ڲ������ޡ���ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//λ�ÿ���ģʽ�У�A����������ٶ�
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //λ�ÿ���ģʽ�У�A����������ٶ�
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//λ�ÿ���ģʽ�У�B����������ٶ�
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//λ�ÿ���ģʽ�У�B����������ٶ�
	  if(Motor_C<-amplitude_C) Motor_C=-amplitude_C;	//λ�ÿ���ģʽ�У�C����������ٶ�
		if(Motor_C>amplitude_C)  Motor_C=amplitude_C;		//λ�ÿ���ģʽ�У�C����������ٶ�
}
/**************************************************************************
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double(100);    
	if(tmp==2){
		Flag_Show=!Flag_Show;//˫��������ʾģʽ                  
		OLED_Clear();	
	}
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage < 1100)//��ص�ѹ����11.1V�رյ��
			{	                                                
      temp=1;      
      PWMA=0;
      PWMB=0;
      PWMC=0;
      INA=0;
      INB=0;
      INC=0;	
			ST=0;   //ʧ�ܵ��
      }
			else
      temp=0;
      return temp;			
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ 
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
int Position_PID_A (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
int Position_PID_B (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
int Position_PID_C (int Encoder,int Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Encoder-Target;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
/**************************************************************************
�������ܣ�ͨ������ָ���С������ң��
��ڲ���������ָ��
����  ֵ����
**************************************************************************/
void Get_RC(u8 mode)
{

	float step=10;  //�����ٶȿ��Ʋ���ֵ��
	u8 Flag_Move=1;
				 switch(Flag_Direction)   //�������
				 {
				 case 1:  Move_X=0;           Move_Y-=step;  Flag_Move=1;               break;
				 case 2:  Move_X+=step;       Move_Y-=step;  Flag_Move=1;               break;
				 case 3:  Move_X+=step;       Move_Y=0;      Flag_Move=1;               break;
				 case 4:  Move_X+=step;       Move_Y+=step;  Flag_Move=1;               break;
				 case 5:  Move_X=0;           Move_Y+=step;  Flag_Move=1;               break;
				 case 6:  Move_X-=step;       Move_Y+=step;  Flag_Move=1;               break;
				 case 7:  Move_X-=step;       Move_Y=0;      Flag_Move=1;               break;
				 case 8:  Move_X-=step;       Move_Y-=step;  Flag_Move=1;               break; 
				 default: Flag_Move=0;    		Move_X=Move_X/1.1;	Move_Y=Move_Y/1.1;	      break;	 
			
			 }			 
			if(Flag_Move==0)		//����޷������ָ��	 �����ת�����״̬
			{	
				if(Flag_Left==1)        Move_Z-=step,Gyro_K=0;    //������   
				else if(Flag_Right==1) 	Move_Z+=step,Gyro_K=0;    //������		
				else 		                Move_Z=0,Gyro_K=0.004;    //ֹͣ
			}	
				if(Flag_Move==1)	Flag_Left=0,Flag_Right=0,Move_Z=0;
				if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //�ٶȿ����޷�
				if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
				if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
				if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
				if(Move_Z<-RC_Velocity) Move_Z=-RC_Velocity;	
				if(Move_Z>RC_Velocity)  Move_Z=RC_Velocity;	 
		 Kinematic_Analysis(Move_X,Move_Y,Move_Z);//�õ�����Ŀ��ֵ�������˶�ѧ����
}


/**************************************************************************
�������ܣ�����Ҫ����/���յ�����У����
��ڲ�����Count_Number��У���ǰ��λ����Mode��0-�Խ������ݽ���У�飬1-�Է������ݽ���У��
����  ֵ��У����
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//��Ҫ���͵����ݽ���У��
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^txbuf[k];
	}
	
	//�Խ��յ������ݽ���У��
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^rxbuf[k];
	}
	return check_sum;
}

/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
�������ܣ�����λ���������ĸ�8λ�͵�8λ�������ϳ�һ��short�����ݺ�������λ��ԭ����
��ڲ�������8λ����8λ
����  ֵ��������X/Y/Z���Ŀ���ٶ�
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//����ת�����м����
	short transition; 
	
	//����8λ�͵�8λ���ϳ�һ��16λ��short������
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return 
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //��λת��, mm/s->m/s						
}


/**************************************************************************
�������ܣ�����CAN���ߴ��ڿ���ָ����д���
��ڲ�������
����  ֵ����
**************************************************************************/
void CAN_N_Usart_Control(void)
{
	if (rxbuf[10] == 0x7d){
		if (rxbuf[9] == Check_Sum(9,0)||1){
			//Usart_ON_Flag=0;
			//�Ӵ�������������Ŀ���ٶȣ� ��λm/s
			Move_Y=-XYZ_Target_Speed_transition(rxbuf[3],rxbuf[4])*4000;
			Move_X=-XYZ_Target_Speed_transition(rxbuf[5],rxbuf[6])*4000;
			Move_Z=-XYZ_Target_Speed_transition(rxbuf[7],rxbuf[8])*780;
			Kinematic_Analysis(Move_X,Move_Y,Move_Z),Gyro_K=0;    //�����˶�ѧ����
		}
	}
}



