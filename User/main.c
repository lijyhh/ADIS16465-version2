/**
  ******************************************************************************
  * @file    main.c
  * @author  Dong-HIT
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   SPI ADIS基本读写例程,主要函数在bsp_spi_adis.c中
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "stm32f4xx.h"
#include "./led/bsp_led.h"
#include "./usart/bsp_debug_usart.h"
#include "./adis/bsp_spi_adis.h" 
#include "./exit/bsp_exti.h"
#include "./systick/bsp_SysTick.h"
#include "./kalman/kalman.h"
#include "./attitude_solution/attitude_solution.h"

//全局变量
struct angle euler={0,0,0};
int new_data;
int ircnt=0;
// 延时函数
void Delay_us(u32 time)
{
  u32 i=8*time;
  while(i--);
}

/*简单任务管理*/
uint32_t Task_Delay[NumOfTask]={0};

/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 */
int main(void)
{ 		
	float Acel[3] = {0};
	float Gyro[3] = {0};
	float Acel2Angle[3] = {0};
	float Angle[3] = {0};
	float Velocity[3] = {0};
	float Temp = 0;	
	int count = 0;
	int cnt = 0;
	float avTemp = 0;	
	float endAngle = 0;	
//	int Temp1;	

	//定义一个卡尔曼运算结构体，三个方向三个结构体
	KalmanCountData k[3];
	Kalman_Filter_Init(k);
	
	/******************** 各种初始化 **************************/
	LED_GPIO_Config();
	SysTick_Init();	
	Debug_USART_Config();	
	EXTI_Config();
	ADIS16465_Init();	 	
	
	
	//检测ADIS16564
	if (SPI_ADIS_ReadID())
	{	
		while(1)
		{
			if(new_data)
			{				
					AHRS_update();
					printf("z偏航角=%3.5f\n",euler.yaw);
//					printf(" y俯仰角=%3.5f, x横滚角=%3.5f\n",euler.pitch,euler.roll);
					new_data = 0;				
			}
//			if(Task_Delay[0]==TASK_ENABLE)
//			{				
//				AHRS_update();
//				printf(" y俯仰角=%3.2f, x横滚角=%3.2f, z偏航角=%3.2f\n",euler.pitch,euler.roll,euler.yaw);
//				
////				LED2_TOGGLE;
//				Task_Delay[0]=500;
// 
//			}
//			
//			if(Task_Delay[1]==0)
//			{
////				ADIS16465ReadAcc(Acel);
////				printf(" 加速度：%5.5f  %5.5f  %5.5f\n",Acel[0],Acel[1],Acel[2]);
////				
////				ADIS16465ReadGyro(Gyro);
////				printf(" 陀螺仪：%2.5f  %2.5f  %2.5f\n",Gyro[0],Gyro[1],Gyro[2]);
////			
////				ADIS16465ReadAngle(Angle);
////				printf(" 角位移：%2.5f  %2.5f  %2.5f\n",Angle[0],Angle[1],Angle[2]);
////				
////				ADIS16465ReadVelocity(Velocity);
////				printf("   速度：%2.5f  %2.5f  %2.5f\r\n",Velocity[0],Velocity[1],Velocity[2]);
//				
////				accel2angle(Acel,Acel2Angle);
////				printf("转换的角度：%3.8f  %3.8f  %3.8f\r\n",Acel2Angle[0],Acel2Angle[1],Acel2Angle[2]);
//				
////				Kalman_Filter(Acel2Angle[0],Gyro[0],&k[0]);
//////				printf("1   %2.5f and %2.5f -------- %f and %f\n",Acel2Angle[0],Gyro[0],k[0].Angle_Final,k[0].Gyro_Final);
////				Kalman_Filter(Acel2Angle[1],Gyro[1],&k[1]);
//////				printf("2   %2.5f and %2.5f -------- %f and %f\n",Acel2Angle[1],Gyro[1],k[1].Angle_Final,k[1].Gyro_Final);
////				Kalman_Filter(Acel2Angle[2],Gyro[2],&k[2]);
//////				printf("3   %2.5f and %2.5f -------- %f and %f\n\r",Acel2Angle[2],Gyro[2],k[2].Angle_Final,k[2].Gyro_Final);
////					printf("Angle1=%2.3f Angle2=%2.3f Angle3=%2.3f------Gyro1=%2.3f Gyro2=%2.3f Gyro3=%2.3f\n",Acel2Angle[0],\
////					Acel2Angle[1],Acel2Angle[2],Gyro[0],Gyro[1],Gyro[2]);
////					printf("Anglex=%2.3f Angley=%2.3f Anglez=%2.3f------Gyrox=%2.3f Gyroy=%2.3f Gyroz=%2.3f\n\r",k[0].Angle_Final,\
////					k[1].Angle_Final,k[2].Angle_Final,k[0].Gyro_Final,k[1].Gyro_Final,k[2].Gyro_Final);

////				printf("\n\r");
////				ADIS16465ReadTemp(&Temp);
////				printf("    温度%.1f\r\n",Temp);				
//				
////				Temp1 = ADIS_ReadReg(DATA_CNTR);
////				printf(" 个数：%d\r\n",Temp1);	
//				
//				Task_Delay[1]=100; //更新一次数据，可根据自己的需求，提高采样频率，如100ms采样一次
//				
//			 }

			//*************************************	下面是增加任务的格式************************************//
	//		if(Task_Delay[i]==0)
	//		{
	//			Task(i);
	//			Task_Delay[i]=;
	//		}

		 }
	}
	else
	{
			LED_RED; 	
			while(1);		
	 }
}

/*********************************************END OF FILE**********************/


