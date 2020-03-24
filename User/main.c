/**
  ******************************************************************************
  * @file    main.c
  * @author  DZH-HIT
  * @version V1.0
  * @date    2020-xx-xx
  * @brief   ADIS读取数据并进行姿态解算
	**main.c
	**stm32f4xx_it.c			该文件中处理SysTick中断和ADIS中断
	**bsp_spi_adis.c			关于imu的数据读取和初始化函数都在该文件中
	**bsp_exti.c					配置ADIS中断引脚
	**kalman.c						卡尔曼滤波的相关代码
	**attitude_solution.c	姿态解算的相关代码
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
//						printf(" y俯仰角=%3.5f, x横滚角=%3.5f，z偏航角=%3.5f\n",euler.pitch,euler.roll,euler.yaw);
						new_data = 0;				
				}
		 }
	}
	else
	{
			LED_RED; 	
			while(1);		
	 }
}

/*********************************************END OF FILE**********************/


