/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include "Remote_Control.h"
#include <cmath>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
PID_TypeDef motor_pid[4];
PID_TypeDef fric_pid[2];
PID_TypeDef GMP_pos_pid;
PID_TypeDef GMP_speed_pid;
PID_TypeDef plate_pid;
int32_t fric_speed = 1200;
int32_t tmp_fric_speed = 0;
int32_t cm_speed_forw = 0;
int32_t cm_speed_left = 0;
int32_t cm_speed_rotate = 0;
int32_t gm_speed = 0;
int32_t gm_pos = 0;
double pwm_pos = 0.1;
int32_t plate_speed = 0;
int32_t PLATE_MAX_SPEED = 500;
int32_t set_spd = 0;
int32_t FRIC_MAX_SPEED = 4000;
int32_t CM_MAX_SPEED = 1000;
uint8_t rec[20];
double distance = 0;
double lidar_tf = 0.10;
int32_t calc_speed(double,double);

int speed_step_sign = +1;

uint16_t TIM_COUNT[2];
#define SpeedStep 500
#define ABS(x)	( (x>0) ? (x) : (-x) )

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty){
	
	switch(tim_channel){	
		case TIM_CHANNEL_1: tim->Instance->CCR1 = (PWM_RESOLUTION*duty) - 1;break;
		case TIM_CHANNEL_2: tim->Instance->CCR2 = (PWM_RESOLUTION*duty) - 1;break;
		case TIM_CHANNEL_3: tim->Instance->CCR3 = (PWM_RESOLUTION*duty) - 1;break;
		case TIM_CHANNEL_4: tim->Instance->CCR4 = (PWM_RESOLUTION*duty) - 1;break;
	}
}

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
	MX_USART6_UART_Init();
  MX_TIM1_Init();
	MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
	/* Open 4 sets of PWM waves respectively */
		/**TIM2 GPIO Configuration    
	PA1     ------> TIM2_CH2
	PA0     ------> TIM2_CH1
	PA2     ------> TIM2_CH3
	PA3     ------> TIM2_CH4 
	*/
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);

	/**TIM4 GPIO Configuration    
	PD15     ------> TIM4_CH4
	PD14     ------> TIM4_CH3
	PD13     ------> TIM4_CH2
	PD12     ------> TIM4_CH1 
	*/
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

	/**TIM5 GPIO Configuration    
	PI0     ------> TIM5_CH4
	PH12     ------> TIM5_CH3
	PH11     ------> TIM5_CH2
	PH10     ------> TIM5_CH1 
	*/

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	
	
	/**TIM8 GPIO Configuration    
	PI7     ------> TIM8_CH3
	PI6     ------> TIM8_CH2
	PI5     ------> TIM8_CH1
	PI2     ------> TIM8_CH4 
	*/
		
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	
  /* USER CODE BEGIN 2 */
  my_can_filter_init_recv_all(&hcan1);     //配置CAN过滤器
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);   //启动CAN接收中断
  HAL_UART_Receive_IT_IDLE(&huart1,UART_Buffer,100);   //启动串口接收

  HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_2,(uint32_t *)TIM_COUNT,2);
	get_moto_offset(&moto_chassis[2],&hcan1);
	
	for(int i=0;i<20;i++)
		{rec[i]='0';}
	/*< 初始化PID参数 >*/
  //chasis motor
	for(int i=0; i<4; i++)
  {	
    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i],PID_Speed,16384,5000,10,0,8000,0,1.5,0.1,0);
  }
	//friction wheel
	for(int i=0; i<2; i++)
  {
    pid_init(&fric_pid[i]);
    fric_pid[i].f_param_init(&fric_pid[i],PID_Speed,16384,5000,10,0,8000,0,1.5,0.1,0);
  }
	//gimbal motor
	pid_init(&GMP_pos_pid);
  GMP_pos_pid.f_param_init(&GMP_pos_pid,PID_Speed,16384,5000,10,0,8000,0,10,0,0);
	pid_init(&GMP_speed_pid);
  GMP_speed_pid.f_param_init(&GMP_speed_pid,PID_Speed,16384,5000,10,0,8000,0,10,0.1,0);
	//plate driver
	pid_init(&plate_pid);
  plate_pid.f_param_init(&plate_pid,PID_Speed,16384,5000,10,0,8000,0,20,0,0);
	
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		get_total_angle(&moto_chassis[2]);
		distance = 0;
		
		//HAL_UART_Receive_IT(&huart6,(uint8_t*)rec,12);
		//
		//if(rec[1]=='0')
		//{
		//	tmp_fric_speed = fric_speed;
		//}
		//else
		//{
		//	for (int i = 0; i < 20; i++)
		//	{
		//		switch (i)
		//		{
		//			case 0:
		//				distance += rec[i]-'0';
		//				break;
		//			case 1:
		//				break;
		//			default:
		//				distance += (rec[i]-'0') * pow(0.1, i - 1);
		//		}
		//	}//read distance from rplidar
		//	distance-=lidar_tf;
		//	tmp_fric_speed=calc_speed(distance,323);
		//}
		
		if(HAL_GetTick() - Latest_Remote_Control_Pack_Time >500)  //如果500ms都没有收到遥控器数据，证明遥控器可能已经离线，切换到按键控制模式。
		{   
      fric_speed = 0;
			plate_speed = 0;
			cm_speed_forw = 0;
			cm_speed_left = 0;
			cm_speed_rotate = 0;
    }
		else
		{
      if(ABS(remote_control.ch3)>100)
			{
				fric_speed = remote_control.ch3*PLATE_MAX_SPEED/660;
			}
			else
			{
				fric_speed = 0;
			}
			
			if(ABS(remote_control.ch4)>100)
			{
				plate_speed = remote_control.ch4*PLATE_MAX_SPEED/660;
			}
			else
			{
				plate_speed = 0;
			}
			
			if(remote_control.ch2>100)
			{
				pwm_pos += 0.0001;
			}
			else if(remote_control.ch2<-100)
			{
				pwm_pos -= 0.0001;
			}
			if(pwm_pos > 0.12) pwm_pos = 0.12;
			else if(pwm_pos < 0.09) pwm_pos = 0.09;
			//fric_speed = FRIC_MAX_SPEED/3;
			//plate_speed = remote_control.ch4*PLATE_MAX_SPEED/660;
			//plate_speed = 100;
			
			cm_speed_forw = 0;
			cm_speed_left = 0;
			cm_speed_rotate = 0;

			if(gm_pos > +60000) gm_pos = +60000;
			else if(gm_pos < -8000) gm_pos = -8000;
			//TODO: gm_pos = gm_pos_calc();
    }
		
		
		//计算pid目标值
		//friction wheel can_id = 1
			fric_pid[0].target =  fric_speed;
			fric_pid[1].target = 0;
		//chasis motor
		  motor_pid[0].target =   cm_speed_forw + cm_speed_left + cm_speed_rotate;
			motor_pid[1].target =   cm_speed_forw + cm_speed_left - cm_speed_rotate;
		  motor_pid[2].target = - cm_speed_forw + cm_speed_left - cm_speed_rotate;
			motor_pid[3].target = - cm_speed_forw + cm_speed_left + cm_speed_rotate;
		//gimbal motor
		  GMP_pos_pid.target = gm_pos;
		//plate driver can_id = 4
		  plate_pid.target = plate_speed;
		
		//根据设定值进行PID计算。
		for(int i=0; i<2; i++)
    {
      fric_pid[i].f_cal_pid(&fric_pid[i],moto_chassis[i].speed_rpm);
		}
		plate_pid.f_cal_pid(&plate_pid,moto_chassis[3].speed_rpm);
		GMP_pos_pid.f_cal_pid(&GMP_pos_pid,moto_chassis[2].total_angle);
    GMP_speed_pid.target = GMP_pos_pid.output/10;
		GMP_speed_pid.f_cal_pid(&GMP_speed_pid,moto_chassis[2].speed_rpm);
		
		for(int i=0; i<4; i++)
    {		
      motor_pid[i].f_cal_pid(&motor_pid[i],moto_chassis[i+4].speed_rpm);
		}
		//写电流值
    set_moto_current(&hcan1, fric_pid[0].output,   //将PID的计算结果通过CAN发送到电机
                             fric_pid[1].output,
		                         //0,plate_pid.output);
		                         //GMP_speed_pid.output,0);
		                         GMP_speed_pid.output,plate_pid.output);
		
		set_cm_current(&hcan1, motor_pid[0].output,   //将PID的计算结果通过CAN发送到电机
                           motor_pid[1].output,
                           motor_pid[2].output,
                           motor_pid[3].output);
		PWM_SetDuty(&htim8,TIM_CHANNEL_1,pwm_pos);
		
    HAL_Delay(1);      //PID控制频率                                                            Delay要注意修改
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	HAL_UART_Transmit(&huart6,(uint8_t*)rec,12,0XFFFF);
}

int32_t calc_speed(double dis,double param)
{
	int32_t result = 1200;
	if(dis<0.3)
	{result=2421;}
	else
	{result = param*sqrt(10.727*pow(dis, 2)/(dis-0.2857));}
	if(result>3000)result=3000;
	return result;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
