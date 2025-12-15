/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t keys[7];
uint16_t pwm_value = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
  if (&huart1 == huart) {
    HAL_UART_Transmit_DMA(&huart1 ,keys, sizeof(keys));
    pwm_value = 0;
    pwm_value += (keys[3]-'0')*100;
    pwm_value += (keys[4]-'0')*10;
    pwm_value += (keys[5]-'0')*1;
    if(keys[0]=='1'){
			if(pwm_value>180){}
			else{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,2000*pwm_value/180+500);//PA0
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,2000*(180-pwm_value)/180+500);//duoji(chan dou)PA1
    }}
		if(keys[0]=='2'){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,2000*pwm_value/180+500);}//duoji(zuo you)PA2
		if(keys[0]=='3'){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,2000*pwm_value/180+500);}//duoji(shang xia)PA3
		if(keys[6]=='0'){
			if(keys[0]=='0'){
			if(keys[1]=='1'){}//ru guo key == 0 jiu zheng zhuan
				else{if(keys[2]=='1'){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,2*pwm_value);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,2*pwm_value);
	  HAL_GPIO_WritePin(PIN11_GPIO_Port,PIN11_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PIN12_GPIO_Port,PIN12_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN13_GPIO_Port,PIN13_Pin,GPIO_PIN_SET);		
		HAL_GPIO_WritePin(PIN14_GPIO_Port,PIN14_Pin,GPIO_PIN_RESET);			
				}//PIN1 gao PIN2 di; PIN3 gao PIN4 di; zheng zhuan
				else{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,2*pwm_value);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,2*pwm_value);
	  HAL_GPIO_WritePin(PIN11_GPIO_Port,PIN11_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN12_GPIO_Port,PIN12_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PIN13_GPIO_Port,PIN13_Pin,GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(PIN14_GPIO_Port,PIN14_Pin,GPIO_PIN_SET);			
				}
		}//ttmada(zuo er lun zheng fan zhuan) PA6 PA7 PB0 PB1 PB2 PB10  PIN1 di PIN2 gao; PIN3 di PIN4 gao ; fanzhaun
			if(keys[1]=='1'){}
				else{if(keys[2]=='1'){
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,2*pwm_value);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,2*pwm_value);
	  HAL_GPIO_WritePin(PIN21_GPIO_Port,PIN21_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PIN22_GPIO_Port,PIN22_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN23_GPIO_Port,PIN23_Pin,GPIO_PIN_SET);		
		HAL_GPIO_WritePin(PIN24_GPIO_Port,PIN24_Pin,GPIO_PIN_RESET);	//PIN1 gao PIN2 di; PIN3 gao PIN4 di; zheng zhuan
				}
				else{
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,2*pwm_value);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,2*pwm_value);
	  HAL_GPIO_WritePin(PIN21_GPIO_Port,PIN21_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN22_GPIO_Port,PIN22_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PIN23_GPIO_Port,PIN23_Pin,GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(PIN24_GPIO_Port,PIN24_Pin,GPIO_PIN_SET);	//PIN1 di PIN2 gao; PIN3 di PIN4 gao ; fanzhaun
				}
		}}}//ttmada(you er lun zheng fan zhuan)PB6 PB7  PB3 PB4 PB8 PB9;
		if(keys[6]=='1'){ //1 zuo zhuan
		if(keys[1]=='0'){
		if(keys[2]=='1'){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,2*pwm_value-100);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,2*pwm_value-100);  //zuo er lun zheng zhuan
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,2*pwm_value);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,2*pwm_value);  //you er lun zheng zhuan
		HAL_GPIO_WritePin(PIN11_GPIO_Port,PIN11_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PIN12_GPIO_Port,PIN12_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN13_GPIO_Port,PIN13_Pin,GPIO_PIN_SET);		
		HAL_GPIO_WritePin(PIN14_GPIO_Port,PIN14_Pin,GPIO_PIN_RESET);			//zuo er lun zheng zhuan
	  HAL_GPIO_WritePin(PIN21_GPIO_Port,PIN21_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PIN22_GPIO_Port,PIN22_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN23_GPIO_Port,PIN23_Pin,GPIO_PIN_SET);		
		HAL_GPIO_WritePin(PIN24_GPIO_Port,PIN24_Pin,GPIO_PIN_RESET); //you er lun zheng zhuan
		}}}//xiao che zuo zhuan
		if(keys[6]=='2'){//2 you zhuan
		if(keys[1]=='0'){
		if(keys[2]=='1'){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,2*pwm_value);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,2*pwm_value);//zuo er lun zheng zhuan
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,2*pwm_value-100);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,2*pwm_value-100);//you er lun zheng zhuan
		HAL_GPIO_WritePin(PIN11_GPIO_Port,PIN11_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PIN12_GPIO_Port,PIN12_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN13_GPIO_Port,PIN13_Pin,GPIO_PIN_SET);		
		HAL_GPIO_WritePin(PIN14_GPIO_Port,PIN14_Pin,GPIO_PIN_RESET);			//zuo er lun zheng zhuan
	  HAL_GPIO_WritePin(PIN21_GPIO_Port,PIN21_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PIN22_GPIO_Port,PIN22_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN23_GPIO_Port,PIN23_Pin,GPIO_PIN_SET);		
		HAL_GPIO_WritePin(PIN24_GPIO_Port,PIN24_Pin,GPIO_PIN_RESET); //you er lun zheng zhuan
		}}}//xiao che you zhuan
		if(keys[1]=='0' && keys[2]=='0' && keys[0]=='0' && keys[3]=='0' && keys[4]=='0' && keys[5]=='0' && keys[6]=='0'){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,0);
		HAL_GPIO_WritePin(PIN11_GPIO_Port,PIN11_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN12_GPIO_Port,PIN12_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN13_GPIO_Port,PIN13_Pin,GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(PIN14_GPIO_Port,PIN14_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(PIN21_GPIO_Port,PIN21_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN22_GPIO_Port,PIN22_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN23_GPIO_Port,PIN23_Pin,GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(PIN24_GPIO_Port,PIN24_Pin,GPIO_PIN_RESET); 
		}
		if(keys[1]=='4' && keys[2]=='4' && keys[0]=='4' && keys[3]=='4' && keys[4]=='4' && keys[5]=='4' && keys[6]=='4'){
			HAL_GPIO_WritePin(DRIVE1_GPIO_Port,DRIVE1_Pin,GPIO_PIN_SET);}
		if(keys[1]=='5' && keys[2]=='5' && keys[0]=='5' && keys[3]=='5' && keys[4]=='5' && keys[5]=='5' && keys[6]=='5'){
			HAL_GPIO_WritePin(DRIVE1_GPIO_Port,DRIVE1_Pin,GPIO_PIN_RESET);}//1 PA8;(FA SHE)
    HAL_UART_Receive_DMA(&huart1, keys, sizeof(keys));
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,1500);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,1500);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1500);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,1500);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,0);
	HAL_GPIO_WritePin(DRIVE1_GPIO_Port,DRIVE1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PIN11_GPIO_Port,PIN11_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PIN12_GPIO_Port,PIN12_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PIN13_GPIO_Port,PIN13_Pin,GPIO_PIN_RESET);		
	HAL_GPIO_WritePin(PIN14_GPIO_Port,PIN14_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PIN21_GPIO_Port,PIN21_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PIN22_GPIO_Port,PIN22_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PIN23_GPIO_Port,PIN23_Pin,GPIO_PIN_RESET);		
	HAL_GPIO_WritePin(PIN24_GPIO_Port,PIN24_Pin,GPIO_PIN_RESET); 
  HAL_UART_Receive_DMA(&huart1, keys, sizeof(keys));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
