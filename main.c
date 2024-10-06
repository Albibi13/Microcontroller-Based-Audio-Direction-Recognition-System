/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMP2 10 //compare set for the timer for the clock of mic
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t i;
int j;
uint8_t k;
uint8_t M=16;
float right_out;
uint8_t right_out_integer;
float left_out;
uint8_t left_out_integer;
float filter[16];
uint8_t right_pcm[16];
uint8_t left_pcm[16];
float PI=3.14159265;
float fc=0.128; // fc is calculated relatively to the sampling frequency fs=1MHz/N, in our case N=32 (see next lines)
float sum;
float threshold=180;
uint8_t timeout=10;
uint8_t rx_dma_buffer[32];
uint8_t rx_dma_buffer_bin[256];
uint8_t right_buff[128];
uint8_t left_buff[128];
char msg[15]="Connected! ";
char l_msg[15]="Left ";
char r_msg[15]="Right ";
int BUFFER_SIZE=32;
int volatile correctlySent=0;
int volatile correctlyReceived=0;
int dma_flag=0;
int counter=0;
int searching=0;
int left_flag=0;
int right_flag=0;

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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim3,TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,__HAL_TIM_GET_COUNTER(&htim3)+CMP2);
  HAL_UART_Transmit_IT(&huart2,(uint8_t *)msg,sizeof(msg));
  if(correctlySent){// if the uart transmission was completed, the callback (see end of script) will set the flag
	  correctlySent=0; // reset the flag so that the next transmission flag setting is effective
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
		  //tim3_ch2 config: frequency depends on CMP2 value
		  if (__HAL_TIM_GET_FLAG(&htim3,TIM_FLAG_CC2)){
			  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,__HAL_TIM_GET_COUNTER(&htim3)+CMP2);
			  __HAL_TIM_CLEAR_FLAG(&htim3,TIM_FLAG_CC2);
		  }
		  // Receive data from the microphones through SPI
		  HAL_SPI_Receive_DMA(&hspi2,(uint8_t*)rx_dma_buffer,BUFFER_SIZE);//here i have 32 bytes = 256 bits = 128 bits per microphone collected
		  if(dma_flag){ // dma_flag is set in the callback of the DMA (DMA1_Stream3_IRQHandler), which can be found in file stm32f4xx_it.c
		  // If the buffer has received my 32 bytes, then i convert them to binary flux of values (PDM)
			  for(i=0;i<BUFFER_SIZE;i++){
				  for(j=7;j>=0;j--){// simple binary conversion from MSB to LSB
					  if(rx_dma_buffer[i]%2){
						  rx_dma_buffer_bin[8*i+j]=1;
					  }
					  else{
						  rx_dma_buffer_bin[8*i+j]=0;
					  }
					  rx_dma_buffer[i]=rx_dma_buffer[i]/2;
				  }
			  }

			  for(j=0;j<sizeof(rx_dma_buffer_bin);j++){
					  if(j%2){// separate left and right bits
							  left_buff[(j-1)/2]=(uint8_t)rx_dma_buffer_bin[j];
					  }
					  else{
							  right_buff[j/2]=(uint8_t)rx_dma_buffer_bin[j];
					  }
			  }
			  for(i=0;i<sizeof(right_pcm);i++){
				right_pcm[i]=0;// initialized to have 0:255 range; we experienced some problems when the values were negative
				left_pcm[i]=0;
				  for(j=0;j<8;j++){// convert to pcm
					  right_pcm[i]=right_pcm[i]+(right_buff[j+8*i])*pow(2,(7-j));
					  left_pcm[i]=left_pcm[i]+(left_buff[j+8*i])*pow(2,(7-j));
				  }
			  }


			  // 	FILTERING
			  //	The filter is a windowed sinc low pass filter with
			  //	cutoff freq fc and M coefficients
			  for(i=0;i<M;i++){
				  if(i-M/2==0){
					  filter[i]=2*PI*fc;//avoid 0 division
				  }
				  else{
					  filter[i]=sin(2*PI*fc*(i-M/2))/(i-M/2);
				  }
				  filter[i]=filter[i]*(0.54-0.46*cos(2*PI*i/M));//window function
			  }
			  sum=0;
			  for(i=0;i<M;i++){//sum for normalization
				  sum=sum+filter[i];
			  }

			  for(i=0;i<M;i++){//normalization
				  filter[i]=filter[i]/sum;
			  }

			  right_out=0;
			  left_out=0;
			  for(i=0;i<M;i++){//the input of the filter are M pcm values for each microphone
				  right_out=right_out+filter[i]*right_pcm[i];
				  left_out=left_out+filter[i]*left_pcm[i];
			  }

			  // the output of this stage are two float values, right out and left out
			  // that go from 0 to 255

			  // float to integer cast operation to send to the uart a lighter data flow when needed

			  right_out_integer=(int)right_out;
			  left_out_integer=(int)left_out;

			  if(right_out_integer>threshold && right_out_integer>left_out_integer){
				  if(left_flag){
					  HAL_UART_Transmit_IT(&huart2,(uint8_t *)l_msg,sizeof(l_msg));
					  left_flag=0;
					  right_flag=0;
					  searching=0;
				  }
				  else{
					  right_flag=1;
					  searching=1;
				  }
			  }
			  else if(left_out_integer>threshold && left_out_integer>right_out_integer){
				  if(right_flag){
					  HAL_UART_Transmit_IT(&huart2,(uint8_t *)r_msg,sizeof(r_msg));
					  left_flag=0;
					  right_flag=0;
					  searching=0;
				  }
				  else{
					  searching=1;
					  left_flag=1;
				  }

			  }
			  if(searching){
				  counter++;
			  }
			  if(counter==timeout){
				  searching=0;
				  counter=0;
				  right_flag=0;
				  left_flag=0;
			  }


			  //NB: uncomment lines 260 to 268 in order to plot both outputs with plot_output.m
			  //	if you want to see the live plot thanks to the python file live_plot.py you must visualize only one output, so uncomment only line 260
			  //	in both cases you need also to comment lines 220 to 253

			  /*HAL_UART_Transmit_IT(&huart2,(uint8_t*)right_out_integer,sizeof(right_out_integer));
			  if(correctlySent){
				  correctlySent=0;
				  HAL_UART_Transmit_IT(&huart2,(uint8_t*)left_out_integer,sizeof(left_out_integer));
				  if(correctlySent){
					  correctlySent=0;
				  }
			  }
			   */
			  dma_flag=0; // dma callback doesn't reset automatically the flag
			 }


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 40-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 20-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart){
	if(huart==&huart2){
		correctlySent=1;
	}
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart){
	if(huart==&huart2){
		correctlyReceived=1;
	}
}
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
