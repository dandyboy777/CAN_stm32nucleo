/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "stack_pr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS232_BAUD 512000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {0,};
uint8_t RxData[8] = {0,};
uint32_t TxMailbox = 0;
char trans_str[100];
uint16_t trans_len=0;
uint8_t rx_buff[20];
uint16_t rx_len=0;
uint16_t speen_indx=0;

struct speed_
{
  char name[15];
  int prescaler; 
  uint32_t TimeSeg1;
  uint32_t TimeSeg2;
  
}speed_settings[]={


{"10kbit/s",500,CAN_BS1_13TQ,CAN_BS2_2TQ},
{"20kbit/s",250,CAN_BS1_13TQ,CAN_BS2_2TQ},
{"31.25kbit/s",160,CAN_BS1_13TQ,CAN_BS2_2TQ},
{"33kbit/s",303,CAN_BS1_6TQ,CAN_BS2_1TQ},
{"40kbit/s",125,CAN_BS1_13TQ,CAN_BS2_2TQ},
{"50kbit/s",100,CAN_BS1_13TQ,CAN_BS2_2TQ},
{"80kbit/s",100,CAN_BS1_8TQ,CAN_BS2_1TQ},
{"83.33kbit/s",60,CAN_BS1_13TQ,CAN_BS2_2TQ},
{"100kbit/s",50,CAN_BS1_13TQ,CAN_BS2_2TQ},
{"125kbit/s",40,CAN_BS1_13TQ,CAN_BS2_2TQ},
{"250kbit/s",20,CAN_BS1_13TQ,CAN_BS2_2TQ},
{"500kbit/s",10,CAN_BS1_13TQ,CAN_BS2_2TQ},
{"800kbit/s",10,CAN_BS1_8TQ,CAN_BS2_1TQ},
{"1mbit/s",5,CAN_BS1_13TQ,CAN_BS2_2TQ},
{"10mbit/s",1,CAN_BS1_6TQ,CAN_BS2_1TQ}

};

#define CANHACKER_SERIAL_RESPONSE     "N0001\r"
#define CANHACKER_SW_VERSION_RESPONSE "v0107\r"
#define CANHACKER_VERSION_RESPONSE    "V1010\r"

char stack_hostToMK[100*8];
unsigned int stack_in;
unsigned int stak_out;
t_stack HostToMK;
//t_stack RsToModem;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void PrintCANRxMessage(CAN_RxHeaderTypeDef *RxHeader_loc, uint8_t *RxData_loc)
{
  int len=0,i;

  
  len+=snprintf(trans_str+len,sizeof(trans_str)-len,"%08d: ", HAL_GetTick());
  len+=snprintf(trans_str+len,sizeof(trans_str)-len,"0x%x: ",RxHeader_loc->StdId);
  for(i=0;i<RxHeader_loc->DLC;i++)
   len+=snprintf(trans_str+len,sizeof(trans_str)-len,"%02X ",RxData_loc[i]);
  if(RxHeader_loc->DLC)
  {
     len+=snprintf(trans_str+len,sizeof(trans_str)-len,"\r\n");
  }
  HAL_UART_Transmit(&huart4, (uint8_t*)trans_str, len, 10);
  
   // stk_push_mas(&HostToMK,(u8*)trans_str,strlen(trans_str));

  
//  HAL_UART_Transmit(&huart4, (uint8_t*)RxHeader_loc, sizeof(CAN_RxHeaderTypeDef), 100);
//  HAL_UART_Transmit(&huart4, (uint8_t*)RxData_loc,RxHeader_loc->DLC , 100);
  
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      
      
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); 
        PrintCANRxMessage(&RxHeader, RxData);
          

        //stk_push_b(&ModemToRs, ch);
         // stk_push_mas(hstack, ptr, len);
//          //stk_pop_b(&ModemToRs, &ch) 
//          stk_init(&RsToModem, (void*) g_stk_stasks, sizeof(g_stk_stasks));
     }
      
}



void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);  
    
    PrintCANRxMessage(&RxHeader, RxData);
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t er = HAL_CAN_GetError(hcan);
  //  snprintf(trans_str,sizeof(trans_str),"ER CAN %lu %08lX\r\n", er, er);
   // HAL_UART_Transmit(&huart4,".", 1, 100);
   // HAL_Delay(10);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t num_bytes=0;
  char tx[10],tx_transm,fl=0;
  
  trans_str[trans_len++]=rx_buff[rx_len];
    rx_len++;
  rx_buff[rx_len]=0;
  if(rx_buff[rx_len-1]=='\r')
  {
//  stk_push_b(&HostToMK,rx_buff[0]);
 
    switch(rx_buff[0])
      {
      case 'V':
       tx_transm = snprintf((char*)tx,sizeof(tx),CANHACKER_VERSION_RESPONSE);
      break;

      case 'v':
       tx_transm = snprintf((char*)tx,sizeof(tx),CANHACKER_SW_VERSION_RESPONSE);
      break;
//      case 0x43://'C':
//       tx_transm = snprintf((char*)tx,sizeof(tx),"\x07\r");
//      break;

      default:
        tx_transm = snprintf((char*)tx,sizeof(tx),"\r");
        fl=1;
      break;
      }
    
     HAL_UART_Transmit(huart, (uint8_t*)tx, tx_transm, 1);
    if(fl)
    {
        HAL_UART_Transmit(huart, "17FF80101010101010101\r", 22, 1);
      
    }
//   stk_push_mas(&HostToMK,rx_buff,num_bytes);
   memcpy(trans_str+trans_len,tx,tx_transm);
   trans_len+=tx_transm;
   rx_len=0;
  }
   HAL_UART_Receive_IT(&huart4, rx_buff+rx_len, 1); //You need to toggle a breakpoint on this line!
  
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13) {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  } else {
      __NOP();
  }
  

  if(speen_indx+1>=(sizeof(speed_settings)/sizeof(speed_settings[0])))
    speen_indx=0;
  else
      speen_indx++;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
   uint16_t buf_inp_len;
    char buf[50];
   uint16_t speed_loc=-1;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
   stk_init(&HostToMK,stack_hostToMK,sizeof(stack_hostToMK));
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  
  /* USER CODE BEGIN 2 */
  TxHeader.StdId = 0x0378;
  TxHeader.ExtId = 0x0378;
  TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
  TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = 0;

  for(uint8_t i = 0; i < 8; i++)
  {
      TxData[i] = (i + 10);
  }
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT (&huart4, rx_buff, 1);
  rx_len=0;
  
  while (1)
  {
//          while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
//
//          if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//          {
//                  HAL_UART_Transmit(&huart4, (uint8_t*)"ER SEND\n", 8, 100);
//          }
          
   buf_inp_len=stk_get_len(&HostToMK);
   while(buf_inp_len>0)
   {
     buf_inp_len=stk_pop_mas(&HostToMK,buf,sizeof(buf));    
     if(buf_inp_len>0)
       HAL_UART_Transmit(&huart4, (uint8_t*)buf, buf_inp_len, 500);
     else 
       break;
                     
   }
   if(speed_loc!=speen_indx)
   {
     speed_loc=speen_indx;
     MX_CAN1_Init();
   }
      
//    HAL_Delay(1);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  CAN_FilterTypeDef  sFilterConfig;
  HAL_CAN_Stop(&hcan1);
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 500;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  
   hcan1.Init.Prescaler = speed_settings[speen_indx].prescaler;
   hcan1.Init.TimeSeg1 =  speed_settings[speen_indx].TimeSeg1;
   hcan1.Init.TimeSeg2 =  speed_settings[speen_indx].TimeSeg2;
   
   
     hcan1.Instance = CAN1;

  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  
  
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; 
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  //sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  {
    char tmp_buf[50];
    int buf_len=0;
    snprintf(tmp_buf,sizeof(tmp_buf),"\r\nSet speed %s\r\n",speed_settings[speen_indx].name);
    stk_push_mas(&HostToMK,tmp_buf,strlen(tmp_buf));
    
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
  }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */
  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */
  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = RS232_BAUD;// 115200;// 256000;//9600;//115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
