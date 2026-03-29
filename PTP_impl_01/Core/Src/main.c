/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PTP_SYNC        0
#define PTP_DELAY_REQ   1
#define PTP_FOLLOW_UP   8
#define PTP_DELAY_RESP  9
#define PTP_ANNOUNCE    11
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][1524];
uint8_t delay_req_frame[58]; //14 b Ethernet header + 44 b PTPv2 Delay_Req by L2


volatile uint32_t sec;
volatile uint32_t nsec;

volatile uint32_t rx_sec;
volatile uint32_t rx_nsec;

volatile uint16_t eth_type;
volatile uint8_t ptp_message;

volatile uint32_t rx_irq_count;
volatile uint32_t ptp_rx_count;

volatile uint32_t rx_hw_sec;
volatile uint32_t rx_hw_nsec;

volatile uint16_t ptp_seq_id;

volatile uint32_t follow_sec;
volatile uint32_t follow_nsec;

volatile uint32_t sync_rx_sec;
volatile uint32_t sync_rx_nsec;

volatile uint16_t sync_seq_id;

volatile int64_t ptp_offset;
volatile int32_t ptp_offset_ns;
volatile int32_t test_offset;

volatile uint8_t ptp_synced = 0;

volatile uint8_t sync_received = 0;


volatile uint16_t delayreq_seq_id = 0;
volatile uint8_t  delayreq_pending = 0;

volatile uint32_t tx_t3_sec;
volatile uint32_t tx_t3_nsec;

volatile uint32_t resp_t4_sec;
volatile uint32_t resp_t4_nsec;

volatile int64_t  mean_path_delay;
volatile int64_t  ptp_offset_e2e;

int64_t t1 = 0;
int64_t t2 = 0;
int64_t t3 = 0;
int64_t t4 = 0;

volatile uint32_t delayreq_tx_desc_idx = 0;
volatile uint8_t  delayreq_tx_ts_waiting = 0;

volatile int64_t offset_avg = 0;


volatile uint32_t delayresp_count = 0;
volatile uint32_t tx_ts_seen = 0;
volatile uint32_t check_1 = 0;
volatile uint32_t check_2 = 0;

volatile int64_t  delta_sync = 0;
volatile int64_t  delta_delay = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static uint8_t PTP_TryReadTxTimestamp(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  //SCB_DisableDCache();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  HAL_ETH_RegisterRxAllocateCallback(&heth, HAL_ETH_RxAllocateCallback);
  HAL_ETH_RegisterRxLinkCallback(&heth, HAL_ETH_RxLinkCallback);
  HAL_ETH_RegisterTxFreeCallback(&heth, HAL_ETH_TxFreeCallback);

  HAL_NVIC_EnableIRQ(ETH_IRQn);

  HAL_ETH_Start_IT(&heth);
  __HAL_ETH_DMA_ENABLE_IT(&heth, ETH_DMA_IT_T);

  /* Configure PTP clock increment */
  ETH->MACFFR |= ETH_MACFFR_RA | ETH_MACFFR_PM;

  ETH->PTPSSIR = 5;

  /* Configure timestamping */

  ETH->PTPTSCR =
        ETH_PTPTSCR_TSE
      | ETH_PTPTSCR_TSPTPPSV2E
      | ETH_PTPTSCR_TSSPTPOEFE
      | ETH_PTPTSCR_TSSEME
      | ETH_PTPTSCR_TSSSR
      | ETH_PTPTSCR_TSSARFE;

  /* Initialize timestamp */

  ETH->PTPTSHUR = 0;
  ETH->PTPTSLUR = 0;

  ETH->PTPTSCR |= ETH_PTPTSCR_TSSTI;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  sec = ETH->PTPTSHR;
	  nsec = ETH->PTPTSLR;




	  PTP_TryReadTxTimestamp();


	  HAL_ETH_ReleaseTxPacket(&heth);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  ETH->DMABMR |= ETH_DMABMR_AAB;
  ETH->DMABMR |= ETH_DMABMR_EDE;

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#define ETH_RX_BUFFER_SIZE 1524

static void PTP_BuildDelayReqFrame(uint8_t *frame, uint16_t seq_id)
{
    memset(frame, 0, 58);

    /* Ethernet header */
    frame[0]  = 0x01;
    frame[1]  = 0x1B;
    frame[2]  = 0x19;
    frame[3]  = 0x00;
    frame[4]  = 0x00;
    frame[5]  = 0x00;

    frame[6]  = 0x00;
    frame[7]  = 0x80;
    frame[8]  = 0xE1;
    frame[9]  = 0x00;
    frame[10] = 0x00;
    frame[11] = 0x00;

    frame[12] = 0x88;
    frame[13] = 0xF7;

    /* PTP header */
    frame[14] = 0x01;      /* messageType = Delay_Req */
    frame[15] = 0x02;      /* versionPTP = 2 */

    frame[16] = 0x00;
    frame[17] = 44;        /* messageLength */

    frame[18] = 0x00;      /* domainNumber */
    frame[19] = 0x00;

    frame[20] = 0x00;      /* flags[15:8] */
    frame[21] = 0x00;      /* flags[7:0] */

    /* correctionField = 0 */
    /* bytes 22..29 already zero */

    /* sourcePortIdentity: пока достаточно clockIdentity + port 1 */
    frame[34] = 0x00;
    frame[35] = 0x80;
    frame[36] = 0xE1;
    frame[37] = 0xFF;
    frame[38] = 0xFE;
    frame[39] = 0x00;
    frame[40] = 0x00;
    frame[41] = 0x00;
    frame[42] = 0x00;
    frame[43] = 0x01;

    frame[44] = (seq_id >> 8) & 0xFF;
    frame[45] = seq_id & 0xFF;

    frame[46] = 0x01;      /* controlField для Delay_Req */
    frame[47] = 0x7F;      /* logMessageInterval */

    /* bytes 48..57 originTimestamp = 0 */
}

static void PTP_SendDelayReq(void)
{
    TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM |
                          ETH_TX_PACKETS_FEATURES_CRCPAD;

    ETH_BufferTypeDef txBuffer;
    uint32_t seq = ++delayreq_seq_id;

    PTP_BuildDelayReqFrame(delay_req_frame, seq);

    txBuffer.buffer = delay_req_frame;
    txBuffer.len    = sizeof(delay_req_frame);
    txBuffer.next   = NULL;

    TxConfig.Length   = sizeof(delay_req_frame);
    TxConfig.TxBuffer = &txBuffer;

    /* Запоминаем индекс текущего TX descriptor ДО передачи */
    delayreq_tx_desc_idx = heth.TxDescList.CurTxDesc;

    ETH_DMADescTypeDef *txdesc =
            (ETH_DMADescTypeDef *)heth.TxDescList.TxDesc[delayreq_tx_desc_idx];

        uint32_t *td = (uint32_t *)txdesc;

        td[0] |= (1UL << 25);   // TDES0.TTSE = request TX timestamp for this frame


    if (HAL_ETH_Transmit_IT(&heth, &TxConfig) == HAL_OK)
    {
        delayreq_pending = 1;
        delayreq_tx_ts_waiting = 1;
    }
}

static uint8_t PTP_TryReadTxTimestamp(void)
{
    if (!delayreq_tx_ts_waiting)
        return 0;

    ETH_DMADescTypeDef *txdesc =
        (ETH_DMADescTypeDef *)heth.TxDescList.TxDesc[delayreq_tx_desc_idx];

    uint32_t *td = (uint32_t *)txdesc;

    if ((td[0] & (1UL << 31)) == 0) // OWN = 0
    {
        if (td[0] & (1UL << 17))   // TTSS
        {
            tx_t3_nsec = td[6];
            tx_t3_sec  = td[7];
            tx_ts_seen++;

            delayreq_tx_ts_waiting = 0;
            return 1;
        }
    }

    return 0;
}



static uint32_t PTP_ParseSecondsLow32(const uint8_t *ts)
{
    return ((uint32_t)ts[2] << 24) |
           ((uint32_t)ts[3] << 16) |
           ((uint32_t)ts[4] << 8)  |
           ((uint32_t)ts[5]);
}

static uint32_t PTP_ParseNanoseconds(const uint8_t *ts)
{
    return ((uint32_t)ts[6] << 24) |
           ((uint32_t)ts[7] << 16) |
           ((uint32_t)ts[8] << 8)  |
           ((uint32_t)ts[9]);
}

static int64_t PTP_ToNs(uint32_t sec, uint32_t nsec)
{
    return ((int64_t)sec * 1000000000LL) + (int64_t)nsec;
}



void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
	static uint32_t idx = 0;

	*buff = Rx_Buff[idx];

	idx++;
	if (idx >= ETH_RX_DESC_CNT)
	    idx = 0;
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t len)
{
    if (*pStart == NULL)
    {
        *pStart = buff;
    }
    else
    {
        *(uint8_t **)(*pEnd) = buff;
    }

    *pEnd = buff;
}

void HAL_ETH_TxFreeCallback(uint32_t *buff)
{
//    ETH_DMADescTypeDef *desc = (ETH_DMADescTypeDef*)buff;
//    uint32_t *d = (uint32_t*)desc;
//
//    tx_ts_seen++;
//
//    if (d[0] & (1 << 17))
//    {
//    	tx_t3_nsec = d[6];
//    	tx_t3_sec  = d[7];
//
//    }
//    else
//    {
//        return; // или игнор пакета
//    }

	(void)buff;


}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)

{
    uint8_t *buf = NULL;

    if (HAL_ETH_ReadData(heth, (void**)&buf) == HAL_OK)
    {
        rx_irq_count++;

        eth_type = ((uint16_t)buf[12] << 8) | buf[13];
        rx_sec = ETH->PTPTSHR;
        rx_nsec = ETH->PTPTSLR;

        if (eth_type == 0x88F7)
        {
            ptp_message = buf[14] & 0x0F;
            ptp_seq_id = ((uint16_t)buf[44] << 8) | buf[45];
            ptp_rx_count++;

            uint32_t idx = heth->RxDescList.RxDescIdx;

            if (idx == 0)
                idx = ETH_RX_DESC_CNT - 1;
            else
                idx--;

            ETH_DMADescTypeDef *desc =
                (ETH_DMADescTypeDef *)heth->RxDescList.RxDesc[idx];

            uint32_t *d = (uint32_t*)desc;

            if (d[0] & (1 << 17))   // timestamp valid
            {
                rx_hw_nsec = d[6];
                rx_hw_sec  = d[7];
            }

            if (ptp_message == PTP_SYNC)
            {
                sync_seq_id = ptp_seq_id;

                sync_rx_sec  = rx_hw_sec;
                sync_rx_nsec = rx_hw_nsec;

                sync_received = 1;
            }

            if (ptp_message == PTP_FOLLOW_UP)
            {
            	const uint8_t *ts = &buf[48];

            	follow_sec  = PTP_ParseSecondsLow32(ts);
            	follow_nsec = PTP_ParseNanoseconds(ts);

                if (ptp_seq_id == sync_seq_id && sync_received)
                {

                    if (ptp_synced)
                    {
//                        if (ptp_offset_ns > 50000 || ptp_offset_ns < -50000)
//                        {
//                            int32_t corr = -ptp_offset_ns;
//                            uint32_t update;
//
//                            if (corr < 0)
//                            {
//                                update = ((uint32_t)(-corr) & 0x7FFFFFFF) | (1UL << 31);
//                            }
//                            else
//                            {
//                                update = (uint32_t)corr & 0x7FFFFFFF;
//                            }
//
//                            if (!(ETH->PTPTSCR & ETH_PTPTSCR_TSSTU))
//                            {
//                                ETH->PTPTSHUR = 0;       // обязательно
//                                ETH->PTPTSLUR = update;
//                                ETH->PTPTSCR |= ETH_PTPTSCR_TSSTU;
//                            }
//                        }
                    }

                    /* ---------- INITIAL SYNC ---------- */

                    if (!ptp_synced)
                    {
                    	check_1++;
                        if (!(ETH->PTPTSCR & ETH_PTPTSCR_TSSTI))
                        {
                            ETH->PTPTSHUR = follow_sec;
                            ETH->PTPTSLUR = follow_nsec;

                            ETH->PTPTSCR |= ETH_PTPTSCR_TSSTI;

                            ptp_synced = 1;
                        }
                    }

                    if (ptp_synced && !delayreq_pending)
                    {
                    	PTP_SendDelayReq();
                    }


                }
            }



            /* ---------- PTP_DELAY_RESP ---------- */

            if (ptp_message == PTP_DELAY_RESP)
            {
                const uint8_t *ts = &buf[48];

                resp_t4_sec  = PTP_ParseSecondsLow32(ts);
                resp_t4_nsec = PTP_ParseNanoseconds(ts);

                if (ptp_seq_id == delayreq_seq_id)
                {
                    delayresp_count++;
                    delayreq_pending = 0;

                    if (tx_t3_sec == 0 && tx_t3_nsec == 0)
                    	return;

                    // calvulate timestaps
                    t1 = PTP_ToNs(follow_sec, follow_nsec);
                    t2 = PTP_ToNs(sync_rx_sec, sync_rx_nsec);

                    if (delayreq_tx_ts_waiting)
                        return;

                    t3 = PTP_ToNs(tx_t3_sec, tx_t3_nsec);
                    t4 = PTP_ToNs(resp_t4_sec, resp_t4_nsec);

                    delta_sync = t2 - t1;
                    delta_delay = t4 - t3;

                    mean_path_delay = ((t2 - t1) + (t4 - t3)) / 2;
                    ptp_offset_e2e  = ((t2 - t1) - (t4 - t3)) / 2;

                    offset_avg = (offset_avg * 3 + ptp_offset_e2e) / 4;


                    if (offset_avg > 100000LL || offset_avg < -100000LL)
                    {
                        int64_t corr = -(offset_avg / 8);
                        if (corr > 1000000LL)  corr = 1000000LL;
                        if (corr < -1000000LL) corr = -1000000LL;

                        uint32_t sec_upd = 0;
                        uint32_t nsec_upd = 0;

                        if (corr < 0)
                        {
                            int64_t mag = -corr;

                            sec_upd  = (uint32_t)(mag / 1000000000LL);
                            nsec_upd = (uint32_t)(mag % 1000000000LL);
                            nsec_upd |= (1UL << 31);   // negative update
                        }
                        else
                        {
                            sec_upd  = (uint32_t)(corr / 1000000000LL);
                            nsec_upd = (uint32_t)(corr % 1000000000LL);
                        }

                        if (!(ETH->PTPTSCR & ETH_PTPTSCR_TSSTU))
                        {
                            ETH->PTPTSHUR = sec_upd;
                            ETH->PTPTSLUR = nsec_upd;
                            ETH->PTPTSCR |= ETH_PTPTSCR_TSSTU;
                        }
                    }


                }
            }






            HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        }
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
#ifdef USE_FULL_ASSERT
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
