/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c MASTER
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
typedef struct
{
  uint16_t seq_id;
  uint32_t sec;
  uint32_t nsec;
  uint8_t  req_port_identity[10];
  uint8_t  dst_mac[6];
  uint8_t  pending;
} PTP_DelayReqEvent;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PTP_SYNC              0U
#define PTP_DELAY_REQ         1U
#define PTP_FOLLOW_UP         8U
#define PTP_DELAY_RESP        9U

#define ETH_RX_BUFFER_SIZE    1524U
#define PTP_WAIT_LOOP         1000000U
#define PTP_SYNC_PERIOD_MS    1000U
#define PTP_NS_PER_SEC        1000000000ULL
#define PTP_SUBSEC_PER_SEC    0x80000000ULL

#define PTP_FOLLOWUP_DELAY_MS   20U
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
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE];

uint8_t sync_frame[58];
uint8_t followup_frame[58];
uint8_t delayresp_frame[68];

volatile uint32_t sec;
volatile uint32_t nsec;

volatile uint32_t rx_irq_count;
volatile uint32_t ptp_rx_count;
volatile uint32_t delayreq_rx_count;
volatile uint32_t sync_tx_seen;
volatile uint32_t followup_tx_count;
volatile uint32_t delayresp_tx_count;

volatile uint16_t eth_type;
volatile uint8_t  ptp_message;
volatile uint16_t ptp_seq_id;

volatile uint16_t master_sync_seq_id;
volatile uint32_t sync_t1_sec;
volatile uint32_t sync_t1_nsec;
volatile uint8_t  sync_tx_ts_waiting;
volatile uint8_t  followup_pending;
volatile uint32_t sync_tx_desc_idx;
volatile uint32_t sync_tx_start_count;
volatile uint32_t sync_tx_fail_count;
volatile uint32_t followup_tx_fail_count;
volatile uint32_t delayresp_tx_fail_count;
volatile uint32_t tx_ts_miss_count;

volatile uint32_t ptp_default_addend = 3976821570UL;
volatile uint32_t ptp_current_addend = 3976821570UL;
volatile uint32_t ptp_addend_update_count;
volatile uint32_t ptp_reg_timeout_count;

volatile uint32_t last_sync_sent_ms;

volatile PTP_DelayReqEvent g_delayreq_evt;

static const uint8_t g_master_mac[6] = {0x00, 0x80, 0xE1, 0x00, 0x00, 0x01};
static const uint8_t g_master_port_identity[10] =
{
  0x00, 0x80, 0xE1, 0xFF, 0xFE, 0x00, 0x00, 0x01, 0x00, 0x01
};

volatile uint32_t followup_ready_ms;
volatile uint16_t followup_seq_id;
volatile uint32_t followup_t1_sec;
volatile uint32_t followup_t1_nsec;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t PTP_LoadAddend(uint32_t addend);
static uint8_t PTP_MasterTimeInit(uint32_t sec_init, uint32_t nsec_init);
static void PTP_MasterHwInit(void);
static void PTP_MasterProcess(void);
static uint8_t PTP_MasterSendSync(void);
static uint8_t PTP_MasterTryReadSyncTxTimestamp(void);
static uint8_t PTP_MasterSendFollowUp(uint16_t seq_id, uint32_t sec_t1, uint32_t nsec_t1);
static uint8_t PTP_MasterSendDelayResp(const PTP_DelayReqEvent *evt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t PTP_WaitRegBitClear(volatile uint32_t *reg, uint32_t mask, uint32_t timeout)
{
  while (timeout--)
  {
    if (((*reg) & mask) == 0U)
    {
      return 1U;
    }
  }
  return 0U;
}

static uint32_t PTP_NsToSubsecUpdate(uint32_t ns)
{
  uint64_t v;

  if (ns >= 1000000000UL)
  {
    ns = 999999999UL;
  }

  v = ((uint64_t)ns * PTP_SUBSEC_PER_SEC + (PTP_NS_PER_SEC / 2ULL)) / PTP_NS_PER_SEC;

  if (v > 0x7FFFFFFFULL)
  {
    v = 0x7FFFFFFFULL;
  }

  return (uint32_t)v;
}

static uint8_t PTP_LoadAddend(uint32_t addend)
{
  if (!PTP_WaitRegBitClear(&ETH->PTPTSCR, ETH_PTPTSCR_TSARU, PTP_WAIT_LOOP))
  {
    ptp_reg_timeout_count++;
    return 0U;
  }

  ETH->PTPTSAR = addend;
  ETH->PTPTSCR |= ETH_PTPTSCR_TSARU;

  if (!PTP_WaitRegBitClear(&ETH->PTPTSCR, ETH_PTPTSCR_TSARU, PTP_WAIT_LOOP))
  {
    ptp_reg_timeout_count++;
    return 0U;
  }

  ptp_current_addend = addend;
  ptp_addend_update_count++;
  return 1U;
}

static uint8_t PTP_MasterTimeInit(uint32_t sec_init, uint32_t nsec_init)
{
  if (!PTP_WaitRegBitClear(&ETH->PTPTSCR, ETH_PTPTSCR_TSSTI | ETH_PTPTSCR_TSSTU, PTP_WAIT_LOOP))
  {
    ptp_reg_timeout_count++;
    return 0U;
  }

  ETH->PTPTSHUR = sec_init;
  ETH->PTPTSLUR = PTP_NsToSubsecUpdate(nsec_init);
  ETH->PTPTSCR |= ETH_PTPTSCR_TSSTI;

  if (!PTP_WaitRegBitClear(&ETH->PTPTSCR, ETH_PTPTSCR_TSSTI, PTP_WAIT_LOOP))
  {
    ptp_reg_timeout_count++;
    return 0U;
  }

  return 1U;
}

static void PTP_MasterHwInit(void)
{
  ETH->DMABMR |= ETH_DMABMR_AAB;
  ETH->DMABMR |= ETH_DMABMR_EDE;

  ETH->MACFFR |= ETH_MACFFR_RA | ETH_MACFFR_PM;

  ETH->PTPSSIR = 5U;

  ETH->PTPTSCR =
      ETH_PTPTSCR_TSE |
      ETH_PTPTSCR_TSPTPPSV2E |
      ETH_PTPTSCR_TSSPTPOEFE |
      ETH_PTPTSCR_TSSEME |
      ETH_PTPTSCR_TSSSR |
      ETH_PTPTSCR_TSSARFE;

  (void)PTP_LoadAddend(ptp_default_addend);
  ETH->PTPTSCR |= ETH_PTPTSCR_TSFCU;

  (void)PTP_MasterTimeInit(1774800000UL, 0U);
}

static void PTP_StoreTimestampField(uint8_t *dst, uint32_t sec_v, uint32_t nsec_v)
{
  dst[0] = 0x00;
  dst[1] = 0x00;
  dst[2] = (uint8_t)(sec_v >> 24);
  dst[3] = (uint8_t)(sec_v >> 16);
  dst[4] = (uint8_t)(sec_v >> 8);
  dst[5] = (uint8_t)(sec_v);
  dst[6] = (uint8_t)(nsec_v >> 24);
  dst[7] = (uint8_t)(nsec_v >> 16);
  dst[8] = (uint8_t)(nsec_v >> 8);
  dst[9] = (uint8_t)(nsec_v);
}

static void PTP_FillL2Header(uint8_t *frame, const uint8_t *dst_mac, const uint8_t *src_mac)
{
  memcpy(&frame[0], dst_mac, 6U);
  memcpy(&frame[6], src_mac, 6U);
  frame[12] = 0x88;
  frame[13] = 0xF7;
}

static void PTP_BuildCommonHeader(uint8_t *frame, uint8_t msg_type, uint16_t message_length,
                                   uint16_t seq_id, uint8_t control_field, uint8_t log_interval,
                                   uint8_t two_step)
{
  frame[14] = (uint8_t)(msg_type & 0x0FU);
  frame[15] = 0x02; /* PTPv2 */

  frame[16] = (uint8_t)(message_length >> 8);
  frame[17] = (uint8_t)(message_length);

  frame[18] = 0x00; /* domainNumber */
  frame[19] = 0x00; /* reserved */

  frame[20] = (two_step != 0U) ? 0x02U : 0x00U; /* flagField[15:8], twoStepFlag = 0x0200 */
  frame[21] = 0x00U;                            /* flagField[7:0] */

  /* correctionField [22..29] = 0 */
  /* reserved [30..33] = 0 */

  memcpy(&frame[34], g_master_port_identity, 10U);

  frame[44] = (uint8_t)(seq_id >> 8);
  frame[45] = (uint8_t)(seq_id);
  frame[46] = control_field;
  frame[47] = log_interval;
}

static void PTP_BuildSyncFrame(uint8_t *frame, uint16_t seq_id)
{
  static const uint8_t ptp_multicast[6] = {0x01, 0x1B, 0x19, 0x00, 0x00, 0x00};

  memset(frame, 0, 58U);
  PTP_FillL2Header(frame, ptp_multicast, g_master_mac);
  PTP_BuildCommonHeader(frame, PTP_SYNC, 44U, seq_id, 0x00U, 0x00U, 1U);

  /* originTimestamp [48..57] kept zero for two-step Sync */
}

static void PTP_BuildFollowUpFrame(uint8_t *frame, uint16_t seq_id, uint32_t sec_t1, uint32_t nsec_t1)
{
  static const uint8_t ptp_multicast[6] = {0x01, 0x1B, 0x19, 0x00, 0x00, 0x00};

  memset(frame, 0, 58U);
  PTP_FillL2Header(frame, ptp_multicast, g_master_mac);
  PTP_BuildCommonHeader(frame, PTP_FOLLOW_UP, 44U, seq_id, 0x02U, 0x00U, 0U);
  PTP_StoreTimestampField(&frame[48], sec_t1, nsec_t1);
}

static void PTP_BuildDelayRespFrame(uint8_t *frame, const PTP_DelayReqEvent *evt)
{
  memset(frame, 0, 68U);
  PTP_FillL2Header(frame, evt->dst_mac, g_master_mac);
  PTP_BuildCommonHeader(frame, PTP_DELAY_RESP, 54U, evt->seq_id, 0x03U, 0x7FU, 0U);
  PTP_StoreTimestampField(&frame[48], evt->sec, evt->nsec);
  memcpy(&frame[58], evt->req_port_identity, 10U);
}

static uint8_t PTP_MasterSendFrame(uint8_t *frame, uint16_t length)
{
  ETH_BufferTypeDef txBuffer;

  txBuffer.buffer = frame;
  txBuffer.len    = length;
  txBuffer.next   = NULL;

  TxConfig.Attributes   = ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_DISABLE;
  TxConfig.CRCPadCtrl   = ETH_CRC_PAD_INSERT;
  TxConfig.Length       = length;
  TxConfig.TxBuffer     = &txBuffer;

  return (HAL_ETH_Transmit_IT(&heth, &TxConfig) == HAL_OK) ? 1U : 0U;
}

static uint8_t PTP_MasterSendSync(void)
{
  ETH_DMADescTypeDef *txdesc;
  uint32_t *td;
  uint16_t seq_id;

  seq_id = (uint16_t)(++master_sync_seq_id);
  PTP_BuildSyncFrame(sync_frame, seq_id);

  sync_tx_desc_idx = heth.TxDescList.CurTxDesc;
  txdesc = (ETH_DMADescTypeDef *)heth.TxDescList.TxDesc[sync_tx_desc_idx];
  td = (uint32_t *)txdesc;
  td[0] |= (1UL << 25); /* TDES0.TTSE */

  if (PTP_MasterSendFrame(sync_frame, sizeof(sync_frame)) != 0U)
  {
    sync_tx_ts_waiting = 1U;
    followup_pending   = 0U;
    sync_tx_start_count++;
    last_sync_sent_ms = HAL_GetTick();
    return 1U;
  }

  sync_tx_fail_count++;
  return 0U;
}

static uint8_t PTP_MasterTryReadSyncTxTimestamp(void)
{
  ETH_DMADescTypeDef *txdesc;
  uint32_t *td;

  if (sync_tx_ts_waiting == 0U)
  {
    return 0U;
  }

  txdesc = (ETH_DMADescTypeDef *)heth.TxDescList.TxDesc[sync_tx_desc_idx];
  td = (uint32_t *)txdesc;

  if ((td[0] & (1UL << 31)) == 0U) /* OWN = 0 */
  {
	  if ((td[0] & (1UL << 17)) != 0U) /* TTSS */
	  {
	    sync_t1_nsec = td[6];
	    sync_t1_sec  = td[7];
	    sync_tx_seen++;
	    sync_tx_ts_waiting = 0U;

	    followup_seq_id    = master_sync_seq_id;
	    followup_t1_sec    = sync_t1_sec;
	    followup_t1_nsec   = sync_t1_nsec;
	    followup_ready_ms  = HAL_GetTick() + PTP_FOLLOWUP_DELAY_MS;
	    followup_pending   = 1U;

	    return 1U;
	  }

    tx_ts_miss_count++;
    sync_tx_ts_waiting = 0U;
  }

  return 0U;
}

static uint8_t PTP_MasterSendFollowUp(uint16_t seq_id, uint32_t sec_t1, uint32_t nsec_t1)
{
  PTP_BuildFollowUpFrame(followup_frame, seq_id, sec_t1, nsec_t1);

  if (PTP_MasterSendFrame(followup_frame, sizeof(followup_frame)) != 0U)
  {
    followup_tx_count++;
    followup_pending = 0U;
    return 1U;
  }

  followup_tx_fail_count++;
  return 0U;
}

static uint8_t PTP_MasterSendDelayResp(const PTP_DelayReqEvent *evt)
{
  PTP_BuildDelayRespFrame(delayresp_frame, evt);

  if (PTP_MasterSendFrame(delayresp_frame, sizeof(delayresp_frame)) != 0U)
  {
    delayresp_tx_count++;
    return 1U;
  }

  delayresp_tx_fail_count++;
  return 0U;
}

static void PTP_MasterHandlePtpRx(ETH_HandleTypeDef *heth_ptr, uint8_t *buf)
{
  uint32_t idx;
  ETH_DMADescTypeDef *desc;
  uint32_t *d;

  ptp_message = (uint8_t)(buf[14] & 0x0FU);
  ptp_seq_id  = (uint16_t)(((uint16_t)buf[44] << 8) | buf[45]);
  ptp_rx_count++;

  idx = heth_ptr->RxDescList.RxDescIdx;
  idx = (idx == 0U) ? (ETH_RX_DESC_CNT - 1U) : (idx - 1U);

  desc = (ETH_DMADescTypeDef *)heth_ptr->RxDescList.RxDesc[idx];
  d = (uint32_t *)desc;

  if (ptp_message == PTP_DELAY_REQ)
  {
    delayreq_rx_count++;

    g_delayreq_evt.seq_id = ptp_seq_id;
    memcpy((void *)g_delayreq_evt.req_port_identity, &buf[34], 10U);
    memcpy((void *)g_delayreq_evt.dst_mac, &buf[6], 6U);

    if ((d[0] & (1UL << 17)) != 0U)
    {
      g_delayreq_evt.nsec = d[6];
      g_delayreq_evt.sec  = d[7];
    }
    else
    {
      g_delayreq_evt.sec  = ETH->PTPTSHR;
      g_delayreq_evt.nsec = ETH->PTPTSLR;
    }

    g_delayreq_evt.pending = 1U;
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
}

static void PTP_MasterProcess(void)
{
  sec = ETH->PTPTSHR;
  nsec = ETH->PTPTSLR;

  (void)PTP_MasterTryReadSyncTxTimestamp();

  if (followup_pending != 0U)
  {
	  if ((int32_t)(HAL_GetTick() - followup_ready_ms) >= 0)
	  {
	    (void)PTP_MasterSendFollowUp(followup_seq_id, followup_t1_sec, followup_t1_nsec);
	  }
  }

  if (g_delayreq_evt.pending != 0U)
  {
    if (PTP_MasterSendDelayResp((const PTP_DelayReqEvent *)&g_delayreq_evt) != 0U)
    {
      g_delayreq_evt.pending = 0U;
    }
  }

  if ((sync_tx_ts_waiting == 0U) && (followup_pending == 0U))
  {
    if ((HAL_GetTick() - last_sync_sent_ms) >= PTP_SYNC_PERIOD_MS)
    {
      (void)PTP_MasterSendSync();
    }
  }

  HAL_ETH_ReleaseTxPacket(&heth);
}

void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
  static uint32_t idx = 0U;

  *buff = Rx_Buff[idx];
  idx++;

  if (idx >= ETH_RX_DESC_CNT)
  {
    idx = 0U;
  }
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t len)
{
  (void)len;

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
  (void)buff;
}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth_ptr)
{
  uint8_t *buf = NULL;

  if (HAL_ETH_ReadData(heth_ptr, (void **)&buf) == HAL_OK)
  {
    rx_irq_count++;

    eth_type = (uint16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    if (eth_type == 0x88F7U)
    {
      PTP_MasterHandlePtpRx(heth_ptr, buf);
    }
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  HAL_ETH_RegisterRxAllocateCallback(&heth, HAL_ETH_RxAllocateCallback);
  HAL_ETH_RegisterRxLinkCallback(&heth, HAL_ETH_RxLinkCallback);
  HAL_ETH_RegisterTxFreeCallback(&heth, HAL_ETH_TxFreeCallback);

  HAL_NVIC_EnableIRQ(ETH_IRQn);

  PTP_MasterHwInit();

  if (HAL_ETH_Start_IT(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_ETH_DMA_ENABLE_IT(&heth, ETH_DMA_IT_T);

  last_sync_sent_ms = HAL_GetTick() - PTP_SYNC_PERIOD_MS;

  followup_ready_ms = 0U;
  followup_seq_id   = 0U;
  followup_t1_sec   = 0U;
  followup_t1_nsec  = 0U;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  PTP_MasterProcess();
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
  MACAddr[5] = 0x01;
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

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes   = ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_DISABLE;
  TxConfig.CRCPadCtrl   = ETH_CRC_PAD_INSERT;
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
