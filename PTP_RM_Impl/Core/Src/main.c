/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c SLAVE
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
    uint8_t  pending;
    uint16_t seq_id;
    uint32_t sec;
    uint32_t nsec;
} PTP_RxTimestampEvent;

typedef struct
{
    uint8_t  pending;
    uint16_t seq_id;
    uint32_t sec;
    uint32_t nsec;
} PTP_FollowUpEvent;

typedef struct
{
    uint8_t  pending;
    uint16_t seq_id;
    uint32_t sec;
    uint32_t nsec;
} PTP_DelayRespEvent;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PTP_SYNC                      0U
#define PTP_DELAY_REQ                 1U
#define PTP_FOLLOW_UP                 8U
#define PTP_DELAY_RESP                9U

#define ETH_RX_BUFFER_SIZE            1524U
#define PTP_WAIT_LOOP                 1000000UL

#define PTP_HCLK_HZ                   216000000ULL
#define PTP_SUBSEC_INCREMENT_NS       5ULL
#define PTP_ENABLE_FINE_MODE          1U

#define PTP_NS_PER_SEC                1000000000ULL
#define PTP_SUBSEC_PER_SEC            0x80000000ULL

#define PTP_OFFSET_DEADBAND_NS        50000LL
#define PTP_CORR_DIV                  4LL
#define PTP_CORR_LIMIT_NS             200000LL

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
uint8_t delay_req_frame[58];

volatile uint32_t sec;
volatile uint32_t nsec;

volatile uint8_t  ptp_synced = 0U;
volatile uint8_t  sync_received = 0U;
volatile uint8_t  delayreq_pending = 0U;
volatile uint8_t  delayreq_tx_ts_waiting = 0U;
volatile uint8_t  ptp_request_delayreq = 0U;

volatile uint16_t sync_seq_id = 0U;
volatile uint16_t delayreq_seq_id = 0U;

volatile uint32_t follow_sec = 0U;
volatile uint32_t follow_nsec = 0U;
volatile uint32_t sync_rx_sec = 0U;
volatile uint32_t sync_rx_nsec = 0U;
volatile uint32_t tx_t3_sec = 0U;
volatile uint32_t tx_t3_nsec = 0U;
volatile uint32_t resp_t4_sec = 0U;
volatile uint32_t resp_t4_nsec = 0U;

volatile int64_t t1 = 0;
volatile int64_t t2 = 0;
volatile int64_t t3 = 0;
volatile int64_t t4 = 0;

volatile int64_t delta_sync = 0;
volatile int64_t delta_delay = 0;
volatile int64_t mean_path_delay = 0;
volatile int64_t ptp_offset_e2e = 0;
volatile int64_t ptp_offset = 0;
volatile int32_t ptp_offset_ns = 0;
volatile int64_t offset_avg = 0;

volatile uint32_t delayresp_count = 0U;
volatile uint32_t tx_ts_seen = 0U;
volatile uint32_t sync_rx_ts_valid_count = 0U;

volatile uint32_t ptp_default_addend = 0U;
volatile uint32_t ptp_current_addend = 0U;
volatile uint32_t ptp_addend_update_count = 0U;
volatile uint32_t ptp_reg_timeout_count = 0U;
volatile uint32_t ptp_tx_req_fail_count = 0U;
volatile int64_t  ptp_last_update_ns = 0;

volatile PTP_RxTimestampEvent g_sync_evt = {0};
volatile PTP_FollowUpEvent    g_follow_evt = {0};
volatile PTP_DelayRespEvent   g_delayresp_evt = {0};

static uint32_t delayreq_tx_desc_idx = 0U;

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
static void PTP_HW_Init(void);
static void PTP_Process(void);
static void PTP_SendDelayReq(void);
static void PTP_BuildDelayReqFrame(uint8_t *frame, uint16_t seq_id);
static uint8_t PTP_WaitRegBitClear(volatile uint32_t *reg, uint32_t mask, uint32_t loops);
static uint32_t PTP_NsToSubsec(uint32_t ns_val);
static uint32_t PTP_UpdateLowFromNs(uint32_t ns_val, uint8_t negative);
static uint32_t PTP_CalcDefaultAddend(void);
static uint8_t PTP_ApplyAddend(uint32_t addend);
static uint8_t PTP_ApplyTimeInitialize(uint32_t sec_val, uint32_t nsec_val);
static uint8_t PTP_ApplyTimeUpdateNs(int64_t corr_ns);
static uint32_t PTP_ParseSecondsLow32(const uint8_t *ts);
static uint32_t PTP_ParseNanoseconds(const uint8_t *ts);
static int64_t  PTP_ToNs(uint32_t sec_val, uint32_t nsec_val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t PTP_WaitRegBitClear(volatile uint32_t *reg, uint32_t mask, uint32_t loops)
{
    while (((*reg) & mask) != 0U)
    {
        if (loops-- == 0U)
        {
            ptp_reg_timeout_count++;
            return 0U;
        }
    }
    return 1U;
}

static uint32_t PTP_NsToSubsec(uint32_t ns_val)
{
    uint64_t tmp;

    if (ns_val >= 1000000000UL)
    {
        ns_val = 999999999UL;
    }

    tmp = ((uint64_t)ns_val * PTP_SUBSEC_PER_SEC + (PTP_NS_PER_SEC / 2ULL)) / PTP_NS_PER_SEC;
    if (tmp > 0x7FFFFFFFULL)
    {
        tmp = 0x7FFFFFFFULL;
    }

    return (uint32_t)tmp;
}

static uint32_t PTP_UpdateLowFromNs(uint32_t ns_val, uint8_t negative)
{
    uint32_t low = PTP_NsToSubsec(ns_val) & 0x7FFFFFFFUL;

    if (negative != 0U)
    {
        low |= 0x80000000UL;
    }

    return low;
}

static uint32_t PTP_CalcDefaultAddend(void)
{
    const uint64_t num = (1ULL << 32) * 1000000000ULL;
    const uint64_t den = PTP_HCLK_HZ * PTP_SUBSEC_INCREMENT_NS;
    return (uint32_t)(num / den);
}

static uint8_t PTP_ApplyAddend(uint32_t addend)
{
    if (!PTP_WaitRegBitClear(&ETH->PTPTSCR, ETH_PTPTSCR_TSARU, PTP_WAIT_LOOP))
    {
        return 0U;
    }

    ETH->PTPTSAR = addend;
    ETH->PTPTSCR |= ETH_PTPTSCR_TSARU;

    if (!PTP_WaitRegBitClear(&ETH->PTPTSCR, ETH_PTPTSCR_TSARU, PTP_WAIT_LOOP))
    {
        return 0U;
    }

    ptp_current_addend = addend;
    ptp_addend_update_count++;
    return 1U;
}

static uint8_t PTP_ApplyTimeInitialize(uint32_t sec_val, uint32_t nsec_val)
{
    uint32_t low;

    if (!PTP_WaitRegBitClear(&ETH->PTPTSCR, ETH_PTPTSCR_TSSTI | ETH_PTPTSCR_TSSTU, PTP_WAIT_LOOP))
    {
        return 0U;
    }

    low = PTP_UpdateLowFromNs(nsec_val, 0U);

    ETH->PTPTSHUR = sec_val;
    ETH->PTPTSLUR = low;
    ptp_last_update_ns = PTP_ToNs(sec_val, nsec_val);

    ETH->PTPTSCR |= ETH_PTPTSCR_TSSTI;

    return PTP_WaitRegBitClear(&ETH->PTPTSCR, ETH_PTPTSCR_TSSTI, PTP_WAIT_LOOP);
}

static uint8_t PTP_ApplyTimeUpdateNs(int64_t corr_ns)
{
    uint32_t sec_upd;
    uint32_t nsec_mag;
    uint8_t negative;
    uint32_t low;

    if (corr_ns == 0)
    {
        return 1U;
    }

    if (!PTP_WaitRegBitClear(&ETH->PTPTSCR, ETH_PTPTSCR_TSSTI | ETH_PTPTSCR_TSSTU, PTP_WAIT_LOOP))
    {
        return 0U;
    }

    sec_upd = 0U;
    nsec_mag = 0U;
    negative = 0U;

    if (corr_ns < 0)
    {
        int64_t mag = -corr_ns;
        sec_upd  = (uint32_t)(mag / 1000000000LL);
        nsec_mag = (uint32_t)(mag % 1000000000LL);
        negative = 1U;
    }
    else
    {
        sec_upd  = (uint32_t)(corr_ns / 1000000000LL);
        nsec_mag = (uint32_t)(corr_ns % 1000000000LL);
    }

    low = PTP_UpdateLowFromNs(nsec_mag, negative);

    ETH->PTPTSHUR = sec_upd;
    ETH->PTPTSLUR = low;
    ptp_last_update_ns = corr_ns;

    ETH->PTPTSCR |= ETH_PTPTSCR_TSSTU;

    return PTP_WaitRegBitClear(&ETH->PTPTSCR, ETH_PTPTSCR_TSSTU, PTP_WAIT_LOOP);
}

static int64_t PTP_ToNs(uint32_t sec_val, uint32_t nsec_val)
{
    return ((int64_t)sec_val * 1000000000LL) + (int64_t)nsec_val;
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

static void PTP_HW_Init(void)
{
    uint32_t ptptscr = 0U;

    ETH->MACFFR |= ETH_MACFFR_RA | ETH_MACFFR_PM;
    ETH->MACIMR |= (1UL << 9);
    ETH->PTPSSIR = (uint32_t)PTP_SUBSEC_INCREMENT_NS;

    ptptscr |= ETH_PTPTSCR_TSE;
    ptptscr |= ETH_PTPTSCR_TSPTPPSV2E;
    ptptscr |= ETH_PTPTSCR_TSSPTPOEFE;
    ptptscr |= ETH_PTPTSCR_TSSSR;
    ptptscr |= ETH_PTPTSCR_TSSARFE;
#if PTP_ENABLE_FINE_MODE
    ptptscr |= ETH_PTPTSCR_TSFCU;
#endif
    ETH->PTPTSCR = ptptscr;

    ptp_default_addend = PTP_CalcDefaultAddend();
    ptp_current_addend = ptp_default_addend;
#if PTP_ENABLE_FINE_MODE
    (void)PTP_ApplyAddend(ptp_default_addend);
#endif

    (void)PTP_ApplyTimeInitialize(0U, 0U);
}

static void PTP_BuildDelayReqFrame(uint8_t *frame, uint16_t seq_id)
{
    memset(frame, 0, 58U);

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

    frame[14] = 0x01;
    frame[15] = 0x02;

    frame[16] = 0x00;
    frame[17] = 44U;

    frame[18] = 0x00;
    frame[19] = 0x00;

    frame[20] = 0x00;
    frame[21] = 0x00;

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

    frame[44] = (uint8_t)((seq_id >> 8) & 0xFFU);
    frame[45] = (uint8_t)(seq_id & 0xFFU);

    frame[46] = 0x01;
    frame[47] = 0x7F;
}

static void PTP_SendDelayReq(void)
{
    ETH_BufferTypeDef txBuffer;
    ETH_DMADescTypeDef *txdesc;
    uint32_t *td;
    uint16_t seq;

    TxConfig.Attributes   = ETH_TX_PACKETS_FEATURES_CRCPAD;
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_DISABLE;
    TxConfig.CRCPadCtrl   = ETH_CRC_PAD_INSERT;

    seq = ++delayreq_seq_id;
    PTP_BuildDelayReqFrame(delay_req_frame, seq);

    txBuffer.buffer = delay_req_frame;
    txBuffer.len    = sizeof(delay_req_frame);
    txBuffer.next   = NULL;

    TxConfig.Length   = sizeof(delay_req_frame);
    TxConfig.TxBuffer = &txBuffer;

    delayreq_tx_desc_idx = heth.TxDescList.CurTxDesc;
    txdesc = (ETH_DMADescTypeDef *)heth.TxDescList.TxDesc[delayreq_tx_desc_idx];
    td = (uint32_t *)txdesc;
    td[0] |= (1UL << 25);

    if (HAL_ETH_Transmit_IT(&heth, &TxConfig) == HAL_OK)
    {
        delayreq_pending = 1U;
        delayreq_tx_ts_waiting = 1U;
    }
    else
    {
        ptp_tx_req_fail_count++;
    }
}

static uint8_t PTP_TryReadTxTimestamp(void)
{
    ETH_DMADescTypeDef *txdesc;
    uint32_t *td;

    if (delayreq_tx_ts_waiting == 0U)
    {
        return 0U;
    }

    txdesc = (ETH_DMADescTypeDef *)heth.TxDescList.TxDesc[delayreq_tx_desc_idx];
    td = (uint32_t *)txdesc;

    if ((td[0] & (1UL << 31)) == 0U)
    {
        if ((td[0] & (1UL << 17)) != 0U)
        {
            tx_t3_nsec = td[6];
            tx_t3_sec  = td[7];
            tx_ts_seen++;
            delayreq_tx_ts_waiting = 0U;
            return 1U;
        }
    }

    return 0U;
}

static void PTP_Process(void)
{
    if (g_sync_evt.pending != 0U)
    {
        sync_seq_id   = g_sync_evt.seq_id;
        sync_rx_sec   = g_sync_evt.sec;
        sync_rx_nsec  = g_sync_evt.nsec;
        sync_received = 1U;
        g_sync_evt.pending = 0U;
    }

    if (g_follow_evt.pending != 0U)
    {
        follow_sec  = g_follow_evt.sec;
        follow_nsec = g_follow_evt.nsec;

        if ((g_follow_evt.seq_id == sync_seq_id) && (sync_received != 0U))
        {
            if (ptp_synced == 0U)
            {
                if (PTP_ApplyTimeInitialize(follow_sec, follow_nsec) != 0U)
                {
                    ptp_synced = 1U;
                }
            }

            if ((ptp_synced != 0U) && (delayreq_pending == 0U))
            {
                ptp_request_delayreq = 1U;
            }
        }

        g_follow_evt.pending = 0U;
    }

    if ((ptp_request_delayreq != 0U) && (delayreq_pending == 0U))
    {
        ptp_request_delayreq = 0U;
        PTP_SendDelayReq();
    }

    if (g_delayresp_evt.pending != 0U)
    {
        resp_t4_sec  = g_delayresp_evt.sec;
        resp_t4_nsec = g_delayresp_evt.nsec;

        if (g_delayresp_evt.seq_id == delayreq_seq_id)
        {
            delayresp_count++;
            delayreq_pending = 0U;

            if (((tx_t3_sec != 0U) || (tx_t3_nsec != 0U)) && (delayreq_tx_ts_waiting == 0U))
            {
                int64_t corr;

                t1 = PTP_ToNs(follow_sec, follow_nsec);
                t2 = PTP_ToNs(sync_rx_sec, sync_rx_nsec);
                t3 = PTP_ToNs(tx_t3_sec, tx_t3_nsec);
                t4 = PTP_ToNs(resp_t4_sec, resp_t4_nsec);

                delta_sync  = t2 - t1;
                delta_delay = t4 - t3;

                mean_path_delay = ((t2 - t1) + (t4 - t3)) / 2LL;
                ptp_offset_e2e  = ((t2 - t1) - (t4 - t3)) / 2LL;
                ptp_offset      = ptp_offset_e2e;
                ptp_offset_ns   = (int32_t)ptp_offset_e2e;

                offset_avg = (offset_avg * 3LL + ptp_offset_e2e) / 4LL;

                if ((offset_avg > PTP_OFFSET_DEADBAND_NS) ||
                    (offset_avg < -PTP_OFFSET_DEADBAND_NS))
                {
                    corr = -(offset_avg / PTP_CORR_DIV);

                    if (corr > PTP_CORR_LIMIT_NS)
                    {
                        corr = PTP_CORR_LIMIT_NS;
                    }
                    if (corr < -PTP_CORR_LIMIT_NS)
                    {
                        corr = -PTP_CORR_LIMIT_NS;
                    }

                    (void)PTP_ApplyTimeUpdateNs(corr);
                }
            }
        }

        g_delayresp_evt.pending = 0U;
    }
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

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth_local)
{
    uint8_t *buf = NULL;

    if (HAL_ETH_ReadData(heth_local, (void **)&buf) == HAL_OK)
    {
        uint16_t eth_type;
        uint8_t  ptp_message;
        uint16_t ptp_seq_id;
        uint32_t rx_sec;
        uint32_t rx_nsec;
        uint32_t idx;
        ETH_DMADescTypeDef *desc;
        uint32_t *d;
        uint8_t rx_hw_valid = 0U;
        uint32_t rx_hw_sec = 0U;
        uint32_t rx_hw_nsec = 0U;

        eth_type = ((uint16_t)buf[12] << 8) | buf[13];
        rx_sec   = ETH->PTPTSHR;
        rx_nsec  = ETH->PTPTSLR;

        if (eth_type != 0x88F7U)
        {
            return;
        }

        ptp_message = buf[14] & 0x0FU;
        ptp_seq_id  = ((uint16_t)buf[44] << 8) | buf[45];

        idx = heth_local->RxDescList.RxDescIdx;
        idx = (idx == 0U) ? (ETH_RX_DESC_CNT - 1U) : (idx - 1U);

        desc = (ETH_DMADescTypeDef *)heth_local->RxDescList.RxDesc[idx];
        d = (uint32_t *)desc;

        /* For received PTP event frames on this setup, d[6]/d[7] carry the valid RX timestamp. */
        if ((d[6] != 0U) || (d[7] != 0U))
        {
            rx_hw_nsec = d[6];
            rx_hw_sec  = d[7];
            rx_hw_valid = 1U;
        }

        if (ptp_message == PTP_SYNC)
        {
            g_sync_evt.pending = 1U;
            g_sync_evt.seq_id  = ptp_seq_id;

            if (rx_hw_valid != 0U)
            {
                g_sync_evt.sec  = rx_hw_sec;
                g_sync_evt.nsec = rx_hw_nsec;
                sync_rx_ts_valid_count++;
            }
            else
            {
                g_sync_evt.sec  = rx_sec;
                g_sync_evt.nsec = rx_nsec;
            }
        }
        else if (ptp_message == PTP_FOLLOW_UP)
        {
            const uint8_t *ts = &buf[48];

            g_follow_evt.pending = 1U;
            g_follow_evt.seq_id  = ptp_seq_id;
            g_follow_evt.sec     = PTP_ParseSecondsLow32(ts);
            g_follow_evt.nsec    = PTP_ParseNanoseconds(ts);
        }
        else if (ptp_message == PTP_DELAY_RESP)
        {
            const uint8_t *ts = &buf[48];

            g_delayresp_evt.pending = 1U;
            g_delayresp_evt.seq_id  = ptp_seq_id;
            g_delayresp_evt.sec     = PTP_ParseSecondsLow32(ts);
            g_delayresp_evt.nsec    = PTP_ParseNanoseconds(ts);
        }

        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
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

  PTP_HW_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  sec = ETH->PTPTSHR;
	  nsec = ETH->PTPTSLR;

	  (void)PTP_TryReadTxTimestamp();
	  HAL_ETH_ReleaseTxPacket(&heth);
	  PTP_Process();

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
  heth.Init.RxBuffLen = ETH_RX_BUFFER_SIZE;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  ETH->DMABMR |= ETH_DMABMR_AAB;
  ETH->DMABMR |= ETH_DMABMR_EDE;

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
	  (void)file;
	  (void)line;
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
