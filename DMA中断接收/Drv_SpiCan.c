/**
  ******************************************************************************
  * @file    Drv_SpiCan.c
  * @author  YZH
  * @brief   spi 转 CAN 设备.
  *
  @verbatim
  ==============================================================================
                     ##### 芯片介绍 #####
  ==============================================================================
[..]
   (+) 芯片型号   ： MCP2517FDT-H_SL
   (+) 内存       ： 2KByte RAM  31FIFO 可配置为发送或者接收缓存区  
                                 1个发送队列 1个事件发送
   (+) 工作电压   ： 2.7~5.5V
   (+) 工作温度   ：-40~150℃
   (+) 外部晶振频率 ： 20MHZ
   (+) 通信方式  ： SPI 模式0或模式3
   (+) 时钟频率  ： 最高20MHZ 
   (+) 工作方式  ：支持 CAN FD 和 CAN2.0B混合模式
                   支持 CAN2.OB 模式
  @endverbatim
  ******************************************************************************
  * @attention
   <pre>
         (+) SPI 时钟不准确  (SPI5 SPI6)时钟按驱动配置直可配置到3.25MHZ
         (+) 循环接收占用CPU利用率过高
         (+) 不采用中断接收 CPU利用率并没有明显的改善，并且 中断引脚 偶尔不响应
    </pre>
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Drv_SpiCan.h"
#include "Drv_Spi.h"
#include "drv_canfdspi_api.h"
#include <string.h>

/** @addtogroup 设备驱动
  * @{
  */
/** @defgroup SPI转CAN驱动
  * @{
  */

/* Private types -------------------------------------------------------------*/
 /** @defgroup SPICAN内部枚举与结构体类型
  * @{
  */ 
   
  /**
  * @brief SPICAN 接收类型
  */ 

typedef struct
{

  CAN_RX_MSGOBJ rxObj;                /*!<  接收消息类型*/ 
  uint8_t buf[SPICAN_DATA_MAXSIZE];   /*!<  接收缓存区*/
} SPICAN_RX_OBJ;

/**
 * @brief  SPICAN 接收缓存区类型
 */
typedef struct
{
  uint8_t head;                               /*!<  接收帧头*/ 
  uint8_t tail;                               /*!<  接收帧尾*/ 
  uint8_t overflow;                           /*!<  接收溢出标志位*/ 
  uint8_t count;                              /*!<  接收计数*/ 
  SPICAN_RX_OBJ spi_rx_obj[SPICAN_FIFO_SIZE]; /*!< SPICAN 接收类型*/
} SPICAN_RX_FIFO;

/**
 * @brief SPICAN 发送类型
 */
typedef struct
{

  CAN_TX_MSGOBJ txObj;               /*!<  发送消息类型*/
  uint8_t buf[SPICAN_DATA_MAXSIZE];  /*!<  发送缓存区*/
} SPICAN_TX_OBJ;

/**
 * @brief SPICAN 发送缓存区类型
 */
typedef struct
{
  uint8_t head;                              /*!<  发送帧头*/ 
  uint8_t tail;                              /*!<  发送帧尾*/  
  uint8_t overflow;                          /*!<  发送溢出标志位*/ 
  uint8_t count;                             /*!<  发送计数*/ 
  SPICAN_TX_OBJ spi_tx_obj[SPICAN_FIFO_SIZE]; /*!< SPICAN 发送类型*/
} SPICAN_TX_FIFO;

/**
 * @}
 */
/* Private macro -------------------------------------------------------------*/

/** @defgroup SPICAN内部宏
 * @{
 */

#define SPICAN3_INT_CLK_ENABLE() __HAL_RCC_GPIOI_CLK_ENABLE()
#define SPICAN3_INT_PORT GPIOI
#define SPICAN3_RXINT_PIN GPIO_PIN_15

#define SPICAN4_INT_CLK_ENABLE() __HAL_RCC_GPIOI_CLK_ENABLE()
#define SPICAN4_INT_PORT GPIOI
#define SPICAN4_RXINT_PIN GPIO_PIN_11
#define SPICAN34_INT_IRQHandler EXTI15_10_IRQHandler
#define SPICAN34_INT_IRQn EXTI15_10_IRQn

#define SPICAN5_INT_CLK_ENABLE() __HAL_RCC_GPIOH_CLK_ENABLE()
#define SPICAN5_INT_PORT GPIOH
#define SPICAN5_RXINT_PIN GPIO_PIN_2
#define SPICAN5_INT_IRQHandler EXTI2_IRQHandler
#define SPICAN5_INT_IRQn EXTI2_IRQn

/**
 * @}
 */
/* Private variables ---------------------------------------------------------*/
SPICAN_RX_FIFO spican_rx_fifo[SPICANMAX] = {0}; /*!< SPICAN接收缓存区*/ 
SPICAN_TX_FIFO spican_tx_fifo[SPICANMAX] = {0}; /*!< SPICAN发送缓存区*/ 

/* Private function ---------------------------------------------------------*/
/** @addtogroup SPICAN 静态函数
  * @{
  */

/**
  * @brief  SPICAN 发送一次.
  * @param[in]  whichdev:  SPICAN设备
  *          @arg @ref SPICAN3  CAN3
  *          @arg @ref SPICAN4  CAN4
  *          @arg @ref SPICAN5  CAN5
  * @param[in]  txObj :  发送结构体
  * @param[in]  buf :  需要发送缓存区地址
  * @param[in]  len :  需要发送字节长度
  * @retval 实际发送的字节数
  */
static uint8_t spican_outonce(uint8_t whichdev, CAN_TX_MSGOBJ *txObj, uint8_t *buf, uint8_t len)
{
  uint8_t retlen = 0;
  uint8_t attempts = 20;
  uint32_t dlctobyte = 0;
  CAN_TX_FIFO_EVENT txFlags = CAN_TX_FIFO_NO_EVENT;
  do
  {
    DRV_CANFDSPI_TransmitChannelEventGet(whichdev, CAN_FIFO_CH2, &txFlags);
    if (attempts == 0)
    {
      Nop();
      Nop();
      return 0;
    }
    attempts--;
  } while (!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT));

  // ???????
  dlctobyte = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)txObj->bF.ctrl.DLC);
  if (DRV_CANFDSPI_TransmitChannelLoad(whichdev, CAN_FIFO_CH2, txObj, buf, dlctobyte, true) != 0)
  {
    retlen = 0;
  }
  else
  {
    retlen = len;
  }
  return retlen;
}

/** @addtogroup SPICAN外部函数
  * @{
  */
/**
  * @brief  SPICAN 初始化.
  * @param[in]  whichdev:  SPICAN设备
  *          @arg @ref SPICAN3  CAN3
  *          @arg @ref SPICAN4  CAN4
  *          @arg @ref SPICAN5  CAN5
  * @param[in]  arbitr_bittime :  仲裁位波特率
  * @param[in]  data_bittime :  数据位波特率 如果接收或者发送采用 CAN2.0 此位无效 
  * @param[in]  plsize : FIFO 负载长度大小
  *          @arg @ref PLSIZE_8  CAN3
  *          @arg @ref PLSIZE_12  CAN4
  *          @arg @ref PLSIZE_16  CAN5
  *          @arg @ref PLSIZE_20  CAN3
  *          @arg @ref PLSIZE_24  CAN4
  *          @arg @ref PLSIZE_32  CAN5
  *          @arg @ref PLSIZE_48  CAN3
  *          @arg @ref PLSIZE_64  CAN4
  * @retval 0:成功  
  * @retval -1:失败
  */
int8_t Spican_Init(uint8_t whichdev, uint8_t arbitr_bittime, uint8_t data_bittime, uint8_t plsize)
{
  int8_t ret = 0;
  CAN_CONFIG config = {0};

  CAN_TX_FIFO_CONFIG txConfig = {0};

  CAN_RX_FIFO_CONFIG rxConfig = {0};

  REG_CiFLTOBJ fObj = {0};

  REG_CiMASK mObj = {0};

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  switch (whichdev)
  {
  case SPICAN3:
    ret = SPI_Init(SPI_CH3, SPI_BAUDRATE_12_5M);
    if (ret != RET_OK)
    {
      return -1;
    }
    SPICAN3_INT_CLK_ENABLE();
    GPIO_InitStruct.Pin = SPICAN3_RXINT_PIN;
    HAL_GPIO_Init(SPICAN3_INT_PORT, &GPIO_InitStruct);
    //  HAL_NVIC_SetPriority(SPICAN34_INT_IRQn, 1, 1);
    //  HAL_NVIC_EnableIRQ(SPICAN34_INT_IRQn);
    break;
  case SPICAN4:
    ret = SPI_Init(SPI_CH5, SPI_BAUDRATE_3_125M);
    if (ret != RET_OK)
    {
      return -1;
    }
    SPICAN4_INT_CLK_ENABLE();
    GPIO_InitStruct.Pin = SPICAN4_RXINT_PIN;
    HAL_GPIO_Init(SPICAN4_INT_PORT, &GPIO_InitStruct);
    //  HAL_NVIC_SetPriority(SPICAN34_INT_IRQn, 0, 3);
    //  HAL_NVIC_EnableIRQ(SPICAN34_INT_IRQn);
    break;
  case SPICAN5:
    ret = SPI_Init(SPI_CH6, SPI_BAUDRATE_3_125M);
    if (ret != RET_OK)
    {
      return -1;
    }
    SPICAN5_INT_CLK_ENABLE();
    GPIO_InitStruct.Pin = SPICAN5_RXINT_PIN;
    HAL_GPIO_Init(SPICAN5_INT_PORT, &GPIO_InitStruct);
    // HAL_NVIC_SetPriority(SPICAN5_INT_IRQn, 1, 1);
    // HAL_NVIC_EnableIRQ(SPICAN5_INT_IRQn);
    break;
  default:

    break;
  }
  // ??λ?豸
  DRV_CANFDSPI_Reset(whichdev);

  // ??????У?
  DRV_CANFDSPI_EccEnable(whichdev);

  // ??λ RAM
  DRV_CANFDSPI_RamInit(whichdev, 0xff);

  // ??λ CAN????
  DRV_CANFDSPI_ConfigureObjectReset(&config);

  config.IsoCrcEnable = 0;
  config.StoreInTEF = 0;

  // ???????????
  DRV_CANFDSPI_Configure(whichdev, &config);

  DRV_CANFDSPI_TransmitBandWidthSharingSet(whichdev, CAN_TXBWS_16);

  // ??λ????FIFO????   ???÷???FIFO 8??????λ
  DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
  txConfig.FifoSize = 10;        // 8;
  txConfig.PayLoadSize = plsize; // plsize;(64+8) * 10 = 720byte
  txConfig.TxPriority = 1;
  DRV_CANFDSPI_TransmitChannelConfigure(whichdev, CAN_FIFO_CH2, &txConfig);

  // ??λ????FIFO????  ???????FIFO 8??????λ
  DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
  rxConfig.FifoSize = 10;        // 20;
  rxConfig.PayLoadSize = plsize; // plsize; (64+12) * 10 = 760byte
  DRV_CANFDSPI_ReceiveChannelConfigure(whichdev, CAN_FIFO_CH1, &rxConfig);

  DRV_CANFDSPI_FilterObjectConfigure(whichdev, CAN_FILTER0, &fObj.bF);
  DRV_CANFDSPI_FilterMaskConfigure(whichdev, CAN_FILTER0, &mObj.bF);

  DRV_CANFDSPI_FilterToFifoLink(whichdev, CAN_FILTER0, CAN_FIFO_CH1, true);

  DRV_CANFDSPI_BitTimeConfigureNominal20MHz(whichdev, (CAN_BITTIME_SETUP)arbitr_bittime);

  DRV_CANFDSPI_BitTimeConfigureData20MHz(whichdev, (CAN_BITTIME_SETUP)data_bittime, CAN_SSP_MODE_AUTO);

  DRV_CANFDSPI_ReceiveChannelEventEnable(whichdev, CAN_FIFO_CH1, CAN_RX_FIFO_NOT_EMPTY_EVENT);
  DRV_CANFDSPI_TransmitChannelEventEnable(whichdev, CAN_FIFO_CH2, CAN_TX_FIFO_EMPTY_EVENT);

  DRV_CANFDSPI_ModuleEventEnable(whichdev, (CAN_MODULE_EVENT)(CAN_TX_EVENT | CAN_RX_EVENT));
  DRV_CANFDSPI_GpioModeConfigure(whichdev, GPIO_MODE_INT, GPIO_MODE_INT);
  DRV_CANFDSPI_GpioInterruptPinsOpenDrainConfigure(whichdev, GPIO_PUSH_PULL);

  // ??????????
  DRV_CANFDSPI_OperationModeSelect(whichdev, CAN_NORMAL_MODE);

  return ret;
}

/**
  * @brief  SPICAN 设置滤波.
  * @param[in]  whichdev:  SPICAN设备
  * @param[in]  sid :  滤波标准ID
  * @param[in]  sidmask :  滤波标准帧掩码
  * @param[in]  eid :  扩展帧ID
  * @param[in]  eidmask :  扩展帧掩码
  */
void Spican_Setfliter(uint8_t whichdev, uint32_t sid, uint32_t sidmask, uint32_t eid, uint32_t eidmask)
{
  REG_CiFLTOBJ fObj = {0};
  REG_CiMASK mObj = {0};

  fObj.bF.SID = sid;
  fObj.bF.EXIDE = 0;
  mObj.bF.MSID = sidmask;
  mObj.bF.MEID = 1;
  DRV_CANFDSPI_FilterObjectConfigure(whichdev, CAN_FILTER0, &fObj.bF);
  DRV_CANFDSPI_FilterMaskConfigure(whichdev, CAN_FILTER0, &mObj.bF);

  // ???? FIFO ?? ?????
  DRV_CANFDSPI_FilterToFifoLink(whichdev, CAN_FILTER0, CAN_FIFO_CH1, true);

  fObj.bF.EID = eid;
  fObj.bF.EXIDE = 1;
  mObj.bF.MSID = eidmask;
  mObj.bF.MEID = 1;
  DRV_CANFDSPI_FilterObjectConfigure(whichdev, CAN_FILTER1, &fObj.bF);
  DRV_CANFDSPI_FilterMaskConfigure(whichdev, CAN_FILTER1, &mObj.bF);

  // ???? FIFO ?? ?????
  DRV_CANFDSPI_FilterToFifoLink(whichdev, CAN_FILTER1, CAN_FIFO_CH1, true);
}

/**
  * @brief  SPICAN 轮询接收
  * @param[in]  whichdev:  SPICAN设备
  *          @arg @ref SPICAN3  CAN3
  *          @arg @ref SPICAN4  CAN4
  *          @arg @ref SPICAN5  CAN5
  * @note 放在线程中使用 （3个CPU占用率16%）  
  */
void Spican_Rx_Loop(uint8_t whichdev)
{
  uint8_t attempts = 20;
  CAN_RX_FIFO_EVENT rxFlags = CAN_RX_FIFO_NO_EVENT;
  CAN_RX_MSGOBJ rxObj = {0};               /*!<  接收消息类型*/ 
  uint8_t buf[SPICAN_DATA_MAXSIZE] = {0};  /*!<  接收消息类型*/ 
  DRV_CANFDSPI_ReceiveChannelEventGet(whichdev, CAN_FIFO_CH1, &rxFlags);
  while ((rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) && attempts > 0)
  {
    // if(rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT){
    if (DRV_CANFDSPI_ReceiveMessageGet(whichdev, CAN_FIFO_CH1, &rxObj, &buf[0], 64) == 0)
    {
      spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].head].rxObj = rxObj;
      memcpy(&spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].head].buf[0], buf, SPICAN_DATA_MAXSIZE);
      spican_rx_fifo[whichdev].head++;
      spican_rx_fifo[whichdev].head %= SPICAN_FIFO_SIZE;
      if (spican_rx_fifo[whichdev].head == spican_rx_fifo[whichdev].tail)
      {
        spican_rx_fifo[whichdev].overflow = 1;
      }
    }
    DRV_CANFDSPI_ReceiveChannelEventGet(whichdev, CAN_FIFO_CH1, &rxFlags);
    attempts--;
  }
}

/**
  * @brief  SPICAN 缓存区接收包数量.
  * @param[in]  whichdev:  SPICAN设备
  *          @arg @ref SPICAN3  CAN3
  *          @arg @ref SPICAN4  CAN4
  *          @arg @ref SPICAN5  CAN5
  * @retval 缓存区接收包数量
  */
uint8_t Spican_Rx_Packetlen(uint8_t whichdev)
{
  uint8_t packetlen = 0;
  packetlen = ((spican_rx_fifo[whichdev].head - spican_rx_fifo[whichdev].tail) + SPICAN_FIFO_SIZE) % SPICAN_FIFO_SIZE;
  return packetlen;
}

/**
  * @brief  SPICAN 缓存区接收包是否溢出.
  * @retval 1 - 溢出 0 - 没有溢出 
  */
uint8_t Spican_Rx_Packet_Isoverflow(uint8_t whichdev)
{
  return spican_rx_fifo[whichdev].overflow;
}

/**
  * @brief  SPICAN 接收.
  * @param[in]  whichdev:  SPICAN设备
  *          @arg @ref SPICAN3  CAN3
  *          @arg @ref SPICAN4  CAN4
  *          @arg @ref SPICAN5  CAN5
  * @param[out]  id :  CAN id
  * @param[out]  type_id :  id 类型    
  *          @arg @ref SID  标准帧
  *          @arg @ref EID  扩展帧
  * @param[out]  buf :  接收缓存区地址
  * @param[in]  len :  需要接收字节长度 小于等于64
  * @param[out]  iscanfd :  是否为 CAN FD 类型  1-是 0-否
  * @retval 实际接收的字节数
  */
uint8_t Spican_In(uint8_t whichdev, uint32_t *id, uint8_t *type_id, uint8_t *buf, uint8_t len, uint8_t *iscanfd)
{
  uint8_t index = 0;
  uint8_t retlen = 0;
  uint32_t dlctobyte = 0;
  if (Spican_Rx_Packetlen(whichdev) > 0)
  {
    *iscanfd = spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].tail].rxObj.bF.ctrl.FDF;
    *type_id = spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].tail].rxObj.bF.ctrl.IDE;
    if (*type_id == EID)
    {
      *id = spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].tail].rxObj.bF.id.EID;
    }
    else
    {
      *id = spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].tail].rxObj.bF.id.SID;
    }
    dlctobyte = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].tail].rxObj.bF.ctrl.DLC);
    retlen = (dlctobyte <= len) ? dlctobyte : len;
    for (index = 0; index < retlen; index++)
    {
      buf[index] = spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].tail].buf[index];
    }
    spican_rx_fifo[whichdev].tail++;
    spican_rx_fifo[whichdev].tail %= SPICAN_FIFO_SIZE;
  }
  return retlen;
}

/**
  * @brief  SPICAN 发送.
  * @param[in]  whichdev:  SPICAN设备
  *          @arg @ref SPICAN3  CAN3
  *          @arg @ref SPICAN4  CAN4
  *          @arg @ref SPICAN5  CAN5
  * @param[in]  id :  CAN id
  * @param[in]  type_id :  id 类型    
  *          @arg @ref SID  标准帧
  *          @arg @ref EID  扩展帧
  * @param[in]  buf :  需要发送缓存区地址
  * @param[in]  len :  需要发送字节长度
  * @param[in]  iscanfd :  是否为 CAN FD 类型  1-是 0-否
  * @retval 实际发送的字节数
  */
uint16_t Spican_Out(uint8_t whichdev, uint32_t id, uint8_t type_id, uint8_t *buf, uint16_t len, uint8_t iscanfd)
{
  uint16_t retlen = 0;
  uint8_t txlen = 0;
  CAN_TX_MSGOBJ txObj = {0};
  if (type_id == SID)
  {
    txObj.bF.id.SID = id;
  }
  else if (type_id == EID)
  {
    txObj.bF.id.EID = id;
  }
  txObj.bF.ctrl.IDE = type_id;
  txObj.bF.ctrl.FDF = iscanfd;
  txObj.bF.ctrl.BRS = iscanfd;

  while (len > 0)
  {
    if (iscanfd == 0)
    {
      if (len / 8 > 0)
      {
        txlen = 8;
        txObj.bF.ctrl.DLC = CAN_DLC_8;
      }
      else
      {
        txlen = len;
        txObj.bF.ctrl.DLC = len;
      }
    }
    if (iscanfd == 1)
    {
      if (len / 64 > 0)
      {
        txlen = 64;
        txObj.bF.ctrl.DLC = CAN_DLC_64;
      }
      else if (len / 48 > 0)
      {
        txlen = 48;
        txObj.bF.ctrl.DLC = CAN_DLC_48;
      }
      else if (len / 32 > 0)
      {
        txlen = 32;
        txObj.bF.ctrl.DLC = CAN_DLC_32;
      }
      else if (len / 24 > 0)
      {
        txlen = 24;
        txObj.bF.ctrl.DLC = CAN_DLC_24;
      }
      else if (len / 20 > 0)
      {
        txlen = 20;
        txObj.bF.ctrl.DLC = CAN_DLC_20;
      }
      else if (len / 16 > 0)
      {
        txlen = 16;
        txObj.bF.ctrl.DLC = CAN_DLC_16;
      }
      else if (len / 12 > 0)
      {
        txlen = 12;
        txObj.bF.ctrl.DLC = CAN_DLC_12;
      }
      else if (len / 8 > 0)
      {
        txlen = 8;
        txObj.bF.ctrl.DLC = CAN_DLC_8;
      }
      else
      {
        txlen = len;
        txObj.bF.ctrl.DLC = len;
      }
    }

    spican_tx_fifo[whichdev].spi_tx_obj[spican_tx_fifo[whichdev].head].txObj = txObj;
    memcpy(&spican_tx_fifo[whichdev].spi_tx_obj[spican_tx_fifo[whichdev].head].buf[0], buf, txlen);
    spican_tx_fifo[whichdev].head++;
    spican_tx_fifo[whichdev].head %= SPICAN_FIFO_SIZE;

    if (spican_tx_fifo[whichdev].head == spican_tx_fifo[whichdev].tail)
    {
      spican_tx_fifo[whichdev].overflow = 1;
    }
    buf += txlen;
    retlen += txlen;
    len -= txlen;
    // if (spican_outonce(whichdev, &txObj, buf, txlen) == txlen)
    // {
    //   buf += txlen;
    //   retlen += txlen;
    //   len -= txlen;
    // }
    // else
    // {
    //   break;
    // }
  }
  spican_startranfer(whichdev);
  return retlen;
}

/**
  * @brief   SPI发送接收
  * @param[in]  spiSlaveDeviceIndex: 从设配号 CAN3~CAN5 
  * @param[in]  SpiTxData: 发送字节缓存区
  * @param[out]  SpiRxData: 接收缓存区 
  * @param[out]  spiTransferSize: 传输字节数
  * @retval   0   成功
  * @retval   -1  失败
  */
int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
  int8_t ret = -1;

  switch (spiSlaveDeviceIndex)
  {
  case SPICAN3:
    ret = (int8_t)SPI_TransmitReceive(SPI_CH3, SpiTxData, SpiRxData, spiTransferSize);
    break;
  case SPICAN4:
    ret = (int8_t)SPI_TransmitReceive(SPI_CH5, SpiTxData, SpiRxData, spiTransferSize);
    break;
  case SPICAN5:
    ret = (int8_t)SPI_TransmitReceive(SPI_CH6, SpiTxData, SpiRxData, spiTransferSize);
    break;
  default:

    break;
  }
  return ret;
}

/**
 * @brief   SPI DMA发送
 * @param[in]  spiSlaveDeviceIndex: 设备号 CAN3~CAN5
 * @param[in]  SpiTxData: 发送字节缓存区
 * @param[out]  SpiRxData: 接收缓存区 
 * @param[out]  spiTransferSize: 传输字节数
 * @retval   0   成功
 * @retval   -1  失败
 */
int8_t DRV_SPIDMA_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
  int8_t ret = -1;

  switch (spiSlaveDeviceIndex)
  {
  case SPICAN3:
    ret = (int8_t)SPI_TransmitReceive_DMA(SPI_CH3, SpiTxData, SpiRxData, spiTransferSize);
    break;
  case SPICAN4:
    ret = (int8_t)SPI_TransmitReceive(SPI_CH5, SpiTxData, SpiRxData, spiTransferSize);
    break;
  case SPICAN5:
    ret = (int8_t)SPI_TransmitReceive(SPI_CH6, SpiTxData, SpiRxData, spiTransferSize);
    break;
  default:

    break;
  }
  return ret;
}
/* -----------------------------------INT IQN---------------------------------*/
/**
* @brief  SPI 3 4 中断函数.  
* @param  
*/
void SPICAN34_INT_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(SPICAN3_RXINT_PIN);
  HAL_GPIO_EXTI_IRQHandler(SPICAN4_RXINT_PIN);
}

/**
* @brief  SPI5中断函数.  
* @param  
*/
void SPICAN5_INT_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(SPICAN5_RXINT_PIN);
}

/**
  * @brief 中断回调函数
  * @param[in] GPIO_Pin: 中断线引脚
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SPICAN3_RXINT_PIN)
  {
    // Spican_Rx_Loop(SPICAN3);
  }
  if (GPIO_Pin == SPICAN4_RXINT_PIN)
  {
    // Spican_Rx_Loop(SPICAN4);
  }
  if (GPIO_Pin == SPICAN5_RXINT_PIN)
  {
    // Spican_Rx_Loop(SPICAN5);
  }
}
uint8_t spiTransmitBuffer_dma[SPI_DEFAULT_BUFFER_LENGTH];

//! SPI Receive buffer
uint8_t spiReceiveBuffer_dma[SPI_DEFAULT_BUFFER_LENGTH];

int8_t DRV_CANFDSPIDMA_ReadByteArray(CANFDSPI_MODULE_ID index, uint16_t address,
        uint8_t *rxd, uint16_t nBytes)
{
    uint16_t i;
    uint16_t spiTransferSize = nBytes + 2;
    int8_t spiTransferError = 0;

    // Validate that length of array is sufficient to hold requested number of bytes
    if (spiTransferSize > sizeof(spiTransmitBuffer_dma)) {
        return -1;
    }

    // Compose command
    spiTransmitBuffer_dma[0] = (uint8_t) ((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
    spiTransmitBuffer_dma[1] = (uint8_t) (address & 0xFF);

    // Clear data
    for (i = 2; i < spiTransferSize; i++) {
        spiTransmitBuffer_dma[i] = 0;
    }

    spiTransferError = DRV_SPIDMA_TransferData(index, spiTransmitBuffer_dma, spiReceiveBuffer_dma, spiTransferSize);

    // Update data
//    for (i = 0; i < nBytes; i++) {
//        rxd[i] = spiReceiveBuffer_dma[i + 2];
//    }

    return spiTransferError;
}

int8_t DRV_CANFDSPIDMA_ReadByte(CANFDSPI_MODULE_ID index, uint16_t address, uint8_t **rxd)
{
    uint16_t spiTransferSize = 3;
    int8_t spiTransferError = 0;

    // Compose command
    spiTransmitBuffer_dma[0] = (uint8_t) ((cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF));
    spiTransmitBuffer_dma[1] = (uint8_t) (address & 0xFF);
    spiTransmitBuffer_dma[2] = 0;

    spiTransferError = DRV_SPIDMA_TransferData(index, spiTransmitBuffer_dma, spiReceiveBuffer_dma, spiTransferSize);

    // Update data
    *rxd = &spiReceiveBuffer_dma[2];

    return spiTransferError;
}

int8_t DRV_CANFDSPIDMA_ReadWordArray(CANFDSPI_MODULE_ID index, uint16_t address,
        uint8_t **rxd, uint16_t nWords)
{
    uint16_t i, j, n;
    REG_t w;
    uint16_t spiTransferSize = nWords * 4 + 2;
    int8_t spiTransferError = 0;

    // Validate that length of array is sufficient to hold requested number of bytes
    if (spiTransferSize > sizeof(spiTransmitBuffer_dma)) {
        return -1;
    }

    // Compose command
    spiTransmitBuffer_dma[0] = (cINSTRUCTION_READ << 4) + ((address >> 8) & 0xF);
    spiTransmitBuffer_dma[1] = address & 0xFF;

    // Clear data
    for (i = 2; i < spiTransferSize; i++) {
        spiTransmitBuffer_dma[i] = 0;
    }

    spiTransferError = DRV_SPIDMA_TransferData(index, spiTransmitBuffer_dma, spiReceiveBuffer_dma, spiTransferSize);
    if (spiTransferError) {
        return spiTransferError;
    }

    // Convert Byte array to Word array
    *rxd = &spiReceiveBuffer_dma[2];
    
    return spiTransferError;
}

int8_t DRV_CANFDSPIDMA_WriteByte(CANFDSPI_MODULE_ID index, uint16_t address, uint8_t txd)
{
    uint16_t spiTransferSize = 3;
    int8_t spiTransferError = 0;

    // Compose command
    spiTransmitBuffer_dma[0] = (uint8_t) ((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
    spiTransmitBuffer_dma[1] = (uint8_t) (address & 0xFF);
    spiTransmitBuffer_dma[2] = txd;

    spiTransferError = DRV_SPIDMA_TransferData(index, spiTransmitBuffer_dma, spiReceiveBuffer_dma, spiTransferSize);

    return spiTransferError;
}

int8_t DRV_CANFDSPIDMA_WriteByteArray(CANFDSPI_MODULE_ID index, uint16_t address,
        uint8_t *txd, uint16_t nBytes)
{
    uint16_t i;
    uint16_t spiTransferSize = nBytes + 2;
    int8_t spiTransferError = 0;

    // Validate that length of array is sufficient to hold requested number of bytes
    if (spiTransferSize > sizeof(spiTransmitBuffer_dma)) {
        return -1;
    }

    // Compose command
    spiTransmitBuffer_dma[0] = (uint8_t) ((cINSTRUCTION_WRITE << 4) + ((address >> 8) & 0xF));
    spiTransmitBuffer_dma[1] = (uint8_t) (address & 0xFF);

    // Add data
    for (i = 0; i < nBytes; i++) {
        spiTransmitBuffer_dma[i+2] = txd[i];
    }

    spiTransferError = DRV_SPIDMA_TransferData(index, spiTransmitBuffer_dma, spiReceiveBuffer_dma, spiTransferSize);

    return spiTransferError;
}


uint8_t **tx_fifo_event = NULL;
uint32_t fifoReg[3];
uint32_t rfifoReg[3];
int8_t DRV_CANFDSPIDMA_TransmitChannelEventGet(CANFDSPI_MODULE_ID index, CAN_FIFO_CHANNEL channel)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);
    spiTransferError = DRV_CANFDSPIDMA_ReadByte(index, a, tx_fifo_event);
    if (spiTransferError) {
        return -1;
    }
    // Update data
    return spiTransferError;
}
uint8_t **rx_fifo_event = NULL;
int8_t DRV_CANFDSPIDMA_ReceiveChannelEventGet(CANFDSPI_MODULE_ID index, CAN_FIFO_CHANNEL channel)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    if (channel == CAN_TXQUEUE_CH0) return -100;

    a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);

    spiTransferError = DRV_CANFDSPIDMA_ReadByte(index, a, rx_fifo_event);
    if (spiTransferError) {
        return -1;
    }
    return spiTransferError;
}

int8_t DRV_CANFDSPIDMA_ReceiveMessageGet(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_RX_MSGOBJ* rxObj,
        uint8_t *rxd, uint8_t nBytes)
{
    uint8_t n = 0;
    uint8_t i = 0;
    uint16_t a;
    
    REG_CiFIFOCON ciFifoCon;
    __attribute__((unused)) REG_CiFIFOSTA ciFifoSta;
    REG_CiFIFOUA ciFifoUa;
    int8_t spiTransferError = 0;


    // Check that it is a receive buffer
    ciFifoCon.word = rfifoReg[0];

    // Get Status
    ciFifoSta.word = rfifoReg[1];

    // Get address
    ciFifoUa.word = rfifoReg[2];
#ifdef USERADDRESS_TIMES_FOUR
    a = 4 * ciFifoUa.bF.UserAddress;
#else
    a = ciFifoUa.bF.UserAddress;
#endif
    a += cRAMADDR_START;

    // Number of bytes to read
    n = nBytes + 8; // Add 8 header bytes

    if (ciFifoCon.rxBF.RxTimeStampEnable) {
        n += 4; // Add 4 time stamp bytes
    }

    // Make sure we read a multiple of 4 bytes from RAM
    if (n % 4) {
        n = n + 4 - (n % 4);
    }

    // Read rxObj using one access
    uint8_t ba[MAX_MSG_SIZE];

    if (n > MAX_MSG_SIZE) {
        n = MAX_MSG_SIZE;
    }

    spiTransferError = DRV_CANFDSPIDMA_ReadByteArray(index, a, ba, n);
    if (spiTransferError) {
        return -3;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPIDMA_ReceiveChannelUpdate(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel)
{
    uint16_t a = 0;
    REG_CiFIFOCON ciFifoCon;
    int8_t spiTransferError = 0;
    ciFifoCon.word = 0;

    // Set UINC
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1; // Byte that contains FRESET
    ciFifoCon.rxBF.UINC = 1;

    // Write byte
    spiTransferError = DRV_CANFDSPIDMA_WriteByte(index, a, ciFifoCon.byte[1]);

    return spiTransferError;
}


int8_t DRV_CANFDSPIDMA_TransmitChannelLoad(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, CAN_TX_MSGOBJ* txObj,
        uint8_t *txd, uint32_t txdNumBytes, bool flush)
{
    uint16_t a;
    uint32_t dataBytesInObject;
    __attribute__((unused)) REG_CiFIFOSTA ciFifoSta;
    REG_CiFIFOUA ciFifoUa;
    int8_t spiTransferError = 0;

    // Check that DLC is big enough for data
    dataBytesInObject = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC) txObj->bF.ctrl.DLC);
    if (dataBytesInObject < txdNumBytes) {
        return -3;
    }

    // Get status
    ciFifoSta.word = fifoReg[1];

    // Get address
    ciFifoUa.word = fifoReg[2];
#ifdef USERADDRESS_TIMES_FOUR
    a = 4 * ciFifoUa.bF.UserAddress;
#else
    a = ciFifoUa.bF.UserAddress;
#endif
    a += cRAMADDR_START;

    uint8_t txBuffer[MAX_MSG_SIZE];

    txBuffer[0] = txObj->byte[0]; //not using 'for' to reduce no of instructions
    txBuffer[1] = txObj->byte[1];
    txBuffer[2] = txObj->byte[2];
    txBuffer[3] = txObj->byte[3];

    txBuffer[4] = txObj->byte[4];
    txBuffer[5] = txObj->byte[5];
    txBuffer[6] = txObj->byte[6];
    txBuffer[7] = txObj->byte[7];

    uint8_t i;
    for (i = 0; i < txdNumBytes; i++) {
        txBuffer[i + 8] = txd[i];
    }

    // Make sure we write a multiple of 4 bytes to RAM
    uint16_t n = 0;
    uint8_t j = 0;

    if (txdNumBytes % 4) {
        // Need to add bytes
        n = 4 - (txdNumBytes % 4);
        i = txdNumBytes + 8;

        for (j = 0; j < n; j++) {
            txBuffer[i + 8 + j] = 0;
        }
    }

    spiTransferError = DRV_CANFDSPIDMA_WriteByteArray(index, a, txBuffer, txdNumBytes + 8 + n);
    if (spiTransferError) {
        return -4;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPIDMA_TransmitChannelUpdate(CANFDSPI_MODULE_ID index,
        CAN_FIFO_CHANNEL channel, bool flush)
{
    uint16_t a;
    REG_CiFIFOCON ciFifoCon;
    int8_t spiTransferError = 0;

    // Set UINC
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1; // Byte that contains FRESET
    ciFifoCon.word = 0;
    ciFifoCon.txBF.UINC = 1;

    // Set TXREQ
    if (flush) {
        ciFifoCon.txBF.TxRequest = 1;
    }

    spiTransferError = DRV_CANFDSPIDMA_WriteByte(index, a, ciFifoCon.byte[1]);
    if (spiTransferError) {
        return -1;
    }

    return spiTransferError;
}

static volatile uint8_t txstep = 0; // 0-判断txfull flag 1-确定txfull flag 开始判断发送使能和txfifo基地址 
                           // 2-判断成功之后发送数据, 3-发送完数据之后 开启地址更新 
uint8_t **txfifoReg = NULL;

REG_CiFIFOCON ciFifoCon = {0};
volatile uint8_t txing = 0;
volatile uint8_t rxing = 0;
void spican_startranfer(uint8_t whichdev)
{
  uint8_t packetlen = 0;
  uint32_t dlctobyte = 0;
  packetlen = ((spican_tx_fifo[whichdev].head - spican_tx_fifo[whichdev].tail) + SPICAN_FIFO_SIZE) % SPICAN_FIFO_SIZE;
//  if(rxing == 1){
//    return;
//  }
  if (packetlen > 0)
  {
    txing = 1; 
    switch (txstep)
    {
    case 0:
         DRV_CANFDSPIDMA_TransmitChannelEventGet(whichdev, CAN_FIFO_CH2);
      break;
    case 1:
        DRV_CANFDSPIDMA_ReadWordArray(whichdev, cREGADDR_CiFIFOCON + (CAN_FIFO_CH2 * CiFIFO_OFFSET), txfifoReg, 3);
      break;
     case 2:
      dlctobyte = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)spican_tx_fifo[whichdev].spi_tx_obj[spican_tx_fifo[whichdev].tail].txObj.bF.ctrl.DLC);
      DRV_CANFDSPIDMA_TransmitChannelLoad(whichdev, CAN_FIFO_CH2, &spican_tx_fifo[whichdev].spi_tx_obj[spican_tx_fifo[whichdev].tail].txObj, \
                                          &spican_tx_fifo[whichdev].spi_tx_obj[spican_tx_fifo[whichdev].tail].buf[0], dlctobyte, true);
          break; 
     case 3:
         DRV_CANFDSPIDMA_TransmitChannelUpdate(whichdev, CAN_FIFO_CH2, true);
         spican_tx_fifo[whichdev].tail++;
         spican_tx_fifo[whichdev].tail %= SPICAN_FIFO_SIZE;  
          break;
    default:
      break;
    }
  }
  else{
    txing = 0;
  }
}
static uint8_t rxstep = 0; // 0-判断rx flag 1-确定rx flag 开始判断发送使能和rxfifo基地址 
                           // 2-判断成功之后接收数据, 3-接收完数据之后 开启地址更新
uint8_t **rxfifoReg = NULL;

void Spican_Rx_DMALoop(uint8_t whichdev)
{
  CAN_RX_MSGOBJ rxObj = {0};              /*!<  ???????????*/
  uint8_t buf[SPICAN_DATA_MAXSIZE] = {0}; /*!< ?????????*/
//  if(txing == 1){
//    return;
//  }
  rxing = 1;
   switch (rxstep)
    {
    case 0:
        DRV_CANFDSPIDMA_ReceiveChannelEventGet(whichdev, CAN_FIFO_CH1);
        
      break;
    case 1:
        DRV_CANFDSPIDMA_ReadWordArray(whichdev, cREGADDR_CiFIFOCON + (CAN_FIFO_CH1 * CiFIFO_OFFSET), rxfifoReg, 3);
      break;
     case 2:
        DRV_CANFDSPIDMA_ReceiveMessageGet(whichdev, CAN_FIFO_CH1, &rxObj, &buf[0], 64);
      break; 
     case 3:
         // UINC channel
        DRV_CANFDSPIDMA_ReceiveChannelUpdate(whichdev, CAN_FIFO_CH1);
      break;
      
    default:
      break;
    }
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  CAN_TX_FIFO_EVENT txFlags = CAN_TX_FIFO_NO_EVENT;
  CAN_RX_FIFO_EVENT rxFlags = CAN_RX_FIFO_NO_EVENT;
  REG_CiFIFOCON rciFifoCon = {0};
  REG_t w;
  REG_t myReg;
  CAN_RX_MSGOBJ rxObj = {0}; 
  uint8_t buf[SPICAN_DATA_MAXSIZE] = {0};
  uint8_t i = 0,j = 0, n = 2;
  if (hspi == &SpiHandle[SPI_CH1])
  {
    SCB_CleanInvalidateDCache();
  }
  if (hspi == &SpiHandle[SPI_CH3])
  {
    SCB_CleanInvalidateDCache();
    if(txing == 1){
    switch (txstep)
    {
    case 0:
    if(*tx_fifo_event != NULL){
      txFlags = (CAN_TX_FIFO_EVENT) (**tx_fifo_event & CAN_TX_FIFO_ALL_EVENTS);
      if (txFlags & CAN_TX_FIFO_NOT_FULL_EVENT){
        txstep = 1;
        **tx_fifo_event = 0;
        *tx_fifo_event = NULL;
      }
    }
      break;
    case 1:
      for (i = 0; i < 3; i++) {
        w.word = 0;
        for (j = 0; j < 4; j++, n++) {
            w.byte[j] = spiReceiveBuffer_dma[n];
        }
        fifoReg[i] = w.word;
      }
      ciFifoCon.word = fifoReg[0];
      if(ciFifoCon.txBF.TxEnable){
        txstep = 2;
      }
      break;
    case 2:
       txstep = 3;
    break;
      case 3:
       txstep = 0;
       break;
    default:     
      break;
    }
    spican_startranfer(SPICAN3);
    }
    if(rxing == 1){
   switch (rxstep)
    {
    case 0:
    if(*rx_fifo_event != NULL){
    // Update data
      rxFlags = (CAN_RX_FIFO_EVENT) (**rx_fifo_event & CAN_RX_FIFO_ALL_EVENTS);
      if (rxFlags & CAN_TX_FIFO_NOT_FULL_EVENT){
        rxstep = 1;
        **rx_fifo_event = 0;
        *rx_fifo_event = NULL;
      }
    }
      break;
    case 1:
      for (i = 0; i < 3; i++) {
        w.word = 0;
        for (j = 0; j < 4; j++, n++) {
            w.byte[j] = spiReceiveBuffer_dma[n];
        }
        rfifoReg[i] = w.word;
      }
      rciFifoCon.word = rfifoReg[0];
      if(!rciFifoCon.txBF.TxEnable){
        rxstep = 2;
      }
      break;
    case 2:
           // Assign message header
    myReg.byte[0] = spiReceiveBuffer_dma[2];
    myReg.byte[1] = spiReceiveBuffer_dma[3];
    myReg.byte[2] = spiReceiveBuffer_dma[4];
    myReg.byte[3] = spiReceiveBuffer_dma[5];
    rxObj.word[0] = myReg.word;

    myReg.byte[0] = spiReceiveBuffer_dma[6];
    myReg.byte[1] = spiReceiveBuffer_dma[7];
    myReg.byte[2] = spiReceiveBuffer_dma[8];
    myReg.byte[3] = spiReceiveBuffer_dma[9];
    rxObj.word[1] = myReg.word;
    rxObj.word[2] = 0;
     // Assign message data
     for (i = 0; i < 64; i++) {
            buf[i] = spiReceiveBuffer_dma[i + 10];
     }
      spican_rx_fifo[SPICAN3].spi_rx_obj[spican_rx_fifo[SPICAN3].head].rxObj = rxObj;
      memcpy(&spican_rx_fifo[SPICAN3].spi_rx_obj[spican_rx_fifo[SPICAN3].head].buf[0], buf, SPICAN_DATA_MAXSIZE);
      spican_rx_fifo[SPICAN3].head++;
      spican_rx_fifo[SPICAN3].head %= SPICAN_FIFO_SIZE;
      if (spican_rx_fifo[SPICAN3].head == spican_rx_fifo[SPICAN3].tail)
      {
        spican_rx_fifo[SPICAN3].overflow = 1;
      }
      rxstep = 3;
    break;
      case 3:
       rxstep = 0;
       break;
    default:     
      break;
    }
    rxing  = 0;
    } 
  }
}



#ifdef Drv_SPICAN_TSET

uint8_t txd[MAX_DATA_BYTES];
uint8_t rxd[MAX_DATA_BYTES];

//! Test RAM access

bool APP_TestRamAccess(uint8_t whichdev)
{
  // Variables
  uint8_t i = 0;
  uint8_t length;
  bool good = false;

  Nop();

  // Verify read/write with different access length
  // Note: RAM can only be accessed in multiples of 4 bytes
  for (length = 4; length <= MAX_DATA_BYTES; length += 4)
  {
    for (i = 0; i < length; i++)
    {
      txd[i] = rand() & 0xff;
      rxd[i] = 0xff;
    }

    Nop();

    // Write data to RAM
    DRV_CANFDSPI_WriteByteArray(whichdev, cRAMADDR_START, txd, length);

    // Read data back from RAM
    DRV_CANFDSPI_ReadByteArray(whichdev, cRAMADDR_START, rxd, length);

    // Verify
    good = false;
    for (i = 0; i < length; i++)
    {
      good = txd[i] == rxd[i];

      if (!good)
      {
        Nop();
        Nop();

        // Data mismatch
        return false;
      }
    }
  }

  return true;
}
/**
 * @brief  SPICAN ???????.
 */

uint8_t buf[255];
void SpiCan_Test(void)
{
  uint16_t i = 0;
  Spican_Init(SPICAN4, ARBITR_1M, DATABIT_1M, CAN_PLSIZE_8);
  Spican_Init(SPICAN5, ARBITR_1M, DATABIT_1M, CAN_PLSIZE_8);
  Spican_Init(SPICAN3, ARBITR_1M, DATABIT_1M, CAN_PLSIZE_8);

  for (i = 0; i < 255; i++)
  {
    buf[i] = i;
  }

  APP_TestRamAccess(SPICAN3);
  APP_TestRamAccess(SPICAN4);
  APP_TestRamAccess(SPICAN5);

 // Spican_Out(SPICAN4, 0x04, EID, buf, 255, 0);
  //Spican_Out(SPICAN5, 0x05, EID, buf, 255, 0);
  Spican_Out(SPICAN3, 0x02, EID, buf, 8, 0);
  Spican_Out(SPICAN3, 0x02, EID, buf, 8, 0);
  while (1)
  {
  }
}

#endif

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
/*****************************END OF FILE**************************************/
