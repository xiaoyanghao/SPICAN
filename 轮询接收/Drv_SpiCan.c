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

   CAN_RX_MSGOBJ rxObj;                  /*!<  接收消息类型*/ 
   uint8_t     buf[SPICAN_DATA_MAXSIZE];   /*!< 接收缓存区*/  
}SPICAN_RX_OBJ;
 
  /**
  * @brief SPICAN 接收缓存区类型
  */ 
typedef struct 
{
   uint8_t     head;     /*!<  接收帧头*/ 
   uint8_t     tail;     /*!<  接收帧尾*/ 
   uint8_t     overflow; /*!<  接收溢出标志位*/ 
   uint8_t     count;    /*!<  接收计数*/ 
   SPICAN_RX_OBJ spi_rx_obj[SPICAN_FIFO_SIZE];   /*!< SPICAN 接收类型*/ 
}SPICAN_RX_FIFO;
 
  /**
* @}
*/    
/* Private macro -------------------------------------------------------------*/

 /** @defgroup SPICAN内部宏
  * @{
  */ 

#define SPICAN3_INT_CLK_ENABLE()          __HAL_RCC_GPIOI_CLK_ENABLE()
#define SPICAN3_INT_PORT                  GPIOI
#define SPICAN3_RXINT_PIN                 GPIO_PIN_15

#define SPICAN4_INT_CLK_ENABLE()          __HAL_RCC_GPIOI_CLK_ENABLE()
#define SPICAN4_INT_PORT                  GPIOI
#define SPICAN4_RXINT_PIN                 GPIO_PIN_11
#define SPICAN34_INT_IRQHandler           EXTI15_10_IRQHandler
#define SPICAN34_INT_IRQn                 EXTI15_10_IRQn

#define SPICAN5_INT_CLK_ENABLE()          __HAL_RCC_GPIOH_CLK_ENABLE()
#define SPICAN5_INT_PORT                  GPIOH
#define SPICAN5_RXINT_PIN                 GPIO_PIN_2
#define SPICAN5_INT_IRQHandler            EXTI2_IRQHandler
#define SPICAN5_INT_IRQn                  EXTI2_IRQn

 /**
* @}
*/  
/* Private variables ---------------------------------------------------------*/
SPICAN_RX_FIFO spican_rx_fifo[SPICANMAX] = {0}; /*!< SPICAN3接收缓存区*/ 

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
   CAN_TX_FIFO_EVENT txFlags  = CAN_TX_FIFO_NO_EVENT;
   do {
     DRV_CANFDSPI_TransmitChannelEventGet(whichdev, CAN_FIFO_CH2, &txFlags);
     if (attempts == 0) {
       Nop();
       Nop();
       return 0;
     }
     attempts--;
   } while (!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT));
   
   // 发送消息
   dlctobyte = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)txObj->bF.ctrl.DLC);
   if (DRV_CANFDSPI_TransmitChannelLoad(whichdev, CAN_FIFO_CH2, txObj, buf, dlctobyte, true) != 0){
     retlen = 0;
   }
   else{
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
  switch (whichdev){
    case SPICAN3: 
      ret = SPI_Init(SPI_CH3, SPI_BAUDRATE_12_5M); 
      if(ret != RET_OK){
        return -1;
      }
      SPICAN3_INT_CLK_ENABLE();
      GPIO_InitStruct.Pin  = SPICAN3_RXINT_PIN;
      HAL_GPIO_Init(SPICAN3_INT_PORT, &GPIO_InitStruct);
    //  HAL_NVIC_SetPriority(SPICAN34_INT_IRQn, 1, 1);
    //  HAL_NVIC_EnableIRQ(SPICAN34_INT_IRQn);  
    break;
    case SPICAN4: 
      ret = SPI_Init(SPI_CH5, SPI_BAUDRATE_3_125M); 
      if(ret != RET_OK){
        return -1;
      }
      SPICAN4_INT_CLK_ENABLE();
      GPIO_InitStruct.Pin  = SPICAN4_RXINT_PIN;
      HAL_GPIO_Init(SPICAN4_INT_PORT, &GPIO_InitStruct);
    //  HAL_NVIC_SetPriority(SPICAN34_INT_IRQn, 0, 3);
    //  HAL_NVIC_EnableIRQ(SPICAN34_INT_IRQn);   
    break;
    case SPICAN5: 
      ret = SPI_Init(SPI_CH6, SPI_BAUDRATE_3_125M); 
      if(ret != RET_OK){
        return -1;
      }
      SPICAN5_INT_CLK_ENABLE();
      GPIO_InitStruct.Pin  = SPICAN5_RXINT_PIN;
      HAL_GPIO_Init(SPICAN5_INT_PORT, &GPIO_InitStruct);
     // HAL_NVIC_SetPriority(SPICAN5_INT_IRQn, 1, 1);
     // HAL_NVIC_EnableIRQ(SPICAN5_INT_IRQn);    
    break; 
    default:
    
    break;
  }
  //复位设备
  DRV_CANFDSPI_Reset(whichdev);
  
  //使能错误校准
  DRV_CANFDSPI_EccEnable(whichdev);
  
  //复位 RAM 
  DRV_CANFDSPI_RamInit(whichdev, 0xff);
  
  //复位 CAN配置
  DRV_CANFDSPI_ConfigureObjectReset(&config);
  
  config.IsoCrcEnable = 0;
  config.StoreInTEF = 0;
  
  // 采用默认配置
  DRV_CANFDSPI_Configure(whichdev, &config);
  
  DRV_CANFDSPI_TransmitBandWidthSharingSet(whichdev, CAN_TXBWS_16);
    
  // 复位发送FIFO配置   配置发送FIFO 8个数据位
  DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
  txConfig.FifoSize = 10;//8;
  txConfig.PayLoadSize = plsize;//plsize;(64+8) * 10 = 720byte
  txConfig.TxPriority = 1;
  DRV_CANFDSPI_TransmitChannelConfigure(whichdev, CAN_FIFO_CH2, &txConfig);
  
  // 复位接收FIFO配置  配置接收FIFO 8个数据位
  DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
  rxConfig.FifoSize = 10;//20;
  rxConfig.PayLoadSize = plsize;//plsize; (64+12) * 10 = 760byte
  DRV_CANFDSPI_ReceiveChannelConfigure(whichdev, CAN_FIFO_CH1, &rxConfig);
  
  DRV_CANFDSPI_FilterObjectConfigure(whichdev, CAN_FILTER0, &fObj.bF);
  DRV_CANFDSPI_FilterMaskConfigure(whichdev, CAN_FILTER0, &mObj.bF);
  
  DRV_CANFDSPI_FilterToFifoLink(whichdev, CAN_FILTER0, CAN_FIFO_CH1, true);
  
  DRV_CANFDSPI_BitTimeConfigureNominal20MHz(whichdev,(CAN_BITTIME_SETUP) arbitr_bittime);
 
  DRV_CANFDSPI_BitTimeConfigureData20MHz(whichdev,(CAN_BITTIME_SETUP) data_bittime, CAN_SSP_MODE_AUTO);
   
  DRV_CANFDSPI_ReceiveChannelEventEnable(whichdev, CAN_FIFO_CH1, CAN_RX_FIFO_NOT_EMPTY_EVENT);
  DRV_CANFDSPI_TransmitChannelEventEnable(whichdev, CAN_FIFO_CH2, CAN_TX_FIFO_EMPTY_EVENT);
   
  DRV_CANFDSPI_ModuleEventEnable(whichdev, (CAN_MODULE_EVENT)(CAN_TX_EVENT | CAN_RX_EVENT));
  DRV_CANFDSPI_GpioModeConfigure(whichdev, GPIO_MODE_INT, GPIO_MODE_INT);
  DRV_CANFDSPI_GpioInterruptPinsOpenDrainConfigure(whichdev, GPIO_PUSH_PULL);
   
   // 设置正常模式
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

  //连接 FIFO 和 滤波器
  DRV_CANFDSPI_FilterToFifoLink(whichdev, CAN_FILTER0, CAN_FIFO_CH1, true);
  
  fObj.bF.EID = eid;
  fObj.bF.EXIDE = 1;
  mObj.bF.MSID = eidmask;
  mObj.bF.MEID = 1; 
  DRV_CANFDSPI_FilterObjectConfigure(whichdev, CAN_FILTER1, &fObj.bF);
  DRV_CANFDSPI_FilterMaskConfigure(whichdev, CAN_FILTER1, &mObj.bF);

  //连接 FIFO 和 滤波器
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
  CAN_RX_MSGOBJ rxObj = {0};                  /*!<  接收消息类型*/ 
  uint8_t     buf[SPICAN_DATA_MAXSIZE] = {0};   /*!<  接收消息类型*/  
  DRV_CANFDSPI_ReceiveChannelEventGet(whichdev, CAN_FIFO_CH1, &rxFlags);
  while((rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) && attempts>0){ 
  //if(rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT){
    if(DRV_CANFDSPI_ReceiveMessageGet(whichdev, CAN_FIFO_CH1, &rxObj, &buf[0],64) == 0){
        spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].head].rxObj = rxObj;
        memcpy(&spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].head].buf[0],buf,SPICAN_DATA_MAXSIZE);
        spican_rx_fifo[whichdev].head++;
        spican_rx_fifo[whichdev].head%=SPICAN_FIFO_SIZE;
        if(spican_rx_fifo[whichdev].head == spican_rx_fifo[whichdev].tail){
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
  packetlen =( (spican_rx_fifo[whichdev].head - spican_rx_fifo[whichdev].tail) + SPICAN_FIFO_SIZE) % SPICAN_FIFO_SIZE;
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
  if(Spican_Rx_Packetlen(whichdev) > 0){
    *iscanfd = spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].tail].rxObj.bF.ctrl.FDF;
    *type_id = spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].tail].rxObj.bF.ctrl.IDE;
    if(*type_id == EID){
      *id = spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].tail].rxObj.bF.id.EID;
    }
    else{
      *id = spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].tail].rxObj.bF.id.SID;
    }
    dlctobyte = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].tail].rxObj.bF.ctrl.DLC);
    retlen = (dlctobyte <= len)? dlctobyte : len;
    for (index = 0; index < retlen; index++){
      buf[index] = spican_rx_fifo[whichdev].spi_rx_obj[spican_rx_fifo[whichdev].tail].buf[index];
    }
    spican_rx_fifo[whichdev].tail++;
    spican_rx_fifo[whichdev].tail%=SPICAN_FIFO_SIZE;
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
   if(type_id == SID){
     txObj.bF.id.SID = id;
   }
   else if(type_id == EID){
     txObj.bF.id.EID = id;
   }
   txObj.bF.ctrl.IDE = type_id;
   txObj.bF.ctrl.FDF = iscanfd;
   txObj.bF.ctrl.BRS = iscanfd;
   
   while(len > 0){
     if(iscanfd == 0){
       if(len / 8 > 0){
         txlen = 8;
         txObj.bF.ctrl.DLC = CAN_DLC_8;
       }
       else{
         txlen = len;
         txObj.bF.ctrl.DLC = len;
       }
     }
     if(iscanfd == 1){
       if(len / 64 > 0){
         txlen = 64;
         txObj.bF.ctrl.DLC = CAN_DLC_64;
       }
       else if(len / 48 > 0){
         txlen = 48;
         txObj.bF.ctrl.DLC = CAN_DLC_48;
       }
       else if(len / 32 > 0){
         txlen = 32;
         txObj.bF.ctrl.DLC = CAN_DLC_32;
       }
       else if(len / 24 > 0){
         txlen = 24;
         txObj.bF.ctrl.DLC = CAN_DLC_24;
       }
       else if(len / 20 > 0){
         txlen = 20;
         txObj.bF.ctrl.DLC = CAN_DLC_20;
       }
       else if(len / 16 > 0){
         txlen = 16;
         txObj.bF.ctrl.DLC = CAN_DLC_16;
       }
       else if(len / 12 > 0){
         txlen = 12;
         txObj.bF.ctrl.DLC = CAN_DLC_12;
       }
        else if(len / 8 > 0){
         txlen = 8;
         txObj.bF.ctrl.DLC = CAN_DLC_8;
       }
       else{
         txlen = len;
         txObj.bF.ctrl.DLC = len;
        }
     }
     if(spican_outonce(whichdev, &txObj, buf, txlen) == txlen){
       buf+=txlen;
       retlen+=txlen;
       len-=txlen;
     }
     else{
       break;
     }
   }
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
  
  switch (spiSlaveDeviceIndex){
    case SPICAN3: 
       ret = (int8_t) SPI_TransmitReceive(SPI_CH3, SpiTxData, SpiRxData, spiTransferSize);    
    break;
    case SPICAN4: 
       ret = (int8_t) SPI_TransmitReceive(SPI_CH5, SpiTxData, SpiRxData, spiTransferSize);   
    break;
    case SPICAN5: 
       ret = (int8_t) SPI_TransmitReceive(SPI_CH6, SpiTxData, SpiRxData, spiTransferSize);  
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
  * @param[in] GPIO_Pin: 中断线引脚s
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SPICAN3_RXINT_PIN){
    //Spican_Rx_Loop(SPICAN3);
  }
  if (GPIO_Pin == SPICAN4_RXINT_PIN){
    //Spican_Rx_Loop(SPICAN4);
  }
  if (GPIO_Pin == SPICAN5_RXINT_PIN){
   // Spican_Rx_Loop(SPICAN5);
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
    for (length = 4; length <= MAX_DATA_BYTES; length += 4) {
        for (i = 0; i < length; i++) {
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
        for (i = 0; i < length; i++) {
            good = txd[i] == rxd[i];

            if (!good) {
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
  * @brief  SPICAN 测试函数.
  */

uint8_t buf[255];
void SpiCan_Test(void)
{
   uint16_t i = 0;
   Spican_Init(SPICAN4, ARBITR_1M, DATABIT_1M, CAN_PLSIZE_8); 
   Spican_Init(SPICAN5, ARBITR_1M, DATABIT_1M, CAN_PLSIZE_8); 
   Spican_Init(SPICAN3, ARBITR_1M, DATABIT_1M, CAN_PLSIZE_8); 
   
   for( i = 0; i < 255; i++){
     buf[i] = i;  
   }
   
   APP_TestRamAccess(SPICAN3);
   APP_TestRamAccess(SPICAN4);
   APP_TestRamAccess(SPICAN5);
  
   Spican_Out(SPICAN4, 0x04, EID, buf, 255, 0);
   Spican_Out(SPICAN5, 0x05, EID, buf, 255, 0);
   Spican_Out(SPICAN3, 0x02, EID, buf, 255, 0);
   while(1){
    
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










































