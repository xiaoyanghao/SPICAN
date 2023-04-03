/**
  ******************************************************************************
  * @file    Drv_SpiCan.c
  * @author  YZH
  * @brief   spi ת CAN �豸.
  *
  @verbatim
  ==============================================================================
                     ##### оƬ���� #####
  ==============================================================================
[..]
   (+) оƬ�ͺ�   �� MCP2517FDT-H_SL
   (+) �ڴ�       �� 2KByte RAM  31FIFO ������Ϊ���ͻ��߽��ջ�����  
                                 1�����Ͷ��� 1���¼�����
   (+) ������ѹ   �� 2.7~5.5V
   (+) �����¶�   ��-40~150��
   (+) �ⲿ����Ƶ�� �� 20MHZ
   (+) ͨ�ŷ�ʽ  �� SPI ģʽ0��ģʽ3
   (+) ʱ��Ƶ��  �� ���20MHZ 
   (+) ������ʽ  ��֧�� CAN FD �� CAN2.0B���ģʽ
                   ֧�� CAN2.OB ģʽ
  @endverbatim
  ******************************************************************************
  * @attention
   <pre>
         (+) SPI ʱ�Ӳ�׼ȷ  (SPI5 SPI6)ʱ�Ӱ���������ֱ�����õ�3.25MHZ
         (+) ѭ������ռ��CPU�����ʹ���
         (+) �������жϽ��� CPU�����ʲ�û�����Եĸ��ƣ����� �ж����� ż������Ӧ
    </pre>
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Drv_SpiCan.h"
#include "Drv_Spi.h"
#include "drv_canfdspi_api.h"
#include <string.h> 
    
/** @addtogroup �豸����
  * @{
  */
/** @defgroup SPIתCAN����
  * @{
  */

/* Private types -------------------------------------------------------------*/
 /** @defgroup SPICAN�ڲ�ö����ṹ������
  * @{
  */ 
   
  /**
  * @brief SPICAN ��������
  */ 

typedef struct 
{

   CAN_RX_MSGOBJ rxObj;                  /*!<  ������Ϣ����*/ 
   uint8_t     buf[SPICAN_DATA_MAXSIZE];   /*!< ���ջ�����*/  
}SPICAN_RX_OBJ;
 
  /**
  * @brief SPICAN ���ջ���������
  */ 
typedef struct 
{
   uint8_t     head;     /*!<  ����֡ͷ*/ 
   uint8_t     tail;     /*!<  ����֡β*/ 
   uint8_t     overflow; /*!<  ���������־λ*/ 
   uint8_t     count;    /*!<  ���ռ���*/ 
   SPICAN_RX_OBJ spi_rx_obj[SPICAN_FIFO_SIZE];   /*!< SPICAN ��������*/ 
}SPICAN_RX_FIFO;
 
  /**
* @}
*/    
/* Private macro -------------------------------------------------------------*/

 /** @defgroup SPICAN�ڲ���
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
SPICAN_RX_FIFO spican_rx_fifo[SPICANMAX] = {0}; /*!< SPICAN3���ջ�����*/ 

/* Private function ---------------------------------------------------------*/
/** @addtogroup SPICAN ��̬����
  * @{
  */
    
/**
  * @brief  SPICAN ����һ��.
  * @param[in]  whichdev:  SPICAN�豸
  *          @arg @ref SPICAN3  CAN3
  *          @arg @ref SPICAN4  CAN4
  *          @arg @ref SPICAN5  CAN5
  * @param[in]  txObj :  ���ͽṹ��
  * @param[in]  buf :  ��Ҫ���ͻ�������ַ
  * @param[in]  len :  ��Ҫ�����ֽڳ���
  * @retval ʵ�ʷ��͵��ֽ���
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
   
   // ������Ϣ
   dlctobyte = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)txObj->bF.ctrl.DLC);
   if (DRV_CANFDSPI_TransmitChannelLoad(whichdev, CAN_FIFO_CH2, txObj, buf, dlctobyte, true) != 0){
     retlen = 0;
   }
   else{
     retlen = len;
   }
   return retlen;
}

/** @addtogroup SPICAN�ⲿ����
  * @{
  */
/**
  * @brief  SPICAN ��ʼ��.
  * @param[in]  whichdev:  SPICAN�豸
  *          @arg @ref SPICAN3  CAN3
  *          @arg @ref SPICAN4  CAN4
  *          @arg @ref SPICAN5  CAN5
  * @param[in]  arbitr_bittime :  �ٲ�λ������
  * @param[in]  data_bittime :  ����λ������ ������ջ��߷��Ͳ��� CAN2.0 ��λ��Ч 
  * @param[in]  plsize : FIFO ���س��ȴ�С
  *          @arg @ref PLSIZE_8  CAN3
  *          @arg @ref PLSIZE_12  CAN4
  *          @arg @ref PLSIZE_16  CAN5
  *          @arg @ref PLSIZE_20  CAN3
  *          @arg @ref PLSIZE_24  CAN4
  *          @arg @ref PLSIZE_32  CAN5
  *          @arg @ref PLSIZE_48  CAN3
  *          @arg @ref PLSIZE_64  CAN4
  * @retval 0:�ɹ�  
  * @retval -1:ʧ��
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
  //��λ�豸
  DRV_CANFDSPI_Reset(whichdev);
  
  //ʹ�ܴ���У׼
  DRV_CANFDSPI_EccEnable(whichdev);
  
  //��λ RAM 
  DRV_CANFDSPI_RamInit(whichdev, 0xff);
  
  //��λ CAN����
  DRV_CANFDSPI_ConfigureObjectReset(&config);
  
  config.IsoCrcEnable = 0;
  config.StoreInTEF = 0;
  
  // ����Ĭ������
  DRV_CANFDSPI_Configure(whichdev, &config);
  
  DRV_CANFDSPI_TransmitBandWidthSharingSet(whichdev, CAN_TXBWS_16);
    
  // ��λ����FIFO����   ���÷���FIFO 8������λ
  DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
  txConfig.FifoSize = 10;//8;
  txConfig.PayLoadSize = plsize;//plsize;(64+8) * 10 = 720byte
  txConfig.TxPriority = 1;
  DRV_CANFDSPI_TransmitChannelConfigure(whichdev, CAN_FIFO_CH2, &txConfig);
  
  // ��λ����FIFO����  ���ý���FIFO 8������λ
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
   
   // ��������ģʽ
  DRV_CANFDSPI_OperationModeSelect(whichdev, CAN_NORMAL_MODE);
  
  return ret;
}

/**
  * @brief  SPICAN �����˲�.
  * @param[in]  whichdev:  SPICAN�豸
  * @param[in]  sid :  �˲���׼ID
  * @param[in]  sidmask :  �˲���׼֡����
  * @param[in]  eid :  ��չ֡ID
  * @param[in]  eidmask :  ��չ֡����
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

  //���� FIFO �� �˲���
  DRV_CANFDSPI_FilterToFifoLink(whichdev, CAN_FILTER0, CAN_FIFO_CH1, true);
  
  fObj.bF.EID = eid;
  fObj.bF.EXIDE = 1;
  mObj.bF.MSID = eidmask;
  mObj.bF.MEID = 1; 
  DRV_CANFDSPI_FilterObjectConfigure(whichdev, CAN_FILTER1, &fObj.bF);
  DRV_CANFDSPI_FilterMaskConfigure(whichdev, CAN_FILTER1, &mObj.bF);

  //���� FIFO �� �˲���
  DRV_CANFDSPI_FilterToFifoLink(whichdev, CAN_FILTER1, CAN_FIFO_CH1, true);
}


/**
  * @brief  SPICAN ��ѯ����
  * @param[in]  whichdev:  SPICAN�豸
  *          @arg @ref SPICAN3  CAN3
  *          @arg @ref SPICAN4  CAN4
  *          @arg @ref SPICAN5  CAN5
  * @note �����߳���ʹ�� ��3��CPUռ����16%��  
  */
void Spican_Rx_Loop(uint8_t whichdev)
{
  uint8_t attempts = 20;
  CAN_RX_FIFO_EVENT rxFlags = CAN_RX_FIFO_NO_EVENT;
  CAN_RX_MSGOBJ rxObj = {0};                  /*!<  ������Ϣ����*/ 
  uint8_t     buf[SPICAN_DATA_MAXSIZE] = {0};   /*!<  ������Ϣ����*/  
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
  * @brief  SPICAN ���������հ�����.
  * @param[in]  whichdev:  SPICAN�豸
  *          @arg @ref SPICAN3  CAN3
  *          @arg @ref SPICAN4  CAN4
  *          @arg @ref SPICAN5  CAN5
  * @retval ���������հ�����
  */
uint8_t Spican_Rx_Packetlen(uint8_t whichdev)
{
  uint8_t packetlen = 0;
  packetlen =( (spican_rx_fifo[whichdev].head - spican_rx_fifo[whichdev].tail) + SPICAN_FIFO_SIZE) % SPICAN_FIFO_SIZE;
  return packetlen;
}

/**
  * @brief  SPICAN ���������հ��Ƿ����.
  * @retval 1 - ��� 0 - û����� 
  */
uint8_t Spican_Rx_Packet_Isoverflow(uint8_t whichdev)
{
  return spican_rx_fifo[whichdev].overflow;
}

/**
  * @brief  SPICAN ����.
  * @param[in]  whichdev:  SPICAN�豸
  *          @arg @ref SPICAN3  CAN3
  *          @arg @ref SPICAN4  CAN4
  *          @arg @ref SPICAN5  CAN5
  * @param[out]  id :  CAN id
  * @param[out]  type_id :  id ����    
  *          @arg @ref SID  ��׼֡
  *          @arg @ref EID  ��չ֡
  * @param[out]  buf :  ���ջ�������ַ
  * @param[in]  len :  ��Ҫ�����ֽڳ��� С�ڵ���64
  * @param[out]  iscanfd :  �Ƿ�Ϊ CAN FD ����  1-�� 0-��
  * @retval ʵ�ʽ��յ��ֽ���
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
  * @brief  SPICAN ����.
  * @param[in]  whichdev:  SPICAN�豸
  *          @arg @ref SPICAN3  CAN3
  *          @arg @ref SPICAN4  CAN4
  *          @arg @ref SPICAN5  CAN5
  * @param[in]  id :  CAN id
  * @param[in]  type_id :  id ����    
  *          @arg @ref SID  ��׼֡
  *          @arg @ref EID  ��չ֡
  * @param[in]  buf :  ��Ҫ���ͻ�������ַ
  * @param[in]  len :  ��Ҫ�����ֽڳ���
  * @param[in]  iscanfd :  �Ƿ�Ϊ CAN FD ����  1-�� 0-��
  * @retval ʵ�ʷ��͵��ֽ���
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
  * @brief   SPI���ͽ���
  * @param[in]  spiSlaveDeviceIndex: ������� CAN3~CAN5 
  * @param[in]  SpiTxData: �����ֽڻ�����
  * @param[out]  SpiRxData: ���ջ����� 
  * @param[out]  spiTransferSize: �����ֽ���
  * @retval   0   �ɹ�
  * @retval   -1  ʧ��
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
* @brief  SPI 3 4 �жϺ���.  
* @param  
*/
void SPICAN34_INT_IRQHandler(void)
{ 
  HAL_GPIO_EXTI_IRQHandler(SPICAN3_RXINT_PIN);
  HAL_GPIO_EXTI_IRQHandler(SPICAN4_RXINT_PIN);
}


/**
* @brief  SPI5�жϺ���.  
* @param  
*/
void SPICAN5_INT_IRQHandler(void)
{ 
  HAL_GPIO_EXTI_IRQHandler(SPICAN5_RXINT_PIN);
}

/**
  * @brief �жϻص�����
  * @param[in] GPIO_Pin: �ж�������s
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
  * @brief  SPICAN ���Ժ���.
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










































