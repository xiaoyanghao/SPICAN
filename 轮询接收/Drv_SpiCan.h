/**
  ******************************************************************************
  * @file    Drv_SpiCan.h
  * @author  YZH
  * @brief   spi ת CAN �豸.
  */

#ifndef _DRV_SPICAN_H
#define _DRV_SPICAN_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"

/** @addtogroup �豸����
  * @{
  */
/** @defgroup SPICAN����
  * @brief   MCP2517FDT-H_SL.
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SPICAN�ⲿö����ṹ������
  * @{
  */ 

  /**
  * @brief SPICAN �豸
  */ 
typedef enum
 {
   
   SPICAN3,            /*!<  CAN3 */ 
   SPICAN4,            /*!<  CAN4 */ 
   SPICAN5,            /*!<  CAN5 */ 
   SPICANMAX
     
 }SPICAN_DEV;
  

 
  /**
  * @brief �ٲ�λ������
  */ 
typedef enum
 {
   
    ARBITR_500K, 
    ARBITR_250K = 0x08, 
    ARBITR_1M = 0x0f, 
    ARBITR_125K = 0x11 
   
 }SPICAN_ARBITR;
 

  /**
  * @brief ����λ������
  */ 
typedef enum
 {
   
    DATABIT_1M, 
    DATABIT_2M, 
    DATABIT_3M,
    DATABIT_4M,
    DATABIT_5M, 
    DATABIT_6M7,
    DATABIT_8M, 
    DATABIT_500K = 0x08,
 }SPICAN_DATABIT;



  /**
  * @brief SPICAN ID ����
  */ 
typedef enum
 {
   
   SID,            /*!<  ��׼֡ */ 
   EID,            /*!<  ��չ֡ */ 
   
 }SPICAN_TYID;


/**
  * @brief SPICAN �����ֽڴ�С
*/ 
typedef enum {
    PLSIZE_8,
    PLSIZE_12,
    PLSIZE_16,
    PLSIZE_20,
    PLSIZE_24,
    PLSIZE_32,
    PLSIZE_48,
    PLSIZE_64
} SPICAN_PLSIZE;
 

/**
* @}
*/  

 /** @defgroup SPICAN�ⲿ��
  * @{
  */ 

#define SPI_DEFAULT_BUFFER_LENGTH  96                          /*!<  SPI Ĭ�ϴ�������ֽڳ��� */ 
#define Nop()                      asm("nop")                  /*!<  nop ��ʱ */ 
#define Drv_SPICAN_TSET                                        /*!<  �豸���� ���� */ 
#define SPICAN_FIFO_SIZE          64                          /*!<  ���ջ�������С */ 
#define SPICAN_DATA_MAXSIZE       64                          /*!<  һ�����������ݳ���BYTE��С*/ 

/**
* @}
*/  
 
 /* Exported function ------------------------------------------------------------*/
/** @defgroup SPICAN�ⲿ���� SPICAN�ⲿ���� 
  * @{
  */ 
int8_t Spican_Init(uint8_t whichdev, uint8_t arbitr_bittime, uint8_t data_bittime, uint8_t plsize);
void Spican_Setfliter(uint8_t whichdev, uint32_t sid, uint32_t sidmask, uint32_t eid, uint32_t eidmask);
uint16_t Spican_Out(uint8_t whichdev, uint32_t id, uint8_t type_id, uint8_t *buf, uint16_t len, uint8_t iscanfd);
uint8_t Spican_In(uint8_t whichdev, uint32_t *id, uint8_t *type_id, uint8_t *buf, uint8_t len, uint8_t *iscanfd);
uint8_t Spican_Rx_Packetlen(uint8_t whichdev);
uint8_t Spican_Rx_Packet_Isoverflow(uint8_t whichdev);
void Spican_Rx_Loop(uint8_t whichdev);
int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize);

#ifdef Drv_SPICAN_TSET
 void SpiCan_Test(void);
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
#ifdef __cplusplus
}
#endif

#endif

/*****************************END OF FILE**************************************/




















