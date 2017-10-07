/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "BulkSerial.h"

const BulkSerialSettings MBLC_OPENLOG = {PB10, PB11, USART3, DMA1_Channel2, DMA1_Channel3, DMA1_FLAG_TC3};
const BulkSerialSettings MBLC_RPI = {PA2, PA3, USART2, DMA1_Channel7, DMA1_Channel6, DMA1_FLAG_TC6};
const BulkSerialSettings MBLC_PROGPORT = {PA9, PA10, USART1, DMA1_Channel4, DMA1_Channel5, DMA1_FLAG_TC5};

const uint8_t OPENLOG_ALIGNMENT_WORD[] = {0xaa, 0xbb};

void BulkSerial::begin(uint32_t baud, uint16_t sizeTx, void *bufTx, uint16_t sizeRx) {
  this->sizeTx = sizeTx;
  this->sizeRx = sizeRx;

  configUSARTPins(bss.USARTx, bss.txPin, bss.rxPin);

  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = baud;
  if (bTxEvenParity) {
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
  } else {
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_Parity = USART_Parity_No;
  }

  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(bss.USARTx, &USART_InitStructure);

  USART_Cmd(bss.USARTx, ENABLE);

  // this is the same for each chip so far
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  if (sizeTx > 0) {
    // setup TX
    DMA_DeInit(bss.chTx);
    DMA_InitStructureTx.DMA_DIR = DMA_DIR_PeripheralDST; // Transmit
    DMA_InitStructureTx.DMA_MemoryBaseAddr = (uint32_t)bufTx;
    DMA_InitStructureTx.DMA_BufferSize = sizeTx;//CHECK
    DMA_InitStructureTx.DMA_PeripheralBaseAddr = (uint32_t)&bss.USARTx->TDR;
    DMA_InitStructureTx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructureTx.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructureTx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructureTx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructureTx.DMA_Mode = DMA_Mode_Normal;// or Circular
    DMA_InitStructureTx.DMA_Priority = DMA_Priority_High;
    // DMA_Init(bss.chTx, &DMA_InitStructure);
    
    // Don't enable till a TX is needed
  // DMA_Cmd(bss.chTx, ENABLE);

    USART_DMACmd(bss.USARTx, USART_DMAReq_Tx, ENABLE);
  }

  if (sizeRx > 0) {
    // setup RX
    // nvicEnable(bss.irqnRx, 14);

    DMA_DeInit(bss.chRx);
    DMA_InitStructureRx.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructureRx.DMA_MemoryBaseAddr = (uint32_t)localBuf;
    DMA_InitStructureRx.DMA_BufferSize = sizeRx;
    DMA_InitStructureRx.DMA_PeripheralBaseAddr = (uint32_t)&bss.USARTx->RDR;
    DMA_InitStructureRx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructureRx.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructureRx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructureRx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructureRx.DMA_Mode = DMA_Mode_Circular;// or Circular
    DMA_InitStructureRx.DMA_Priority = DMA_Priority_High;
    DMA_Init(bss.chRx, &DMA_InitStructureRx);

    // later we can enable callback interrupts when a new message has arrived
    // DMA_ITConfig(bss.chRx, DMA_IT_TC, ENABLE);
    
    DMA_Cmd(bss.chRx, ENABLE);

    USART_DMACmd(bss.USARTx, USART_DMAReq_Rx, ENABLE);
  }
}

void BulkSerial::write() {
  if (!enabled) return;

  DMA_Cmd(bss.chTx, DISABLE);
  DMA_Init(bss.chTx, &DMA_InitStructureTx);
  DMA_Cmd(bss.chTx, ENABLE);
}

int BulkSerial::received(uint16_t alignment, uint8_t *dest) {
  if (DMA_GetFlagStatus(bss.flagRxTc)) {
    DMA_ClearFlag(bss.flagRxTc);
    // check alignment
    uint8_t align2 = alignment >> 8;
    uint8_t align1 = alignment & 0xff;
    for (int i=0; i<sizeRx; ++i) {
      // check for alignment in different indices into the localBuffer
      if (localBuf[(curLocalBufIndex+i) % sizeRx] == align1 && localBuf[(curLocalBufIndex+i+1) % sizeRx] == align2) {
        // found right alignment
        for (int j=0; j<sizeRx; ++j) {
          dest[j] = localBuf[(curLocalBufIndex+i+j) % sizeRx];
        }
        curLocalBufIndex = curLocalBufIndex+i;
        return 1;
      }
    }
    // nothing aligned
    return -1;
  }
  return 0;
}

void BulkSerial::initOpenLog(const char *header, const char *fmt) {
  delay(500);
  for (int i = 0; header[i] != 0; i++){
    while (USART_GetFlagStatus(bss.USARTx, USART_FLAG_TXE) == RESET);
    USART_SendData(bss.USARTx, header[i]);
  }
  while (USART_GetFlagStatus(bss.USARTx, USART_FLAG_TXE) == RESET);
  USART_SendData(bss.USARTx, '\n');
  for (int i = 0; fmt[i] != 0; i++){
    while (USART_GetFlagStatus(bss.USARTx, USART_FLAG_TXE) == RESET);
    USART_SendData(bss.USARTx, fmt[i]);
  }
  while (USART_GetFlagStatus(bss.USARTx, USART_FLAG_TXE) == RESET);
  USART_SendData(bss.USARTx, '\n');
}
