/*
 * Some constants, hardware definitions and comments taken from ST's HAL driver
 * library, COPYRIGHT(c) 2015 STMicroelectronics.
 */

/*
 * FreeRTOS+TCP V2.0.11
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_DNS.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"

#include "gd32f4xx.h"
#include "gd32f4xx_enet_eval.h"

/* Interrupt events to process.  Currently only the Rx event is processed
although code for other events is included to allow for possible future
expansion. */
#define EMAC_IF_RX_EVENT        1UL
#define EMAC_IF_ERR_EVENT       4UL
#define EMAC_IF_ALL_EVENT       ( EMAC_IF_RX_EVENT | EMAC_IF_ERR_EVENT )

#ifndef niEMAC_HANDLER_TASK_PRIORITY
#define niEMAC_HANDLER_TASK_PRIORITY	configMAX_PRIORITIES - 1
#endif

#define ipFRAGMENT_OFFSET_BIT_MASK		( ( uint16_t ) 0x0fff ) /* The bits in the two byte IP header field that make up the fragment offset value. */

/* Default the size of the stack used by the EMAC deferred handler task to twice
the size of the stack used by the idle task - but allow this to be overridden in
FreeRTOSConfig.h as configMINIMAL_STACK_SIZE is a user definable constant. */
#ifndef configEMAC_TASK_STACK_SIZE
#define configEMAC_TASK_STACK_SIZE ( 4 * configMINIMAL_STACK_SIZE )
#endif

/*-----------------------------------------------------------*/

/* ENET RxDMA/TxDMA descriptor */
extern enet_descriptors_struct  rxdesc_tab[ENET_RXBUF_NUM], txdesc_tab[ENET_TXBUF_NUM];

/* ENET receive buffer  */
extern uint8_t rx_buff[ENET_RXBUF_NUM][ENET_RXBUF_SIZE]; 

/* ENET transmit buffer */
extern uint8_t tx_buff[ENET_TXBUF_NUM][ENET_TXBUF_SIZE]; 

/*global transmit and receive descriptors pointers */
extern enet_descriptors_struct  *dma_current_txdesc;
extern enet_descriptors_struct  *dma_current_rxdesc;

/*
 * A deferred interrupt handler task that processes
 */
static void prvEMACHandlerTask(void *pvParameters);

/*
 * See if there is a new packet and forward it to the IP-task.
 */
static BaseType_t prvNetworkInterfaceInput(void);

#if( ipconfigUSE_LLMNR != 0 )
/*
 * For LLMNR, an extra MAC-address must be configured to
 * be able to receive the multicast messages.
 */
static void prvMACAddressConfig(ETH_HandleTypeDef *heth, uint32_t ulIndex, uint8_t *Addr);
#endif

/*-----------------------------------------------------------*/

/* Bit map of outstanding ETH interrupt events for processing.  Currently only
the Rx interrupt is handled, although code is included for other events to
enable future expansion. */
static volatile uint32_t ulISREvents;

#if( ipconfigUSE_LLMNR == 1 )
static const uint8_t xLLMNR_MACAddress[] = { 0x01, 0x00, 0x5E, 0x00, 0x00, 0xFC };
#endif

/* Holds the handle of the task used as a deferred interrupt processor.  The
handle is used so direct notifications can be sent to the task for all EMAC/DMA
related interrupts. */
static TaskHandle_t xEMACTaskHandle = NULL;

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceInitialise(void)
{
    BaseType_t xResult = pdPASS;
    
    enet_system_setup();
    enet_mac_address_set(ENET_MAC_ADDRESS0, (uint8_t *)FreeRTOS_GetMACAddress());
    
#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
    enet_ptp_enhanced_descriptors_chain_init(ENET_DMA_TX);
    enet_ptp_enhanced_descriptors_chain_init(ENET_DMA_RX);
#else

    enet_descriptors_chain_init(ENET_DMA_TX);
    enet_descriptors_chain_init(ENET_DMA_RX);

#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */

    /* enable ethernet Rx interrrupt */
    {   int i;
        for(i=0; i<ENET_RXBUF_NUM; i++){ 
           enet_rx_desc_immediate_receive_complete_interrupt(&rxdesc_tab[i]);
        }
    }

#ifdef CHECKSUM_BY_HARDWARE
    /* enable the TCP, UDP and ICMP checksum insertion for the Tx frames */
    for(i=0; i < ENET_TXBUF_NUM; i++){
        enet_transmit_checksum_config(&txdesc_tab[i], ENET_CHECKSUM_TCPUDPICMP_FULL);
    }
#endif /* CHECKSUM_BY_HARDWARE */
    
#if( ipconfigUSE_LLMNR != 0 )
        {
            /* Program the LLMNR address at index 1. */
            enet_mac_address_set(ENET_MAC_ADDRESS1, (uint8_t *) xLLMNR_MACAddress);
            enet_address_filter_enable(ENET_MAC_ADDRESS1);
        }
#endif

    /* The deferred interrupt handler task is created at the highest
    possible priority to ensure the interrupt handler can return directly
    to it.  The task's handle is stored in xEMACTaskHandle so interrupts can
    notify the task when there is something to process. */
    xTaskCreate(prvEMACHandlerTask, "EMAC", configEMAC_TASK_STACK_SIZE, NULL,
                niEMAC_HANDLER_TASK_PRIORITY, &xEMACTaskHandle);
    
    enet_enable();
    
    return xResult;
}
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/

BaseType_t xNetworkInterfaceOutput(NetworkBufferDescriptor_t *const pxDescriptor,
                                   BaseType_t bReleaseAfterSend)
{
    uint8_t *buffer;
    uint32_t ulTransmitSize = 0;
    BaseType_t xReturn = pdFAIL;
    
    ulTransmitSize = pxDescriptor->xDataLength;

    if(ulTransmitSize > ENET_TXBUF_SIZE) {
        ulTransmitSize = ENET_TXBUF_SIZE;
    }
    
    while((uint32_t)RESET != (dma_current_txdesc->status & ENET_TDES0_DAV)){
    }  
    
    buffer = (uint8_t *)(enet_desc_information_get(dma_current_txdesc, TXDESC_BUFFER_1_ADDR));
    
    /* Copy the bytes. */
    memcpy((void *) buffer, pxDescriptor->pucEthernetBuffer, ulTransmitSize);

    /* note: padding and CRC for transmitted frame 
       are automatically inserted by DMA */

    /* transmit descriptors to give to DMA */ 
#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
    ENET_NOCOPY_PTPFRAME_TRANSMIT_ENHANCED_MODE(framelength, NULL);
  
#else
    ENET_NOCOPY_FRAME_TRANSMIT(ulTransmitSize);
#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */
    
    iptraceNETWORK_INTERFACE_TRANSMIT();
    xReturn = pdPASS; 
    
    /* The buffer has been sent so can be released. */
    if(bReleaseAfterSend != pdFALSE) {
        vReleaseNetworkBufferAndDescriptor(pxDescriptor);
    }

    return xReturn;
}
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/

static BaseType_t prvNetworkInterfaceInput(void)
{
    NetworkBufferDescriptor_t *pxNewDescriptor = NULL;
    xIPStackEvent_t xRxEvent = { eNetworkRxEvent, NULL };
    const TickType_t xDescriptorWaitTime = pdMS_TO_TICKS(250);
    uint8_t *pucBuffer;
    BaseType_t xReturn = pdFAIL;
    BaseType_t xReceivedLength;

    /* obtain the size of the packet and put it into the "len" variable. */
    xReceivedLength = enet_desc_information_get(dma_current_rxdesc, RXDESC_FRAME_LENGTH);
    pucBuffer = (uint8_t *)(enet_desc_information_get(dma_current_rxdesc, RXDESC_BUFFER_1_ADDR));
    
    if(xReceivedLength > 0){
        pxNewDescriptor = pxGetNetworkBufferWithDescriptor(ENET_RXBUF_SIZE, xDescriptorWaitTime);
        if(pxNewDescriptor == NULL) {
            /* A new descriptor can not be allocated now. This packet will be dropped. */
            while(1);
        }else{
            memcpy(pxNewDescriptor->pucEthernetBuffer, pucBuffer, xReceivedLength);
            pxNewDescriptor->xDataLength = xReceivedLength;
            xRxEvent.pvData = (void *) pxNewDescriptor;

            /* Pass the data to the TCP/IP task for processing. */
            if(xSendEventStructToIPTask(&xRxEvent, xDescriptorWaitTime) == pdFALSE) {
                /* Could not send the descriptor into the TCP/IP stack, it
                must be released. */
                vReleaseNetworkBufferAndDescriptor(pxNewDescriptor);
                iptraceETHERNET_RX_EVENT_LOST();
                while(1);
            } else {
                iptraceNETWORK_INTERFACE_RECEIVE();
                xReturn = pdPASS;
            }
        }
        
#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
        ENET_NOCOPY_PTPFRAME_RECEIVE_ENHANCED_MODE(NULL);
#else
        ENET_NOCOPY_FRAME_RECEIVE();
#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */
        
    }
    
    return xReturn;
}
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/

/* Uncomment this in case BufferAllocation_1.c is used. */

#define niBUFFER_1_PACKET_SIZE		1536

void vNetworkInterfaceAllocateRAMToBuffers(NetworkBufferDescriptor_t
        pxNetworkBuffers[ ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS ])
{
    static uint8_t
    ucNetworkPackets[ ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS * niBUFFER_1_PACKET_SIZE ] __attribute__((
                aligned(32)));
    uint8_t *ucRAMBuffer = ucNetworkPackets;
    uint32_t ul;

    for(ul = 0; ul < ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS; ul++) {
        pxNetworkBuffers[ ul ].pucEthernetBuffer = ucRAMBuffer + ipBUFFER_PADDING;
        *((unsigned *) ucRAMBuffer) = (unsigned)(&(pxNetworkBuffers[ ul ]));
        ucRAMBuffer += niBUFFER_1_PACKET_SIZE;
    }
}
/*-----------------------------------------------------------*/

static void prvEMACHandlerTask(void *pvParameters)
{
    UBaseType_t uxLastMinBufferCount = 0;
#if( ipconfigCHECK_IP_QUEUE_SPACE != 0 )
    UBaseType_t uxLastMinQueueSpace = 0;
#endif
    UBaseType_t uxCurrentCount;
    BaseType_t xResult;
    const TickType_t ulMaxBlockTime = pdMS_TO_TICKS(100UL);
    /* Remove compiler warnings about unused parameters. */
    (void) pvParameters;

    for(;;) {
        xResult = 0;
        uxCurrentCount = uxGetMinimumFreeNetworkBuffers();

        if(uxLastMinBufferCount != uxCurrentCount) {
            /* The logging produced below may be helpful
            while tuning +TCP: see how many buffers are in use. */
            uxLastMinBufferCount = uxCurrentCount;
            FreeRTOS_printf(("Network buffers: %lu lowest %lu\n",
                             uxGetNumberOfFreeNetworkBuffers(), uxCurrentCount));
        }

#if( ipconfigCHECK_IP_QUEUE_SPACE != 0 )
        {
            uxCurrentCount = uxGetMinimumIPQueueSpace();

            if(uxLastMinQueueSpace != uxCurrentCount) {
                /* The logging produced below may be helpful
                while tuning +TCP: see how many buffers are in use. */
                uxLastMinQueueSpace = uxCurrentCount;
                FreeRTOS_printf(("Queue space: lowest %lu\n", uxCurrentCount));
            }
        }
#endif /* ipconfigCHECK_IP_QUEUE_SPACE */

        if((ulISREvents & EMAC_IF_ALL_EVENT) == 0) {
            /* No events to process now, wait for the next. */
            ulTaskNotifyTake(pdFALSE, ulMaxBlockTime);
        }

        if((ulISREvents & EMAC_IF_RX_EVENT) != 0) {
            ulISREvents &= ~EMAC_IF_RX_EVENT;
            xResult = prvNetworkInterfaceInput();

            if(xResult > 0) {
                while(prvNetworkInterfaceInput() > 0) {
                }
            }
        }

        if((ulISREvents & EMAC_IF_ERR_EVENT) != 0) {
            /* Future extension: logging about errors that occurred. */
            ulISREvents &= ~EMAC_IF_ERR_EVENT;
        }
    }
}
/*-----------------------------------------------------------*/

void ENET_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    /* frame received */
    if(SET == enet_interrupt_flag_get(ENET_DMA_INT_FLAG_RS)){
        /* Ethernet RX-Complete callback function, elsewhere declared as weak. */
        ulISREvents |= EMAC_IF_RX_EVENT;
        /* Wakeup the prvEMACHandlerTask. */
        if(xEMACTaskHandle != NULL) {
            vTaskNotifyGiveFromISR(xEMACTaskHandle, &xHigherPriorityTaskWoken);
        }
    }
    
    /* clear the enet DMA Rx interrupt pending bits */
    enet_interrupt_flag_clear(ENET_DMA_INT_FLAG_RS_CLR);
    enet_interrupt_flag_clear(ENET_DMA_INT_FLAG_NI_CLR);
    
    /* switch tasks if necessary */
    if(pdFALSE != xHigherPriorityTaskWoken){
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
