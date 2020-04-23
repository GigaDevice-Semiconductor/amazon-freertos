/*!
    \file  main.c
    \brief enet demo 

    \version 2016-08-15, V1.0.0, demo for GD32F4xx
    \version 2018-12-12, V2.0.0, demo for GD32F4xx
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f4xx.h"
#include "main.h"
#include "gd32f450z_eval.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

#include <stdio.h>

#define INIT_TASK_PRIO   ( tskIDLE_PRIORITY + 1 )
#define DHCP_TASK_PRIO   ( tskIDLE_PRIORITY + 4 )
#define LED_TASK_PRIO    ( tskIDLE_PRIORITY + 2 )

//uint8_t ucMACAddress[ 6 ] =
//{
//    configMAC_ADDR0,
//    configMAC_ADDR1,
//    configMAC_ADDR2,
//    configMAC_ADDR3,
//    configMAC_ADDR4,
//    configMAC_ADDR5
//};
extern uint8_t ucMACAddress[ ipMAC_ADDRESS_LENGTH_BYTES ];

/* The default IP and MAC address used by the demo.  The address configuration
 * defined here will be used if ipconfigUSE_DHCP is 0, or if ipconfigUSE_DHCP is
 * 1 but a DHCP server could not be contacted.  See the online documentation for
 * more information.  In both cases the node can be discovered using
 * "ping RTOSDemo". */
static const uint8_t ucIPAddress[ 4 ] =
{
    configIP_ADDR0,
    configIP_ADDR1,
    configIP_ADDR2,
    configIP_ADDR3
};
static const uint8_t ucNetMask[ 4 ] =
{
    configNET_MASK0,
    configNET_MASK1,
    configNET_MASK2,
    configNET_MASK3
};
static const uint8_t ucGatewayAddress[ 4 ] =
{
    configGATEWAY_ADDR0,
    configGATEWAY_ADDR1,
    configGATEWAY_ADDR2,
    configGATEWAY_ADDR3
};
static const uint8_t ucDNSServerAddress[ 4 ] =
{
    configDNS_SERVER_ADDR0,
    configDNS_SERVER_ADDR1,
    configDNS_SERVER_ADDR2,
    configDNS_SERVER_ADDR3
};

static UBaseType_t ulNextRand;
extern struct netif g_mynetif;
 
void led_task(void * pvParameters); 
void init_task(void * pvParameters);
void uasrt_transmit(char *str, uint32_t len);
void vMyUSARTOutput(char *str);

void uasrt_transmit(char *str, uint32_t len)
{
    while(len != 0){
        usart_flag_clear(USART0, USART_FLAG_TC);
        usart_data_transmit(USART0, (uint32_t)*str);
        while(RESET == usart_flag_get(USART0, USART_FLAG_TC));
        usart_flag_clear(USART0, USART_FLAG_TC);
        len--;
        str++;
    }
}

void vMyUSARTOutput(char *str)
{
    uasrt_transmit(str, strlen(str));
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* configure 4 bits pre-emption priority */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    /* init task */
    xTaskCreate(init_task, "INIT", configMINIMAL_STACK_SIZE * 2, NULL, INIT_TASK_PRIO, NULL);
    
    /* start scheduler */
    vTaskStartScheduler();

    while(1){
    }
}

/*!
    \brief      init task
    \param[in]  pvParameters not used
    \param[out] none
    \retval     none
*/
void init_task(void * pvParameters)
{
    gd_eval_com_init(EVAL_COM0);
    gd_eval_led_init(LED3);

    FreeRTOS_IPInit( ucIPAddress,
                 ucNetMask,
                 ucGatewayAddress,
                 ucDNSServerAddress,
                 ucMACAddress );
    /* start toogle LED task every 250ms */
    xTaskCreate(led_task, "LED", configMINIMAL_STACK_SIZE, NULL, LED_TASK_PRIO, NULL);

    for( ;; ){
        vTaskDelete(NULL);
    }
}

/*!
    \brief      led task
    \param[in]  pvParameters not used
    \param[out] none
    \retval     none
*/
void led_task(void * pvParameters)
{  
    for( ;; ){
        /* toggle LED3 each 250ms */
        gd_eval_led_toggle(LED3);
        vTaskDelay(250);
    }
}

void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
    uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
    char cBuffer[ 16 ];


    /* If the network has just come up...*/
    if( eNetworkEvent == eNetworkUp )
    {
        /* Print out the network configuration, which may have come from a DHCP
         * server. */
        FreeRTOS_GetAddressConfiguration(
            &ulIPAddress,
            &ulNetMask,
            &ulGatewayAddress,
            &ulDNSServerAddress );
        FreeRTOS_inet_ntoa( ulIPAddress, cBuffer );
        FreeRTOS_printf( ( "\r\n\r\nIP Address: %s\r\n", cBuffer ) );

        FreeRTOS_inet_ntoa( ulNetMask, cBuffer );
        FreeRTOS_printf( ( "Subnet Mask: %s\r\n", cBuffer ) );

        FreeRTOS_inet_ntoa( ulGatewayAddress, cBuffer );
        FreeRTOS_printf( ( "Gateway Address: %s\r\n", cBuffer ) );

        FreeRTOS_inet_ntoa( ulDNSServerAddress, cBuffer );
        FreeRTOS_printf( ( "DNS Server Address: %s\r\n\r\n\r\n", cBuffer ) );
    }
}
#if 1
/*!
    \brief      check whether the TRNG module is ready
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus trng_ready_check(void)
{
    uint32_t timeout = 0;
    FlagStatus trng_flag = RESET;
    ErrStatus reval = SUCCESS;
    
    /* check wherther the random data is valid */
    do{
        timeout++;
        trng_flag = trng_flag_get(TRNG_FLAG_DRDY);
    }while((RESET == trng_flag) &&(0xFFFF > timeout));
    
    if(RESET == trng_flag)
    {   
        /* ready check timeout */
        printf("Error: TRNG can't ready \r\n");
        trng_flag = trng_flag_get(TRNG_FLAG_CECS);
        printf("Clock error current status: %d \r\n", trng_flag);
        trng_flag = trng_flag_get(TRNG_FLAG_SECS);
        printf("Seed error current status: %d \r\n", trng_flag);  
        reval = ERROR;
    }
    
    /* return check status */
    return reval;
}


/*!
    \brief      configure TRNG module
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus trng_configuration(void)
{
    ErrStatus reval = SUCCESS;
    
    /* TRNG module clock enable */
    rcu_periph_clock_enable(RCU_TRNG);
    
    /* TRNG registers reset */
    trng_deinit();
    trng_enable();
    /* check TRNG work status */
    reval = trng_ready_check();
    
    return reval;
}

UBaseType_t uxRand(void)
{
    uint32_t random_data = 0;
    uint8_t retry = 0;
    /* configure TRNG module */
    while((ERROR == trng_configuration()) && retry < 3){
        printf("TRNG init fail \r\n");
        printf("TRNG init retry \r\n");
        retry++;
    }
    random_data = trng_get_true_random_data();
    return (UBaseType_t)random_data;
}
#endif
#if 0
UBaseType_t uxRand( void )
{
const uint32_t ulMultiplier = 0x015a4e35UL, ulIncrement = 1UL;

	/*
	 * Utility function to generate a pseudo random number.
	 *
	 * !!!NOTE!!!
	 * This is not a secure method of generating a random number.  Production
	 * devices should use a True Random Number Generator (TRNG).
	 */
	ulNextRand = ( ulMultiplier * ulNextRand ) + ulIncrement;
	return( ( int ) ( ulNextRand >> 16UL ) & 0x7fffUL );
}
#endif

/*
 * Callback that provides the inputs necessary to generate a randomized TCP
 * Initial Sequence Number per RFC 6528.  THIS IS ONLY A DUMMY IMPLEMENTATION
 * THAT RETURNS A PSEUDO RANDOM NUMBER SO IS NOT INTENDED FOR USE IN PRODUCTION
 * SYSTEMS.
 */
//extern uint32_t ulApplicationGetNextSequenceNumber( uint32_t ulSourceAddress,
//													uint16_t usSourcePort,
//													uint32_t ulDestinationAddress,
//													uint16_t usDestinationPort )
//{
//	( void ) ulSourceAddress;
//	( void ) usSourcePort;
//	( void ) ulDestinationAddress;
//	( void ) usDestinationPort;

//	return uxRand();
//}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM0, (uint8_t) ch);
    while (RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE));
    return ch;
}
