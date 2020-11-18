/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    serialhandler.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "serialhandler.h"
#include "driver/usart/drv_usart.h"
#include "driver/usart/drv_usart_definitions.h"
#include "driver/driver_common.h"
#include <FreeRTOS.h>
#include <queue.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

SERIALHANDLER_DATA serialhandlerData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
int i;
static DRV_HANDLE espHandle, imuHandle;
extern QueueHandle_t g_espSerialByteQueue;
extern QueueHandle_t g_imuSerialByteQueue;
static volatile TickType_t lastRecv;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

static void timerCallback(uintptr_t context, uint32_t alarmCount){
    uint8_t byte = 0;
    if(g_espSerialByteQueue){
        ESP_INTToggle();
        while(!DRV_USART0_ReceiverBufferIsEmpty()){
            ESP_INTToggle();
            byte = PLIB_USART_ReceiverByteReceive(USART_ID_1);
            SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_RECEIVE);
            xQueueSendToBackFromISR(g_espSerialByteQueue, &byte, 0);
            //DRV_USART0_WriteByte(byte);
        }
    }

    if(g_imuSerialByteQueue){
        IMU_INTToggle();
        while(!DRV_USART1_ReceiverBufferIsEmpty()){
            IMU_INTToggle();
            byte = PLIB_USART_ReceiverByteReceive(USART_ID_3);
            SYS_INT_SourceStatusClear(INT_SOURCE_USART_3_RECEIVE);
            xQueueSendToBackFromISR(g_imuSerialByteQueue, &byte, 0);
        }
    }
    
    lastRecv = xTaskGetTickCountFromISR();
}

void uartCallback(const SYS_MODULE_INDEX index){
    uint8_t byte = 0;
    
//    switch(index){
//        case DRV_USART_INDEX_0:
//            ESP_INTToggle();
//            byte = PLIB_USART_ReceiverByteReceive(USART_ID_1);
//            SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_RECEIVE);
//            xQueueSendToBackFromISR(g_espSerialByteQueue, &byte, 0);
//            break;
//        case DRV_USART_INDEX_1:
//            IMU_INTToggle();
//            byte = PLIB_USART_ReceiverByteReceive(USART_ID_3);
//            SYS_INT_SourceStatusClear(INT_SOURCE_USART_3_RECEIVE);
//            xQueueSendToBackFromISR(g_imuSerialByteQueue, &byte, 0);
//            break;
//    }
    
        switch(index){
        case DRV_USART_INDEX_0:
            while(!DRV_USART0_ReceiverBufferIsEmpty()){
                ESP_INTToggle();
                uint8_t byte = DRV_USART0_ReadByte();
                xQueueSendToBackFromISR(g_espSerialByteQueue, &byte, 0);
                //DRV_USART0_WriteByte(byte);
            }
            break;
        case DRV_USART_INDEX_1:
            while(!DRV_USART1_ReceiverBufferIsEmpty()){
                IMU_INTToggle();
                uint8_t byte = DRV_USART1_ReadByte();
                xQueueSendToBackFromISR(g_imuSerialByteQueue, &byte, 0);
            }
            break;
    }
    
    lastRecv = xTaskGetTickCountFromISR();
}

/*******************************************************************************
  Function:
    void SERIALHANDLER_Initialize ( void )

  Remarks:
    See prototype in serialhandler.h.
 */

void SERIALHANDLER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    serialhandlerData.state = SERIALHANDLER_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    hal_comms_init(NULL, 0);
    imu_comms_init();
}


/******************************************************************************
  Function:
    void SERIALHANDLER_Tasks ( void )

  Remarks:
    See prototype in serialhandler.h.
 */

void SERIALHANDLER_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( serialhandlerData.state )
    {
        /* Application's initial state. */
        case SERIALHANDLER_STATE_INIT:
        {
            bool appInitialized = true;
            
            espHandle = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_EXCLUSIVE);
            imuHandle = DRV_USART_Open(DRV_USART_INDEX_1, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_EXCLUSIVE);
            
            //DRV_USART0_ByteReceiveCallbackSet(uartCallback);
            //DRV_USART1_ByteReceiveCallbackSet(uartCallback);
            
            ESP_INTToggle();
            IMU_INTToggle();

            while(!DRV_TMR1_AlarmRegister(4000, true, NULL, timerCallback));
            DRV_TMR1_AlarmEnable(true);
            DRV_TMR1_Start();
        
            if (appInitialized)
            {
            
                serialhandlerData.state = SERIALHANDLER_STATE_SERVICE_TASKS;
            }
            break;
        }

        case SERIALHANDLER_STATE_SERVICE_TASKS:
        {
            if(xTaskGetTickCount() - lastRecv > pdMS_TO_TICKS(10)){
                ESP_INTToggle();
                IMU_INTToggle();
            }
            
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
