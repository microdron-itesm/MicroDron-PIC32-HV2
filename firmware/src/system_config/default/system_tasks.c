/*******************************************************************************
 System Tasks File

  File Name:
    system_tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled state
    machines.

  Description:
    This file contains source code necessary to maintain system's polled state
    machines.  It implements the "SYS_Tasks" function that calls the individual
    "Tasks" functions for all the MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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

#include "system_config.h"
#include "system_definitions.h"
#include "app.h"
#include "mavlink_recv_task.h"
#include "mavlink_send_task.h"
#include "mavlink_status_task.h"
#include "att_controller_task.h"
#include "serialhandler.h"
#include "imu_update_task.h"


// *****************************************************************************
// *****************************************************************************
// Section: Local Prototypes
// *****************************************************************************
// *****************************************************************************


 
static void _SYS_Tasks ( void );
 
 
static void _APP_Tasks(void);
static void _MAVLINK_RECV_TASK_Tasks(void);
static void _MAVLINK_SEND_TASK_Tasks(void);
static void _MAVLINK_STATUS_TASK_Tasks(void);
static void _ATT_CONTROLLER_TASK_Tasks(void);
static void _SERIALHANDLER_Tasks(void);
static void _IMU_UPDATE_TASK_Tasks(void);


// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/

void SYS_Tasks ( void )
{
    /* Create OS Thread for Sys Tasks. */
    xTaskCreate((TaskFunction_t) _SYS_Tasks,
                "Sys Tasks",
                1024, NULL, 1, NULL);

 
 
    /* Create OS Thread for APP Tasks. */
    xTaskCreate((TaskFunction_t) _APP_Tasks,
                "APP Tasks",
                1024, NULL, 0, NULL);

    /* Create OS Thread for MAVLINK_RECV_TASK Tasks. */
    xTaskCreate((TaskFunction_t) _MAVLINK_RECV_TASK_Tasks,
                "MAVLINK_RECV_TASK Tasks",
                32768, NULL, 5, NULL);

    /* Create OS Thread for MAVLINK_SEND_TASK Tasks. */
    xTaskCreate((TaskFunction_t) _MAVLINK_SEND_TASK_Tasks,
                "MAVLINK_SEND_TASK Tasks",
                32768, NULL, 4, NULL);

    /* Create OS Thread for MAVLINK_STATUS_TASK Tasks. */
    xTaskCreate((TaskFunction_t) _MAVLINK_STATUS_TASK_Tasks,
                "MAVLINK_STATUS_TASK Tasks",
                8192, NULL, 3, NULL);

    /* Create OS Thread for ATT_CONTROLLER_TASK Tasks. */
    xTaskCreate((TaskFunction_t) _ATT_CONTROLLER_TASK_Tasks,
                "ATT_CONTROLLER_TASK Tasks",
                8192, NULL, 7, NULL);

    /* Create OS Thread for SERIALHANDLER Tasks. */
    xTaskCreate((TaskFunction_t) _SERIALHANDLER_Tasks,
                "SERIALHANDLER Tasks",
                1024, NULL, 5, NULL);

    /* Create OS Thread for IMU_UPDATE_TASK Tasks. */
    xTaskCreate((TaskFunction_t) _IMU_UPDATE_TASK_Tasks,
                "IMU_UPDATE_TASK Tasks",
                1024, NULL, 5, NULL);

    /**************
     * Start RTOS * 
     **************/
    vTaskStartScheduler(); /* This function never returns. */
}


/*******************************************************************************
  Function:
    void _SYS_Tasks ( void )

  Summary:
    Maintains state machines of system modules.
*/
static void _SYS_Tasks ( void)
{
    while(1)
    {
        /* Maintain system services */
        SYS_DEVCON_Tasks(sysObj.sysDevcon);

        /* Maintain Device Drivers */
 
 

        /* Maintain Middleware */

        /* Task Delay */
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

 
 

/*******************************************************************************
  Function:
    void _APP_Tasks ( void )

  Summary:
    Maintains state machine of APP.
*/

static void _APP_Tasks(void)
{
    while(1)
    {
        APP_Tasks();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
  Function:
    void _MAVLINK_RECV_TASK_Tasks ( void )

  Summary:
    Maintains state machine of MAVLINK_RECV_TASK.
*/

static void _MAVLINK_RECV_TASK_Tasks(void)
{
    while(1)
    {
        MAVLINK_RECV_TASK_Tasks();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
  Function:
    void _MAVLINK_SEND_TASK_Tasks ( void )

  Summary:
    Maintains state machine of MAVLINK_SEND_TASK.
*/

static void _MAVLINK_SEND_TASK_Tasks(void)
{
    while(1)
    {
        MAVLINK_SEND_TASK_Tasks();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
  Function:
    void _MAVLINK_STATUS_TASK_Tasks ( void )

  Summary:
    Maintains state machine of MAVLINK_STATUS_TASK.
*/

static void _MAVLINK_STATUS_TASK_Tasks(void)
{
    while(1)
    {
        MAVLINK_STATUS_TASK_Tasks();
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
  Function:
    void _ATT_CONTROLLER_TASK_Tasks ( void )

  Summary:
    Maintains state machine of ATT_CONTROLLER_TASK.
*/

static void _ATT_CONTROLLER_TASK_Tasks(void)
{
    while(1)
    {
        ATT_CONTROLLER_TASK_Tasks();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
  Function:
    void _SERIALHANDLER_Tasks ( void )

  Summary:
    Maintains state machine of SERIALHANDLER.
*/

static void _SERIALHANDLER_Tasks(void)
{
    while(1)
    {
        SERIALHANDLER_Tasks();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
  Function:
    void _IMU_UPDATE_TASK_Tasks ( void )

  Summary:
    Maintains state machine of IMU_UPDATE_TASK.
*/

static void _IMU_UPDATE_TASK_Tasks(void)
{
    while(1)
    {
        IMU_UPDATE_TASK_Tasks();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
 End of File
 */
