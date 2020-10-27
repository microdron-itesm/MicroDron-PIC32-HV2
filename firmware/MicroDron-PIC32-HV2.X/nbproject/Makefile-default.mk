#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron-PIC32-HV2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron-PIC32-HV2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../MicroDrone-Firmware/HAL/PIC_HV2/imu.c ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c ../MicroDrone-Firmware/libs/Math/Quaternion.c ../MicroDrone-Firmware/libs/Math/Vector3D.c ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/general_exception_handler.c ../src/system_config/default/general-exception-context.S ../src/system_config/default/fassert.c ../src/system_config/default/system_tasks.c ../src/system_config/default/system_interrupt_a.S ../src/system_config/default/rtos_hooks.c ../src/main.c ../src/app.c ../src/mavlink_recv_task.c ../src/mavlink_send_task.c ../src/mavlink_status_task.c ../src/att_controller_task.c ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c ../MicroDrone-Firmware/src/Control/PID/PID.c ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c ../MicroDrone-Firmware/src/Control/SensFusion.c ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c ../MicroDrone-Firmware/src/Control/AttitudeController.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c ../MicroDrone-Firmware/src/Utils/num.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1650642999/imu.o ${OBJECTDIR}/_ext/1650642999/comms.o ${OBJECTDIR}/_ext/1650642999/tof.o ${OBJECTDIR}/_ext/1650642999/motors.o ${OBJECTDIR}/_ext/971049567/Quaternion.o ${OBJECTDIR}/_ext/971049567/Vector3D.o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o ${OBJECTDIR}/_ext/1688732426/general-exception-context.o ${OBJECTDIR}/_ext/1688732426/fassert.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ${OBJECTDIR}/_ext/457403440/heap_1.o ${OBJECTDIR}/_ext/1571139743/port.o ${OBJECTDIR}/_ext/1571139743/port_asm.o ${OBJECTDIR}/_ext/1276567923/croutine.o ${OBJECTDIR}/_ext/1276567923/list.o ${OBJECTDIR}/_ext/1276567923/queue.o ${OBJECTDIR}/_ext/1276567923/tasks.o ${OBJECTDIR}/_ext/1276567923/timers.o ${OBJECTDIR}/_ext/1276567923/event_groups.o ${OBJECTDIR}/_ext/443942460/PID.o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ${OBJECTDIR}/_ext/1313821444/num.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1650642999/imu.o.d ${OBJECTDIR}/_ext/1650642999/comms.o.d ${OBJECTDIR}/_ext/1650642999/tof.o.d ${OBJECTDIR}/_ext/1650642999/motors.o.d ${OBJECTDIR}/_ext/971049567/Quaternion.o.d ${OBJECTDIR}/_ext/971049567/Vector3D.o.d ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d ${OBJECTDIR}/_ext/1688732426/system_init.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o.d ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d ${OBJECTDIR}/_ext/1688732426/fassert.o.d ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1360937237/app.o.d ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d ${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d ${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d ${OBJECTDIR}/_ext/121284916/sys_devcon.o.d ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d ${OBJECTDIR}/_ext/457403440/heap_1.o.d ${OBJECTDIR}/_ext/1571139743/port.o.d ${OBJECTDIR}/_ext/1571139743/port_asm.o.d ${OBJECTDIR}/_ext/1276567923/croutine.o.d ${OBJECTDIR}/_ext/1276567923/list.o.d ${OBJECTDIR}/_ext/1276567923/queue.o.d ${OBJECTDIR}/_ext/1276567923/tasks.o.d ${OBJECTDIR}/_ext/1276567923/timers.o.d ${OBJECTDIR}/_ext/1276567923/event_groups.o.d ${OBJECTDIR}/_ext/443942460/PID.o.d ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d ${OBJECTDIR}/_ext/1313821444/num.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1650642999/imu.o ${OBJECTDIR}/_ext/1650642999/comms.o ${OBJECTDIR}/_ext/1650642999/tof.o ${OBJECTDIR}/_ext/1650642999/motors.o ${OBJECTDIR}/_ext/971049567/Quaternion.o ${OBJECTDIR}/_ext/971049567/Vector3D.o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o ${OBJECTDIR}/_ext/1688732426/general-exception-context.o ${OBJECTDIR}/_ext/1688732426/fassert.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ${OBJECTDIR}/_ext/457403440/heap_1.o ${OBJECTDIR}/_ext/1571139743/port.o ${OBJECTDIR}/_ext/1571139743/port_asm.o ${OBJECTDIR}/_ext/1276567923/croutine.o ${OBJECTDIR}/_ext/1276567923/list.o ${OBJECTDIR}/_ext/1276567923/queue.o ${OBJECTDIR}/_ext/1276567923/tasks.o ${OBJECTDIR}/_ext/1276567923/timers.o ${OBJECTDIR}/_ext/1276567923/event_groups.o ${OBJECTDIR}/_ext/443942460/PID.o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ${OBJECTDIR}/_ext/1313821444/num.o

# Source Files
SOURCEFILES=../MicroDrone-Firmware/HAL/PIC_HV2/imu.c ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c ../MicroDrone-Firmware/libs/Math/Quaternion.c ../MicroDrone-Firmware/libs/Math/Vector3D.c ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/general_exception_handler.c ../src/system_config/default/general-exception-context.S ../src/system_config/default/fassert.c ../src/system_config/default/system_tasks.c ../src/system_config/default/system_interrupt_a.S ../src/system_config/default/rtos_hooks.c ../src/main.c ../src/app.c ../src/mavlink_recv_task.c ../src/mavlink_send_task.c ../src/mavlink_status_task.c ../src/att_controller_task.c ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c ../MicroDrone-Firmware/src/Control/PID/PID.c ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c ../MicroDrone-Firmware/src/Control/SensFusion.c ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c ../MicroDrone-Firmware/src/Control/AttitudeController.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c ../MicroDrone-Firmware/src/Utils/num.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron-PIC32-HV2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MZ2048ECM100
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1688732426/general-exception-context.o: ../src/system_config/default/general-exception-context.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general-exception-context.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.ok ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d"  -o ${OBJECTDIR}/_ext/1688732426/general-exception-context.o ../src/system_config/default/general-exception-context.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/general-exception-context.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d" "${OBJECTDIR}/_ext/1688732426/general-exception-context.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o: ../src/system_config/default/system_interrupt_a.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ../src/system_config/default/system_interrupt_a.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1571139743/port_asm.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1571139743" 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o.ok ${OBJECTDIR}/_ext/1571139743/port_asm.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1571139743/port_asm.o.d"  -o ${OBJECTDIR}/_ext/1571139743/port_asm.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1571139743/port_asm.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1571139743/port_asm.o.d" "${OBJECTDIR}/_ext/1571139743/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1688732426/general-exception-context.o: ../src/system_config/default/general-exception-context.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general-exception-context.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.ok ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d"  -o ${OBJECTDIR}/_ext/1688732426/general-exception-context.o ../src/system_config/default/general-exception-context.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/general-exception-context.o.asm.d",--gdwarf-2,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d" "${OBJECTDIR}/_ext/1688732426/general-exception-context.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o: ../src/system_config/default/system_interrupt_a.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ../src/system_config/default/system_interrupt_a.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d",--gdwarf-2,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.asm.d",--gdwarf-2,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1571139743/port_asm.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1571139743" 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o.ok ${OBJECTDIR}/_ext/1571139743/port_asm.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1571139743/port_asm.o.d"  -o ${OBJECTDIR}/_ext/1571139743/port_asm.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1571139743/port_asm.o.asm.d",--gdwarf-2,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1571139743/port_asm.o.d" "${OBJECTDIR}/_ext/1571139743/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1650642999/imu.o: ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1650642999/imu.o.d" -o ${OBJECTDIR}/_ext/1650642999/imu.o ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1650642999/imu.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1650642999/comms.o: ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1650642999/comms.o.d" -o ${OBJECTDIR}/_ext/1650642999/comms.o ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1650642999/comms.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1650642999/tof.o: ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1650642999/tof.o.d" -o ${OBJECTDIR}/_ext/1650642999/tof.o ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1650642999/tof.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1650642999/motors.o: ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1650642999/motors.o.d" -o ${OBJECTDIR}/_ext/1650642999/motors.o ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1650642999/motors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/971049567/Quaternion.o: ../MicroDrone-Firmware/libs/Math/Quaternion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" -o ${OBJECTDIR}/_ext/971049567/Quaternion.o ../MicroDrone-Firmware/libs/Math/Quaternion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/971049567/Vector3D.o: ../MicroDrone-Firmware/libs/Math/Vector3D.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" -o ${OBJECTDIR}/_ext/971049567/Vector3D.o ../MicroDrone-Firmware/libs/Math/Vector3D.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1047219354/drv_oc_static.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/327000265/drv_usart_static.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o: ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/general_exception_handler.o: ../src/system_config/default/general_exception_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1688732426/general_exception_handler.o.d" -o ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o ../src/system_config/default/general_exception_handler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/general_exception_handler.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/fassert.o: ../src/system_config/default/fassert.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1688732426/fassert.o.d" -o ${OBJECTDIR}/_ext/1688732426/fassert.o ../src/system_config/default/fassert.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/fassert.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/rtos_hooks.o: ../src/system_config/default/rtos_hooks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ../src/system_config/default/rtos_hooks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o: ../src/mavlink_recv_task.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ../src/mavlink_recv_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o: ../src/mavlink_send_task.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ../src/mavlink_send_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o: ../src/mavlink_status_task.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ../src/mavlink_status_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/att_controller_task.o: ../src/att_controller_task.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ../src/att_controller_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1353086322/osal_freertos.o: ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353086322" 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/121284916/sys_devcon.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/121284916/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o: ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1317643790" 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/457403440/heap_1.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/457403440" 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o.d 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/457403440/heap_1.o.d" -o ${OBJECTDIR}/_ext/457403440/heap_1.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/457403440/heap_1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1571139743/port.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1571139743" 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1571139743/port.o.d" -o ${OBJECTDIR}/_ext/1571139743/port.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1571139743/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1276567923/croutine.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1276567923/croutine.o.d" -o ${OBJECTDIR}/_ext/1276567923/croutine.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1276567923/croutine.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1276567923/list.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1276567923/list.o.d" -o ${OBJECTDIR}/_ext/1276567923/list.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1276567923/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1276567923/queue.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1276567923/queue.o.d" -o ${OBJECTDIR}/_ext/1276567923/queue.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1276567923/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1276567923/tasks.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1276567923/tasks.o.d" -o ${OBJECTDIR}/_ext/1276567923/tasks.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1276567923/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1276567923/timers.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1276567923/timers.o.d" -o ${OBJECTDIR}/_ext/1276567923/timers.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1276567923/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1276567923/event_groups.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1276567923/event_groups.o.d" -o ${OBJECTDIR}/_ext/1276567923/event_groups.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1276567923/event_groups.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/443942460/PID.o: ../MicroDrone-Firmware/src/Control/PID/PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/443942460" 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o.d 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/443942460/PID.o.d" -o ${OBJECTDIR}/_ext/443942460/PID.o ../MicroDrone-Firmware/src/Control/PID/PID.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/443942460/PID.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o: ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1035051164" 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" -o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o: ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" -o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/SensFusion.o: ../MicroDrone-Firmware/src/Control/SensFusion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" -o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ../MicroDrone-Firmware/src/Control/SensFusion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o: ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" -o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/AttitudeController.o: ../MicroDrone-Firmware/src/Control/AttitudeController.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" -o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ../MicroDrone-Firmware/src/Control/AttitudeController.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/149289479/MAVLinkSender.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o: ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1715884449" 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" -o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o: ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1313821444/num.o: ../MicroDrone-Firmware/src/Utils/num.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1313821444" 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o.d 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1313821444/num.o.d" -o ${OBJECTDIR}/_ext/1313821444/num.o ../MicroDrone-Firmware/src/Utils/num.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1313821444/num.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1650642999/imu.o: ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1650642999/imu.o.d" -o ${OBJECTDIR}/_ext/1650642999/imu.o ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1650642999/imu.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1650642999/comms.o: ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1650642999/comms.o.d" -o ${OBJECTDIR}/_ext/1650642999/comms.o ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1650642999/comms.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1650642999/tof.o: ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1650642999/tof.o.d" -o ${OBJECTDIR}/_ext/1650642999/tof.o ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1650642999/tof.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1650642999/motors.o: ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1650642999/motors.o.d" -o ${OBJECTDIR}/_ext/1650642999/motors.o ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1650642999/motors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/971049567/Quaternion.o: ../MicroDrone-Firmware/libs/Math/Quaternion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" -o ${OBJECTDIR}/_ext/971049567/Quaternion.o ../MicroDrone-Firmware/libs/Math/Quaternion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/971049567/Vector3D.o: ../MicroDrone-Firmware/libs/Math/Vector3D.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" -o ${OBJECTDIR}/_ext/971049567/Vector3D.o ../MicroDrone-Firmware/libs/Math/Vector3D.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1047219354/drv_oc_static.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/327000265/drv_usart_static.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o: ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/general_exception_handler.o: ../src/system_config/default/general_exception_handler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1688732426/general_exception_handler.o.d" -o ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o ../src/system_config/default/general_exception_handler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/general_exception_handler.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/fassert.o: ../src/system_config/default/fassert.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1688732426/fassert.o.d" -o ${OBJECTDIR}/_ext/1688732426/fassert.o ../src/system_config/default/fassert.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/fassert.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/rtos_hooks.o: ../src/system_config/default/rtos_hooks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ../src/system_config/default/rtos_hooks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o: ../src/mavlink_recv_task.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ../src/mavlink_recv_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o: ../src/mavlink_send_task.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ../src/mavlink_send_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o: ../src/mavlink_status_task.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ../src/mavlink_status_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/att_controller_task.o: ../src/att_controller_task.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ../src/att_controller_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1353086322/osal_freertos.o: ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1353086322" 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/121284916/sys_devcon.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/121284916/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o: ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1317643790" 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/457403440/heap_1.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/457403440" 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o.d 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/457403440/heap_1.o.d" -o ${OBJECTDIR}/_ext/457403440/heap_1.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/457403440/heap_1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1571139743/port.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1571139743" 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1571139743/port.o.d" -o ${OBJECTDIR}/_ext/1571139743/port.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1571139743/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1276567923/croutine.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1276567923/croutine.o.d" -o ${OBJECTDIR}/_ext/1276567923/croutine.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1276567923/croutine.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1276567923/list.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1276567923/list.o.d" -o ${OBJECTDIR}/_ext/1276567923/list.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1276567923/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1276567923/queue.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1276567923/queue.o.d" -o ${OBJECTDIR}/_ext/1276567923/queue.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1276567923/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1276567923/tasks.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1276567923/tasks.o.d" -o ${OBJECTDIR}/_ext/1276567923/tasks.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1276567923/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1276567923/timers.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1276567923/timers.o.d" -o ${OBJECTDIR}/_ext/1276567923/timers.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1276567923/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1276567923/event_groups.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1276567923/event_groups.o.d" -o ${OBJECTDIR}/_ext/1276567923/event_groups.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1276567923/event_groups.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/443942460/PID.o: ../MicroDrone-Firmware/src/Control/PID/PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/443942460" 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o.d 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/443942460/PID.o.d" -o ${OBJECTDIR}/_ext/443942460/PID.o ../MicroDrone-Firmware/src/Control/PID/PID.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/443942460/PID.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o: ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1035051164" 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" -o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o: ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" -o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/SensFusion.o: ../MicroDrone-Firmware/src/Control/SensFusion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" -o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ../MicroDrone-Firmware/src/Control/SensFusion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o: ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" -o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1204493032/AttitudeController.o: ../MicroDrone-Firmware/src/Control/AttitudeController.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" -o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ../MicroDrone-Firmware/src/Control/AttitudeController.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/149289479/MAVLinkSender.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o: ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1715884449" 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" -o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o: ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1313821444/num.o: ../MicroDrone-Firmware/src/Utils/num.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1313821444" 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o.d 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/framework/math/dsp" -I"../../../../../microchip/harmony/v2_06/framework/math/libq" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MMD -MF "${OBJECTDIR}/_ext/1313821444/num.o.d" -o ${OBJECTDIR}/_ext/1313821444/num.o ../MicroDrone-Firmware/src/Utils/num.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1313821444/num.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron-PIC32-HV2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../microchip/harmony/v2_06/bin/framework/math/dsp/dsp_pic32mz_nofpu.a ../../../../../microchip/harmony/v2_06/bin/framework/math/libq/libq_c_mips32_mz_l.a ../../../../../microchip/harmony/v2_06/bin/framework/peripheral/PIC32MZ2048ECM100_peripherals.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g   -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron-PIC32-HV2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../microchip/harmony/v2_06/bin/framework/math/dsp/dsp_pic32mz_nofpu.a ../../../../../microchip/harmony/v2_06/bin/framework/math/libq/libq_c_mips32_mz_l.a ../../../../../microchip/harmony/v2_06/bin/framework/peripheral/PIC32MZ2048ECM100_peripherals.a      -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x27F   -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=_min_heap_size=0,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron-PIC32-HV2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../microchip/harmony/v2_06/bin/framework/math/dsp/dsp_pic32mz_nofpu.a ../../../../../microchip/harmony/v2_06/bin/framework/math/libq/libq_c_mips32_mz_l.a ../../../../../microchip/harmony/v2_06/bin/framework/peripheral/PIC32MZ2048ECM100_peripherals.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron-PIC32-HV2.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../microchip/harmony/v2_06/bin/framework/math/dsp/dsp_pic32mz_nofpu.a ../../../../../microchip/harmony/v2_06/bin/framework/math/libq/libq_c_mips32_mz_l.a ../../../../../microchip/harmony/v2_06/bin/framework/peripheral/PIC32MZ2048ECM100_peripherals.a      -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=0,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	${MP_CC_DIR}/xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron-PIC32-HV2.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
