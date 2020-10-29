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
SOURCEFILES_QUOTED_IF_SPACED=../MicroDrone-Firmware/HAL/PIC_HV2/imu.c ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c ../MicroDrone-Firmware/libs/Math/Quaternion.c ../MicroDrone-Firmware/libs/Math/Vector3D.c ../src/main.c ../src/app.c ../src/mavlink_recv_task.c ../src/mavlink_send_task.c ../src/mavlink_status_task.c ../src/att_controller_task.c ../src/serialhandler.c ../src/imu_update_task.c ../MicroDrone-Firmware/src/Control/PID/PID.c ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c ../MicroDrone-Firmware/src/Control/SensFusion.c ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c ../MicroDrone-Firmware/src/Control/AttitudeController.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c ../MicroDrone-Firmware/src/Utils/num.c ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/general_exception_handler.c ../src/system_config/default/general-exception-context.S ../src/system_config/default/fassert.c ../src/system_config/default/system_tasks.c ../src/system_config/default/system_interrupt_a.S ../src/system_config/default/rtos_hooks.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1650642999/imu.o ${OBJECTDIR}/_ext/1650642999/comms.o ${OBJECTDIR}/_ext/1650642999/tof.o ${OBJECTDIR}/_ext/1650642999/motors.o ${OBJECTDIR}/_ext/971049567/Quaternion.o ${OBJECTDIR}/_ext/971049567/Vector3D.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ${OBJECTDIR}/_ext/1360937237/serialhandler.o ${OBJECTDIR}/_ext/1360937237/imu_update_task.o ${OBJECTDIR}/_ext/443942460/PID.o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ${OBJECTDIR}/_ext/1313821444/num.o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ${OBJECTDIR}/_ext/1276567923/croutine.o ${OBJECTDIR}/_ext/1276567923/list.o ${OBJECTDIR}/_ext/1276567923/queue.o ${OBJECTDIR}/_ext/1276567923/tasks.o ${OBJECTDIR}/_ext/1276567923/timers.o ${OBJECTDIR}/_ext/1276567923/event_groups.o ${OBJECTDIR}/_ext/1276567923/stream_buffer.o ${OBJECTDIR}/_ext/457403440/heap_1.o ${OBJECTDIR}/_ext/1571139743/port.o ${OBJECTDIR}/_ext/1571139743/port_asm.o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o ${OBJECTDIR}/_ext/1688732426/general-exception-context.o ${OBJECTDIR}/_ext/1688732426/fassert.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1650642999/imu.o.d ${OBJECTDIR}/_ext/1650642999/comms.o.d ${OBJECTDIR}/_ext/1650642999/tof.o.d ${OBJECTDIR}/_ext/1650642999/motors.o.d ${OBJECTDIR}/_ext/971049567/Quaternion.o.d ${OBJECTDIR}/_ext/971049567/Vector3D.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1360937237/app.o.d ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d ${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d ${OBJECTDIR}/_ext/1360937237/serialhandler.o.d ${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d ${OBJECTDIR}/_ext/443942460/PID.o.d ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d ${OBJECTDIR}/_ext/1313821444/num.o.d ${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d ${OBJECTDIR}/_ext/121284916/sys_devcon.o.d ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d ${OBJECTDIR}/_ext/1276567923/croutine.o.d ${OBJECTDIR}/_ext/1276567923/list.o.d ${OBJECTDIR}/_ext/1276567923/queue.o.d ${OBJECTDIR}/_ext/1276567923/tasks.o.d ${OBJECTDIR}/_ext/1276567923/timers.o.d ${OBJECTDIR}/_ext/1276567923/event_groups.o.d ${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d ${OBJECTDIR}/_ext/457403440/heap_1.o.d ${OBJECTDIR}/_ext/1571139743/port.o.d ${OBJECTDIR}/_ext/1571139743/port_asm.o.d ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d ${OBJECTDIR}/_ext/1688732426/system_init.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o.d ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d ${OBJECTDIR}/_ext/1688732426/fassert.o.d ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1650642999/imu.o ${OBJECTDIR}/_ext/1650642999/comms.o ${OBJECTDIR}/_ext/1650642999/tof.o ${OBJECTDIR}/_ext/1650642999/motors.o ${OBJECTDIR}/_ext/971049567/Quaternion.o ${OBJECTDIR}/_ext/971049567/Vector3D.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ${OBJECTDIR}/_ext/1360937237/serialhandler.o ${OBJECTDIR}/_ext/1360937237/imu_update_task.o ${OBJECTDIR}/_ext/443942460/PID.o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ${OBJECTDIR}/_ext/1313821444/num.o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ${OBJECTDIR}/_ext/1276567923/croutine.o ${OBJECTDIR}/_ext/1276567923/list.o ${OBJECTDIR}/_ext/1276567923/queue.o ${OBJECTDIR}/_ext/1276567923/tasks.o ${OBJECTDIR}/_ext/1276567923/timers.o ${OBJECTDIR}/_ext/1276567923/event_groups.o ${OBJECTDIR}/_ext/1276567923/stream_buffer.o ${OBJECTDIR}/_ext/457403440/heap_1.o ${OBJECTDIR}/_ext/1571139743/port.o ${OBJECTDIR}/_ext/1571139743/port_asm.o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o ${OBJECTDIR}/_ext/1688732426/general-exception-context.o ${OBJECTDIR}/_ext/1688732426/fassert.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o

# Source Files
SOURCEFILES=../MicroDrone-Firmware/HAL/PIC_HV2/imu.c ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c ../MicroDrone-Firmware/libs/Math/Quaternion.c ../MicroDrone-Firmware/libs/Math/Vector3D.c ../src/main.c ../src/app.c ../src/mavlink_recv_task.c ../src/mavlink_send_task.c ../src/mavlink_status_task.c ../src/att_controller_task.c ../src/serialhandler.c ../src/imu_update_task.c ../MicroDrone-Firmware/src/Control/PID/PID.c ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c ../MicroDrone-Firmware/src/Control/SensFusion.c ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c ../MicroDrone-Firmware/src/Control/AttitudeController.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c ../MicroDrone-Firmware/src/Utils/num.c ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/general_exception_handler.c ../src/system_config/default/general-exception-context.S ../src/system_config/default/fassert.c ../src/system_config/default/system_tasks.c ../src/system_config/default/system_interrupt_a.S ../src/system_config/default/rtos_hooks.c



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
${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  .generated_files/b28fb44a47c9d8bcc5042441a24f010a87e9ab2d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1571139743/port_asm.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  .generated_files/8bf20cd8675109a1e5a6169865f0ab512d7e4fd9.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1571139743" 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o.ok ${OBJECTDIR}/_ext/1571139743/port_asm.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1571139743/port_asm.o.d"  -o ${OBJECTDIR}/_ext/1571139743/port_asm.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1571139743/port_asm.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1571139743/port_asm.o.d" "${OBJECTDIR}/_ext/1571139743/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/general-exception-context.o: ../src/system_config/default/general-exception-context.S  .generated_files/7d33fa1495aac749a9fc6c431507e4b2f05d237.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general-exception-context.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.ok ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d"  -o ${OBJECTDIR}/_ext/1688732426/general-exception-context.o ../src/system_config/default/general-exception-context.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/general-exception-context.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d" "${OBJECTDIR}/_ext/1688732426/general-exception-context.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o: ../src/system_config/default/system_interrupt_a.S  .generated_files/f682c6f8be67462b25929733e7f0735b686ff86c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ../src/system_config/default/system_interrupt_a.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  .generated_files/d38c80e0220a2e421c89b80fbdbb4637a956a822.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.asm.d",--gdwarf-2,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1571139743/port_asm.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  .generated_files/1ac6eb41e2b15b568bc8d8aecf671df94ea26ff8.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1571139743" 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o.ok ${OBJECTDIR}/_ext/1571139743/port_asm.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1571139743/port_asm.o.d"  -o ${OBJECTDIR}/_ext/1571139743/port_asm.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1571139743/port_asm.o.asm.d",--gdwarf-2,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1571139743/port_asm.o.d" "${OBJECTDIR}/_ext/1571139743/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/general-exception-context.o: ../src/system_config/default/general-exception-context.S  .generated_files/8f02ee7059ee9d6bca7c3169bae918c7f1409aca.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general-exception-context.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.ok ${OBJECTDIR}/_ext/1688732426/general-exception-context.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d"  -o ${OBJECTDIR}/_ext/1688732426/general-exception-context.o ../src/system_config/default/general-exception-context.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/general-exception-context.o.asm.d",--gdwarf-2,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/general-exception-context.o.d" "${OBJECTDIR}/_ext/1688732426/general-exception-context.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o: ../src/system_config/default/system_interrupt_a.S  .generated_files/7ff5a2dfb95d4f39263a4aface00ab56262177e1.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ../src/system_config/default/system_interrupt_a.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d",--gdwarf-2,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1650642999/imu.o: ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c  .generated_files/fb436552d42099cb68e00e7b8d031b0e0d085417.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/imu.o.d" -o ${OBJECTDIR}/_ext/1650642999/imu.o ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/comms.o: ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c  .generated_files/141acc26460504389c814876be5af92b57e41b33.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/comms.o.d" -o ${OBJECTDIR}/_ext/1650642999/comms.o ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/tof.o: ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c  .generated_files/bd0f2d8bb86f362853111d0d9b0c3e94afd89cae.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/tof.o.d" -o ${OBJECTDIR}/_ext/1650642999/tof.o ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/motors.o: ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c  .generated_files/eb22c927b5b39d24a24085c25b28dfd4df6a7d60.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/motors.o.d" -o ${OBJECTDIR}/_ext/1650642999/motors.o ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/971049567/Quaternion.o: ../MicroDrone-Firmware/libs/Math/Quaternion.c  .generated_files/6dde63f1e7083459e177989b3bb57dea7fa12e4d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" -o ${OBJECTDIR}/_ext/971049567/Quaternion.o ../MicroDrone-Firmware/libs/Math/Quaternion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/971049567/Vector3D.o: ../MicroDrone-Firmware/libs/Math/Vector3D.c  .generated_files/59af795704738cf599e0f7693aad7ed4b8e732.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" -o ${OBJECTDIR}/_ext/971049567/Vector3D.o ../MicroDrone-Firmware/libs/Math/Vector3D.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  .generated_files/21307b08d3b821f60ee3ab943c8ad66d6f3d6897.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  .generated_files/884555ed619b6567f6e25bf8de1fec48fd2c2b7a.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o: ../src/mavlink_recv_task.c  .generated_files/e87d09c0ee6d5242cc9fd75e4fa7dabbdeb920dd.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ../src/mavlink_recv_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o: ../src/mavlink_send_task.c  .generated_files/f51ffa26c8bd31a03b42661ac659a5e3d7bacd79.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ../src/mavlink_send_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o: ../src/mavlink_status_task.c  .generated_files/66aa8b422a88d8b84103c90ef92c4202f9579fce.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ../src/mavlink_status_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/att_controller_task.o: ../src/att_controller_task.c  .generated_files/7a1e41c59c106cad4111a8e8d23eb1fc5299f0e1.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ../src/att_controller_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/serialhandler.o: ../src/serialhandler.c  .generated_files/a650df66af5cae990b8bf24f8732da3c533f4c9b.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/serialhandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/serialhandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/serialhandler.o.d" -o ${OBJECTDIR}/_ext/1360937237/serialhandler.o ../src/serialhandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/imu_update_task.o: ../src/imu_update_task.c  .generated_files/407e4dd89fb260bf242cafec3dd498f1d35fc537.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/imu_update_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/imu_update_task.o ../src/imu_update_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/443942460/PID.o: ../MicroDrone-Firmware/src/Control/PID/PID.c  .generated_files/28d39709a6aed6513e280dc1d309753fa1ff4509.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/443942460" 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o.d 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/443942460/PID.o.d" -o ${OBJECTDIR}/_ext/443942460/PID.o ../MicroDrone-Firmware/src/Control/PID/PID.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o: ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c  .generated_files/39e90df5c6f25b238559b52c7fa5488522a960d5.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1035051164" 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" -o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o: ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c  .generated_files/a63b20e4043021272f63904161b9c750708a21d0.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" -o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/SensFusion.o: ../MicroDrone-Firmware/src/Control/SensFusion.c  .generated_files/320b28fa4d7577e05265dbda68973f577aeb1389.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" -o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ../MicroDrone-Firmware/src/Control/SensFusion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o: ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c  .generated_files/4e00e208ee0292bee3cf82298a7ebf41f1df575c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" -o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/AttitudeController.o: ../MicroDrone-Firmware/src/Control/AttitudeController.c  .generated_files/42c831dfb500bd5d1665bafe7dd4e0395bd8c284.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" -o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ../MicroDrone-Firmware/src/Control/AttitudeController.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/149289479/MAVLinkSender.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c  .generated_files/d1b7ea0d502cdeb10c3f35f81f2d5192287b537b.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c  .generated_files/73fdcfdd9fd635734f1bfdb16a9ea9c8ae3fb724.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o: ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c  .generated_files/a8c1b158cd076db9ce8073a855bb5782a72f5d89.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1715884449" 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" -o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c  .generated_files/c4c954da8f7522b36c31f0757daa4f4ab6d4a5c3.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c  .generated_files/15a207cebdabcca9bf70b6c97038b70bd711d303.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c  .generated_files/95a080c0bbc8693cccd07dbb8edeaba53c90d466.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o: ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c  .generated_files/2ff6b8840b641867df2158cba3f78f87f069bd6e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c  .generated_files/11b17cef4ee02183aaf93caf1e26b8f77535ba47.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1313821444/num.o: ../MicroDrone-Firmware/src/Utils/num.c  .generated_files/57d0e23c806668a629f4a5d3cc80e1d14c8247a7.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1313821444" 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o.d 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1313821444/num.o.d" -o ${OBJECTDIR}/_ext/1313821444/num.o ../MicroDrone-Firmware/src/Utils/num.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1353086322/osal_freertos.o: ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c  .generated_files/8b4d671b8b27ade68bd3c849e8c44c8f1ff71ec0.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1353086322" 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/121284916/sys_devcon.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c  .generated_files/62cd8a4b53a87c04ca746bcf557436969c189090.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c  .generated_files/fba11859ccb2115139c14bb7732654bbfb9ae72.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o: ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c  .generated_files/6a030f7d33cd1bd8c739509cb3ae97d00de646d1.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1317643790" 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/croutine.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c  .generated_files/dcf9b1af4f40ff83b2cdbf868d0f2fce19a18e8a.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/croutine.o.d" -o ${OBJECTDIR}/_ext/1276567923/croutine.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/list.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c  .generated_files/7be62dd80886d5e339de703af7df8b1af93ffcf8.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/list.o.d" -o ${OBJECTDIR}/_ext/1276567923/list.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/queue.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c  .generated_files/de8e250b7010e3662a18151deb2e9353568fc4a2.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/queue.o.d" -o ${OBJECTDIR}/_ext/1276567923/queue.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/tasks.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c  .generated_files/a74ec84b8211dff13fe13beb55369be6212bb6fb.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/tasks.o.d" -o ${OBJECTDIR}/_ext/1276567923/tasks.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/timers.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c  .generated_files/f51b73237f9239cc8ce04f216223c901681649c0.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/timers.o.d" -o ${OBJECTDIR}/_ext/1276567923/timers.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/event_groups.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c  .generated_files/409106552b641fac34a8db70b0f6213d281c9cc0.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/event_groups.o.d" -o ${OBJECTDIR}/_ext/1276567923/event_groups.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/stream_buffer.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c  .generated_files/4522d2704502cc7aeeeb1ff020c044c2e9553f94.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/stream_buffer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d" -o ${OBJECTDIR}/_ext/1276567923/stream_buffer.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/457403440/heap_1.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c  .generated_files/acb8ac5faeda6f72a1bd75a3d59fb855b610be9e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/457403440" 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o.d 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/457403440/heap_1.o.d" -o ${OBJECTDIR}/_ext/457403440/heap_1.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1571139743/port.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c  .generated_files/e92cc6a9ce1e176391474e2d2835963c3c6f33bc.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1571139743" 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1571139743/port.o.d" -o ${OBJECTDIR}/_ext/1571139743/port.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c  .generated_files/3c421871575e8fd84577fa8dcfd9714cbda6122a.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1047219354/drv_oc_static.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c  .generated_files/547c17d78f162d114c966250e1451af5808a38f6.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c  .generated_files/79393f38dc6124be0200e7f374bef4d7b928c60a.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c  .generated_files/77dc5ea7461c6d4bf967c394dc7e15737d644086.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c  .generated_files/56c61446a24acffecb65e00143c3620a85cbe2c0.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_static.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c  .generated_files/28f02d9ce963bfa0e53c5d8e3b3805315d2ad245.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c  .generated_files/e37f865b96df4a6ea623b48671f73e4b7ef24aa5.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o: ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c  .generated_files/3d9e3bfe164ba91bc6729e6331c5d7c3e78ef69e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  .generated_files/68f67208474b5d410855480a31e3074ddf66a471.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  .generated_files/5706216daa9081694c9adc5f4ac1f2ab1dba445.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  .generated_files/f8ee8b7d7b8db8368210ef527dff23473a0ef709.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/general_exception_handler.o: ../src/system_config/default/general_exception_handler.c  .generated_files/62aac3cf04272e1ec944b5ff530048812f13b510.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/general_exception_handler.o.d" -o ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o ../src/system_config/default/general_exception_handler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/fassert.o: ../src/system_config/default/fassert.c  .generated_files/b24002c0b925bc0e9fd301ace87bd6076e900d32.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/fassert.o.d" -o ${OBJECTDIR}/_ext/1688732426/fassert.o ../src/system_config/default/fassert.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  .generated_files/ad177087b3a626ff943fffb1948fc162f0abc37.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/rtos_hooks.o: ../src/system_config/default/rtos_hooks.c  .generated_files/1b691e9d45ac8405f5674a8d5845e722866104e2.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ../src/system_config/default/rtos_hooks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/_ext/1650642999/imu.o: ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c  .generated_files/ae411147347d1f1f1f257a8eb7acd889200b2228.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/imu.o.d" -o ${OBJECTDIR}/_ext/1650642999/imu.o ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/comms.o: ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c  .generated_files/2b16e0bbca50e6ffcfcd06f09d62ac06d676f9ba.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/comms.o.d" -o ${OBJECTDIR}/_ext/1650642999/comms.o ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/tof.o: ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c  .generated_files/15e2d7acd2d38b2fc2835353dc584cbf87c67dc9.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/tof.o.d" -o ${OBJECTDIR}/_ext/1650642999/tof.o ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/motors.o: ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c  .generated_files/8350273e7cb56fda38fe1b6c39b21c5cde3a210.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/motors.o.d" -o ${OBJECTDIR}/_ext/1650642999/motors.o ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/971049567/Quaternion.o: ../MicroDrone-Firmware/libs/Math/Quaternion.c  .generated_files/470fdb8fc836f2ac2a1bd39820744fa0e019d651.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" -o ${OBJECTDIR}/_ext/971049567/Quaternion.o ../MicroDrone-Firmware/libs/Math/Quaternion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/971049567/Vector3D.o: ../MicroDrone-Firmware/libs/Math/Vector3D.c  .generated_files/7bfa4df9f151b872aebea237acc58d8293e43cee.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" -o ${OBJECTDIR}/_ext/971049567/Vector3D.o ../MicroDrone-Firmware/libs/Math/Vector3D.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  .generated_files/a0907eeb6feea015dfbf89914d62e4759aafb177.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  .generated_files/31b7a263c9423e30f762b968328900f35b53c045.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o: ../src/mavlink_recv_task.c  .generated_files/6e427bedba1fb73d1c502f64146f949efcd9489.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ../src/mavlink_recv_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o: ../src/mavlink_send_task.c  .generated_files/ad858c49c638faac7fc5bd77a9b0c47351ccc5fb.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ../src/mavlink_send_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o: ../src/mavlink_status_task.c  .generated_files/f5622dec0fc3515dc0f5b55deb6de4785c3ad59f.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ../src/mavlink_status_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/att_controller_task.o: ../src/att_controller_task.c  .generated_files/cb9d88b9d88af35461471534edfff6d5478b11be.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ../src/att_controller_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/serialhandler.o: ../src/serialhandler.c  .generated_files/3192cff6d26054c35e278e55ceb875789f12c5ef.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/serialhandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/serialhandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/serialhandler.o.d" -o ${OBJECTDIR}/_ext/1360937237/serialhandler.o ../src/serialhandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/imu_update_task.o: ../src/imu_update_task.c  .generated_files/52ada6edec26cfcf5efe0aa470a5ad58dbba4b6.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/imu_update_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/imu_update_task.o ../src/imu_update_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/443942460/PID.o: ../MicroDrone-Firmware/src/Control/PID/PID.c  .generated_files/5def750932923cf1fd7b370f1a7f870bf9921a16.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/443942460" 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o.d 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/443942460/PID.o.d" -o ${OBJECTDIR}/_ext/443942460/PID.o ../MicroDrone-Firmware/src/Control/PID/PID.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o: ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c  .generated_files/842bfec376045f056238401501bc45d55926df91.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1035051164" 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" -o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o: ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c  .generated_files/592f48f49719a2ce07a3ef8f2cce240d73092dd8.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" -o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/SensFusion.o: ../MicroDrone-Firmware/src/Control/SensFusion.c  .generated_files/30a7011eb59ad978c87214434e02e3fb6949063d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" -o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ../MicroDrone-Firmware/src/Control/SensFusion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o: ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c  .generated_files/635948106c4cfc2b54de0cd53a6b35e0efb54030.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" -o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/AttitudeController.o: ../MicroDrone-Firmware/src/Control/AttitudeController.c  .generated_files/d421de0029957e1d0e0dcf1350de0259933e10b6.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" -o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ../MicroDrone-Firmware/src/Control/AttitudeController.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/149289479/MAVLinkSender.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c  .generated_files/3b3002eebc70b126f4c97fda059640ba0589b321.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c  .generated_files/f050edea58fcd2f2b4a6a3f6ce78e2a5648c85a.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o: ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c  .generated_files/11af13239836fee04a748293447e199bf5f7867d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1715884449" 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" -o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c  .generated_files/edec6541c1a3bfcae60eeffdc90adec1bc05df34.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c  .generated_files/2a2adf879e85b41d13d1bfa1acdb70851008bfb4.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c  .generated_files/c12befef6877cf0c7a807a9a5938083065719cce.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o: ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c  .generated_files/99c89b56b91cfce05867fee338ea653d8f4279f3.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c  .generated_files/5217ce81ad88db388ecf0ce072b84efc5036b2c1.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1313821444/num.o: ../MicroDrone-Firmware/src/Utils/num.c  .generated_files/55ce496673258ed7a91bf5969a0079fef97acbfd.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1313821444" 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o.d 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1313821444/num.o.d" -o ${OBJECTDIR}/_ext/1313821444/num.o ../MicroDrone-Firmware/src/Utils/num.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1353086322/osal_freertos.o: ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c  .generated_files/d1cd9fe646c0f93ebccde726ac053bd67c388ee0.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1353086322" 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/121284916/sys_devcon.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c  .generated_files/27f83ec91da1d21b3b77859d76acddb24ef6f18d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c  .generated_files/80549679b91e573559c747d3f3f82f3fe81f8ef5.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o: ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c  .generated_files/cb6d795b4a4e6ff6a911112f37c6e884ee841f76.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1317643790" 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/croutine.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c  .generated_files/674ff3e3c7a84248bb842d7c27cac83f394692e5.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/croutine.o.d" -o ${OBJECTDIR}/_ext/1276567923/croutine.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/list.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c  .generated_files/13f1cac5d7a5593558dc876fb36c518a2f1ce989.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/list.o.d" -o ${OBJECTDIR}/_ext/1276567923/list.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/queue.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c  .generated_files/ada78076b37c7dcacaf4601acf3316a0574e5196.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/queue.o.d" -o ${OBJECTDIR}/_ext/1276567923/queue.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/tasks.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c  .generated_files/65167980113e57d024104996bfe20bd2e66ace5f.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/tasks.o.d" -o ${OBJECTDIR}/_ext/1276567923/tasks.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/timers.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c  .generated_files/f5756b52f947f9d67654a0091097b05af8b3d980.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/timers.o.d" -o ${OBJECTDIR}/_ext/1276567923/timers.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/event_groups.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c  .generated_files/8257afc3e9bab11cc9393a245c8e2d85b840a046.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/event_groups.o.d" -o ${OBJECTDIR}/_ext/1276567923/event_groups.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/stream_buffer.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c  .generated_files/cc6d0303c91336454205672cf6ddd9415790e33f.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/stream_buffer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d" -o ${OBJECTDIR}/_ext/1276567923/stream_buffer.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/457403440/heap_1.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c  .generated_files/af154f71528a1fb1e8351aaad9e6474666634ad4.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/457403440" 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o.d 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/457403440/heap_1.o.d" -o ${OBJECTDIR}/_ext/457403440/heap_1.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1571139743/port.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c  .generated_files/e92e8dea0ea6f4d523dbcd4c1d0942e2b1b51024.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1571139743" 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1571139743/port.o.d" -o ${OBJECTDIR}/_ext/1571139743/port.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c  .generated_files/81bb821e9c1d68438dad38193f7dda60d9cde260.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1047219354/drv_oc_static.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c  .generated_files/32ed54729869c135a2f48b0f554deaf52e8b6baf.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c  .generated_files/bc9d4b290796abd78da373485f6327618d8ddbd7.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c  .generated_files/71cc94e455cdbe6105a7399d8c9da2e33a9f4d9d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c  .generated_files/ddd9f9a083226662b6526466585f1fb4ef21e4.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_static.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c  .generated_files/d2f0391d976477d534121b9a321ffc1ee62d43d3.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c  .generated_files/89c908aef9975a1201afff19993a58685564fbc4.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o: ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c  .generated_files/3b99333ce59adca1ee1657db3d97743e9e45192d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  .generated_files/1cd39159f08e8f9407f0c866e4e943736fe68d2a.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  .generated_files/2f4d5de45fd7867682109a8da55b1f1c351f39f1.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  .generated_files/fb9133f6b90fabb01f4ed42e3226e6a2c807e176.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/general_exception_handler.o: ../src/system_config/default/general_exception_handler.c  .generated_files/2199d3cc36471fb08f066f65d61696f9c359a105.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/general_exception_handler.o.d" -o ${OBJECTDIR}/_ext/1688732426/general_exception_handler.o ../src/system_config/default/general_exception_handler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/fassert.o: ../src/system_config/default/fassert.c  .generated_files/3c717da88cb8cebad35d85390900d409b3d49a2e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/fassert.o.d" -o ${OBJECTDIR}/_ext/1688732426/fassert.o ../src/system_config/default/fassert.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  .generated_files/8b3818935d10fa1f77b0ef578141d77e9e6e0bb0.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/rtos_hooks.o: ../src/system_config/default/rtos_hooks.c  .generated_files/2d02ca54744ad7811b962602840bfd0160179ceb.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ../src/system_config/default/rtos_hooks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron-PIC32-HV2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../microchip/harmony/v2_06/bin/framework/peripheral/PIC32MZ2048ECM100_peripherals.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g   -mprocessor=$(MP_PROCESSOR_OPTION) -O1 -o dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron-PIC32-HV2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../microchip/harmony/v2_06/bin/framework/peripheral/PIC32MZ2048ECM100_peripherals.a      -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x27F   -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=_min_heap_size=0,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron-PIC32-HV2.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../microchip/harmony/v2_06/bin/framework/peripheral/PIC32MZ2048ECM100_peripherals.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION) -O1 -o dist/${CND_CONF}/${IMAGE_TYPE}/MicroDron-PIC32-HV2.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../microchip/harmony/v2_06/bin/framework/peripheral/PIC32MZ2048ECM100_peripherals.a      -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=0,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
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
