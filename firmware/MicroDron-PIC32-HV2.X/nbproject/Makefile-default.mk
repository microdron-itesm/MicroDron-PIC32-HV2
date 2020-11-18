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
SOURCEFILES_QUOTED_IF_SPACED=../MicroDrone-Firmware/HAL/PIC_HV2/imu.c ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c ../MicroDrone-Firmware/libs/Math/Quaternion.c ../MicroDrone-Firmware/libs/Math/Vector3D.c ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/fassert.c ../src/system_config/default/system_tasks.c ../src/system_config/default/system_interrupt_a.S ../src/system_config/default/rtos_hooks.c ../src/main.c ../src/app.c ../src/mavlink_recv_task.c ../src/mavlink_send_task.c ../src/mavlink_status_task.c ../src/att_controller_task.c ../src/serialhandler.c ../src/imu_update_task.c ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c ../MicroDrone-Firmware/src/Control/PID/PID.c ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c ../MicroDrone-Firmware/src/Control/SensFusion.c ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c ../MicroDrone-Firmware/src/Control/AttitudeController.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c ../MicroDrone-Firmware/src/Utils/num.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1650642999/imu.o ${OBJECTDIR}/_ext/1650642999/comms.o ${OBJECTDIR}/_ext/1650642999/tof.o ${OBJECTDIR}/_ext/1650642999/motors.o ${OBJECTDIR}/_ext/971049567/Quaternion.o ${OBJECTDIR}/_ext/971049567/Vector3D.o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/fassert.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ${OBJECTDIR}/_ext/1360937237/serialhandler.o ${OBJECTDIR}/_ext/1360937237/imu_update_task.o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ${OBJECTDIR}/_ext/457403440/heap_1.o ${OBJECTDIR}/_ext/1571139743/port.o ${OBJECTDIR}/_ext/1571139743/port_asm.o ${OBJECTDIR}/_ext/1276567923/croutine.o ${OBJECTDIR}/_ext/1276567923/list.o ${OBJECTDIR}/_ext/1276567923/queue.o ${OBJECTDIR}/_ext/1276567923/tasks.o ${OBJECTDIR}/_ext/1276567923/timers.o ${OBJECTDIR}/_ext/1276567923/event_groups.o ${OBJECTDIR}/_ext/1276567923/stream_buffer.o ${OBJECTDIR}/_ext/443942460/PID.o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ${OBJECTDIR}/_ext/1313821444/num.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1650642999/imu.o.d ${OBJECTDIR}/_ext/1650642999/comms.o.d ${OBJECTDIR}/_ext/1650642999/tof.o.d ${OBJECTDIR}/_ext/1650642999/motors.o.d ${OBJECTDIR}/_ext/971049567/Quaternion.o.d ${OBJECTDIR}/_ext/971049567/Vector3D.o.d ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d ${OBJECTDIR}/_ext/1688732426/system_init.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d ${OBJECTDIR}/_ext/1688732426/fassert.o.d ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1360937237/app.o.d ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d ${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d ${OBJECTDIR}/_ext/1360937237/serialhandler.o.d ${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d ${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d ${OBJECTDIR}/_ext/121284916/sys_devcon.o.d ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d ${OBJECTDIR}/_ext/457403440/heap_1.o.d ${OBJECTDIR}/_ext/1571139743/port.o.d ${OBJECTDIR}/_ext/1571139743/port_asm.o.d ${OBJECTDIR}/_ext/1276567923/croutine.o.d ${OBJECTDIR}/_ext/1276567923/list.o.d ${OBJECTDIR}/_ext/1276567923/queue.o.d ${OBJECTDIR}/_ext/1276567923/tasks.o.d ${OBJECTDIR}/_ext/1276567923/timers.o.d ${OBJECTDIR}/_ext/1276567923/event_groups.o.d ${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d ${OBJECTDIR}/_ext/443942460/PID.o.d ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d ${OBJECTDIR}/_ext/1313821444/num.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1650642999/imu.o ${OBJECTDIR}/_ext/1650642999/comms.o ${OBJECTDIR}/_ext/1650642999/tof.o ${OBJECTDIR}/_ext/1650642999/motors.o ${OBJECTDIR}/_ext/971049567/Quaternion.o ${OBJECTDIR}/_ext/971049567/Vector3D.o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/fassert.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ${OBJECTDIR}/_ext/1360937237/serialhandler.o ${OBJECTDIR}/_ext/1360937237/imu_update_task.o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ${OBJECTDIR}/_ext/457403440/heap_1.o ${OBJECTDIR}/_ext/1571139743/port.o ${OBJECTDIR}/_ext/1571139743/port_asm.o ${OBJECTDIR}/_ext/1276567923/croutine.o ${OBJECTDIR}/_ext/1276567923/list.o ${OBJECTDIR}/_ext/1276567923/queue.o ${OBJECTDIR}/_ext/1276567923/tasks.o ${OBJECTDIR}/_ext/1276567923/timers.o ${OBJECTDIR}/_ext/1276567923/event_groups.o ${OBJECTDIR}/_ext/1276567923/stream_buffer.o ${OBJECTDIR}/_ext/443942460/PID.o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ${OBJECTDIR}/_ext/1313821444/num.o

# Source Files
SOURCEFILES=../MicroDrone-Firmware/HAL/PIC_HV2/imu.c ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c ../MicroDrone-Firmware/libs/Math/Quaternion.c ../MicroDrone-Firmware/libs/Math/Vector3D.c ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/fassert.c ../src/system_config/default/system_tasks.c ../src/system_config/default/system_interrupt_a.S ../src/system_config/default/rtos_hooks.c ../src/main.c ../src/app.c ../src/mavlink_recv_task.c ../src/mavlink_send_task.c ../src/mavlink_status_task.c ../src/att_controller_task.c ../src/serialhandler.c ../src/imu_update_task.c ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c ../MicroDrone-Firmware/src/Control/PID/PID.c ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c ../MicroDrone-Firmware/src/Control/SensFusion.c ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c ../MicroDrone-Firmware/src/Control/AttitudeController.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c ../MicroDrone-Firmware/src/Utils/num.c



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
${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o: ../src/system_config/default/system_interrupt_a.S  .generated_files/f682c6f8be67462b25929733e7f0735b686ff86c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ../src/system_config/default/system_interrupt_a.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
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
	
else
${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o: ../src/system_config/default/system_interrupt_a.S  .generated_files/b4dd3fd629651a55829c095ef547067509a7db83.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ../src/system_config/default/system_interrupt_a.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d",-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  .generated_files/9bd2c8867b3e6978762972355b8514bbd24112e7.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.asm.d",-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1571139743/port_asm.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  .generated_files/b286baaff86ad6015c9d4fb5c85a37c3e7b69df5.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1571139743" 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port_asm.o.ok ${OBJECTDIR}/_ext/1571139743/port_asm.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1571139743/port_asm.o.d"  -o ${OBJECTDIR}/_ext/1571139743/port_asm.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1571139743/port_asm.o.asm.d",-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1571139743/port_asm.o.d" "${OBJECTDIR}/_ext/1571139743/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1650642999/imu.o: ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c  .generated_files/93ac8f3a5a5953de95d7d8445ea190b979d46a6d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/imu.o.d" -o ${OBJECTDIR}/_ext/1650642999/imu.o ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/comms.o: ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c  .generated_files/c3df3397c0ff3820b0bd61d33ba7f289e1d86bfa.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/comms.o.d" -o ${OBJECTDIR}/_ext/1650642999/comms.o ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/tof.o: ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c  .generated_files/93b3f00267ee425117a585cdf4801d54997c0e7f.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/tof.o.d" -o ${OBJECTDIR}/_ext/1650642999/tof.o ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/motors.o: ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c  .generated_files/35dc90ac502a7f57d50f43f6916f5b3e38b41095.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/motors.o.d" -o ${OBJECTDIR}/_ext/1650642999/motors.o ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/971049567/Quaternion.o: ../MicroDrone-Firmware/libs/Math/Quaternion.c  .generated_files/3311631e04e136d1e706e416e912482d1a1ae087.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" -o ${OBJECTDIR}/_ext/971049567/Quaternion.o ../MicroDrone-Firmware/libs/Math/Quaternion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/971049567/Vector3D.o: ../MicroDrone-Firmware/libs/Math/Vector3D.c  .generated_files/a15ef34de504de1a151982d90adaf22fb9c7524a.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" -o ${OBJECTDIR}/_ext/971049567/Vector3D.o ../MicroDrone-Firmware/libs/Math/Vector3D.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c  .generated_files/345021d80a5b26250d82bbc083b163e1ba6ad04f.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1047219354/drv_oc_static.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c  .generated_files/bf1c24b5a9b82f00c7eb5b2bac0a387d00ddb97a.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c  .generated_files/61b9d528177817905f28b47c87be40be3e76ffbd.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c  .generated_files/321dc2a7ea6c85dfdb00f69c44722a6a46b71422.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c  .generated_files/d4876ae4e527a2187db28685032377eff6baddbf.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_static.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c  .generated_files/930dbbad5cb0b5e6cea726997bd916d06a60a6e7.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c  .generated_files/5c91feabfd27031b1c5d4a268b7ea2bf12d786ff.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o: ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c  .generated_files/ec3913771e15cca36828fef1113becf69c78a4b2.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  .generated_files/5ca297145e19f64fc19b7fa1215f991ea7d1f0fd.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  .generated_files/659f66d76821ee6eaf67e1117ec419a2b8958ebd.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  .generated_files/a79cf0b4b87f8a629eab318c259312e75e20a0da.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/fassert.o: ../src/system_config/default/fassert.c  .generated_files/e4e14ace28a61036d89c74e35e7cfbd5f61589b4.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/fassert.o.d" -o ${OBJECTDIR}/_ext/1688732426/fassert.o ../src/system_config/default/fassert.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  .generated_files/5daa65cf409706aaf3a54cb85e6b7c000062a1a9.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/rtos_hooks.o: ../src/system_config/default/rtos_hooks.c  .generated_files/48b0b78589fffdab64fd3b71760d56be5b004012.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ../src/system_config/default/rtos_hooks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  .generated_files/856ee522213f94d1832e250992f8ecf3555fdee4.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  .generated_files/4dca1e6826de3675769d4151bde4b59c50a0caff.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o: ../src/mavlink_recv_task.c  .generated_files/c9e227849839e698c3a5b332182aec678dd7139a.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ../src/mavlink_recv_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o: ../src/mavlink_send_task.c  .generated_files/4247810ab4ebab16505204982f467678a8952e68.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ../src/mavlink_send_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o: ../src/mavlink_status_task.c  .generated_files/c8a123e8214829700f848918f9f1945d24c6d11e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ../src/mavlink_status_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/att_controller_task.o: ../src/att_controller_task.c  .generated_files/b899d4ca7baaa45574ee70bf51c02031dbfd5e31.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ../src/att_controller_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/serialhandler.o: ../src/serialhandler.c  .generated_files/f2fe14c85b17c63984717df1f0abb01506ab1bca.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/serialhandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/serialhandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/serialhandler.o.d" -o ${OBJECTDIR}/_ext/1360937237/serialhandler.o ../src/serialhandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/imu_update_task.o: ../src/imu_update_task.c  .generated_files/ff15f370c248469bb7365091398a06efa5d91523.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/imu_update_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/imu_update_task.o ../src/imu_update_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1353086322/osal_freertos.o: ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c  .generated_files/281930d545f3aa66419b521eeb4d4d2653cd5aa3.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1353086322" 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/121284916/sys_devcon.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c  .generated_files/68b2dd336d81ad9f178a1d46c40124b07b52f510.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c  .generated_files/b2224a6dc31143e9e8629af2616c457649c301d5.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o: ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c  .generated_files/3b894014b8c21c89f09bd06aad0b423ad211174b.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1317643790" 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/457403440/heap_1.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c  .generated_files/b441903b3e3d04c58db463737ebe1c8dedd63ad4.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/457403440" 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o.d 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/457403440/heap_1.o.d" -o ${OBJECTDIR}/_ext/457403440/heap_1.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1571139743/port.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c  .generated_files/1ccf651f62c416c272d5a7383ece6870606bc3fc.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1571139743" 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1571139743/port.o.d" -o ${OBJECTDIR}/_ext/1571139743/port.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/croutine.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c  .generated_files/de33ec2c0b04a430737898d9a0fe3f02f05d1d3e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/croutine.o.d" -o ${OBJECTDIR}/_ext/1276567923/croutine.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/list.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c  .generated_files/3b032a342ea0c5dc55352f1d9ce786071bb43c9.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/list.o.d" -o ${OBJECTDIR}/_ext/1276567923/list.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/queue.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c  .generated_files/cdc3c23cdf4a861b412ac545c015efe191c7bf01.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/queue.o.d" -o ${OBJECTDIR}/_ext/1276567923/queue.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/tasks.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c  .generated_files/1639197f9c2f3e65b7a12fd7bc7c6e268c1087f.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/tasks.o.d" -o ${OBJECTDIR}/_ext/1276567923/tasks.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/timers.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c  .generated_files/acaa6918a9994400c8b6747779226e27771d9daa.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/timers.o.d" -o ${OBJECTDIR}/_ext/1276567923/timers.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/event_groups.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c  .generated_files/b6acac10593ef3efea2e269484799be3487c9e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/event_groups.o.d" -o ${OBJECTDIR}/_ext/1276567923/event_groups.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/stream_buffer.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c  .generated_files/d9422912b2ffa2fb52a4ea4ca8ee26fd27d5d657.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/stream_buffer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d" -o ${OBJECTDIR}/_ext/1276567923/stream_buffer.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/443942460/PID.o: ../MicroDrone-Firmware/src/Control/PID/PID.c  .generated_files/e2d82d1d56426dd060f06c860747fd618a395899.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/443942460" 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o.d 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/443942460/PID.o.d" -o ${OBJECTDIR}/_ext/443942460/PID.o ../MicroDrone-Firmware/src/Control/PID/PID.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o: ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c  .generated_files/695fb7eee080d4b3f8f4db16a2c61613a0102323.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1035051164" 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" -o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o: ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c  .generated_files/d4a598047e70b054ea9d86cf1ea2c76fbcc36eb7.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" -o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/SensFusion.o: ../MicroDrone-Firmware/src/Control/SensFusion.c  .generated_files/dd32507ce2ee27943dbff724aa674b837eb7e0f0.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" -o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ../MicroDrone-Firmware/src/Control/SensFusion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o: ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c  .generated_files/99f46119027b983b4f5c48bfe67a711b8f95684f.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" -o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/AttitudeController.o: ../MicroDrone-Firmware/src/Control/AttitudeController.c  .generated_files/25ca247ba4f6f8d1682a60ea70bbc9c0d8f7999f.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" -o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ../MicroDrone-Firmware/src/Control/AttitudeController.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/149289479/MAVLinkSender.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c  .generated_files/b9c1d86f61aff5dddd250602064e64e11b49d31d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c  .generated_files/abe053fe46c9927b72965bac9b9bd082b63341c2.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o: ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c  .generated_files/f92a11dad914ef7b298934aaf0b037770ea3de62.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1715884449" 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" -o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c  .generated_files/9f978cca31a417283e5af9768c238042101b32de.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c  .generated_files/ba6c5b79868184e0cfc8b401e8b29142b6c134d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c  .generated_files/e5b995e3b8014d385956654eb512bc164d7fd30c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o: ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c  .generated_files/b0c0a8d72935a5bf87bc44a8d3460d771396c7cb.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c  .generated_files/bc09637a319c500dd35183f0ff01cb9658552f3c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1313821444/num.o: ../MicroDrone-Firmware/src/Utils/num.c  .generated_files/f5c11d810d1718fc6623ceb8d6ee83eea4c8a1da.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1313821444" 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o.d 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1313821444/num.o.d" -o ${OBJECTDIR}/_ext/1313821444/num.o ../MicroDrone-Firmware/src/Utils/num.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/_ext/1650642999/imu.o: ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c  .generated_files/861210180925b33319a6eca1dfe15f4282feeba6.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/imu.o.d" -o ${OBJECTDIR}/_ext/1650642999/imu.o ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/comms.o: ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c  .generated_files/786afd2c324cdcba414796584d5d9aab438359ee.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/comms.o.d" -o ${OBJECTDIR}/_ext/1650642999/comms.o ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/tof.o: ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c  .generated_files/ff539b84b5b9fdab995b1dedfb338cea0906760e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/tof.o.d" -o ${OBJECTDIR}/_ext/1650642999/tof.o ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/motors.o: ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c  .generated_files/1083342e2ea5cc943dd4d2c46613970ccd2b28fb.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/motors.o.d" -o ${OBJECTDIR}/_ext/1650642999/motors.o ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/971049567/Quaternion.o: ../MicroDrone-Firmware/libs/Math/Quaternion.c  .generated_files/44f81b4e2b4d9be1a00f7bf5314db00dffe77cc6.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" -o ${OBJECTDIR}/_ext/971049567/Quaternion.o ../MicroDrone-Firmware/libs/Math/Quaternion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/971049567/Vector3D.o: ../MicroDrone-Firmware/libs/Math/Vector3D.c  .generated_files/2e094c74fdc5e03b5778f274be2e542edf7fa426.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" -o ${OBJECTDIR}/_ext/971049567/Vector3D.o ../MicroDrone-Firmware/libs/Math/Vector3D.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c  .generated_files/dd633ca7b78440f7903dccd47d084cbb6212ee24.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1047219354/drv_oc_static.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c  .generated_files/9675518434aa01fbf0516ca110e896cdff120050.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c  .generated_files/7040fc16ff0261c81864bbdb4593023fefaed2a4.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c  .generated_files/d76c09c41013170164408387e866bf107dc7285b.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c  .generated_files/216ae2ff4c21526decd705ec3dfc9dbfef4d906f.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_static.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c  .generated_files/5d2a0807883c44080f27358496c8824264bce120.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c  .generated_files/260b9ca655409e11f1a51b350df46bf35d12ed.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o: ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c  .generated_files/d316b15b8bd80669c0242fd5b4b5a1135e3a9af5.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  .generated_files/4ddc93e7b905f94301f05a82cc4d487498f90561.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  .generated_files/c3e234764a69a5e3b1893075aaf713123cbc2951.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  .generated_files/7cc425bfe57639511d83ecc7c98862bec8562d12.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/fassert.o: ../src/system_config/default/fassert.c  .generated_files/53abec7abc0d2c7cf5207f79dcb3cad027699cfa.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/fassert.o.d" -o ${OBJECTDIR}/_ext/1688732426/fassert.o ../src/system_config/default/fassert.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  .generated_files/57026a7775456e92fb54ef259ed0348d28b178de.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/rtos_hooks.o: ../src/system_config/default/rtos_hooks.c  .generated_files/23e858c1ce562e7914f3fe06dbdd037d142a46e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ../src/system_config/default/rtos_hooks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  .generated_files/9c276a33ee1b8da5eff9b79e7fbf5212042fe02c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  .generated_files/4d23ade79924afce1ef26bf2b42a5e9914021122.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o: ../src/mavlink_recv_task.c  .generated_files/2e4ac3192c9a2c06d92f9bca1e51b0197b9ac450.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ../src/mavlink_recv_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o: ../src/mavlink_send_task.c  .generated_files/1b919206af174c602e27bd495196715937e12d18.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ../src/mavlink_send_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o: ../src/mavlink_status_task.c  .generated_files/11ee416b7352baeec61febee2e8462d520f57935.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ../src/mavlink_status_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/att_controller_task.o: ../src/att_controller_task.c  .generated_files/97b4a8aeb6545d3422e49452986552d9d58aa664.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ../src/att_controller_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/serialhandler.o: ../src/serialhandler.c  .generated_files/7aa3e553c0667a818d309c56ddebf174c1983eae.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/serialhandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/serialhandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/serialhandler.o.d" -o ${OBJECTDIR}/_ext/1360937237/serialhandler.o ../src/serialhandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/imu_update_task.o: ../src/imu_update_task.c  .generated_files/d66de84fd16af620c4169fc2e6020728447ff988.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/imu_update_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/imu_update_task.o ../src/imu_update_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1353086322/osal_freertos.o: ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c  .generated_files/ccaf78b836e7652e86097f05c23667a37f3cf6df.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1353086322" 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/121284916/sys_devcon.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c  .generated_files/d198e475fd4490143ba7d342f83235188a4c8918.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c  .generated_files/2044fb8bfd5d438ef51b51fb4a05fccaa4e21c92.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o: ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c  .generated_files/236ee2220f49fcda1c80501725a24f2af7233e8e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1317643790" 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/457403440/heap_1.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c  .generated_files/1745b846c332b2de3a3285986346812dad3c5a96.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/457403440" 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o.d 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/457403440/heap_1.o.d" -o ${OBJECTDIR}/_ext/457403440/heap_1.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1571139743/port.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c  .generated_files/19ff24d9c7d41d63e3aa77e7764e205a929b2fbd.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1571139743" 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1571139743/port.o.d" -o ${OBJECTDIR}/_ext/1571139743/port.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/croutine.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c  .generated_files/fe0b9f0c25c5bb99311b9382fea2889ec0ab7b71.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/croutine.o.d" -o ${OBJECTDIR}/_ext/1276567923/croutine.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/list.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c  .generated_files/74d9d7624412388a6e8f2f88f35510032dd6825e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/list.o.d" -o ${OBJECTDIR}/_ext/1276567923/list.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/queue.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c  .generated_files/e4dc27141264cda9575d6f6f3b5d2fbbe8a4714b.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/queue.o.d" -o ${OBJECTDIR}/_ext/1276567923/queue.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/tasks.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c  .generated_files/8bd9ef7325c6df516bf7ff49d1c0859d83b5a4a1.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/tasks.o.d" -o ${OBJECTDIR}/_ext/1276567923/tasks.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/timers.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c  .generated_files/d70e729d6e150c7057d2c58124f53d60438d5241.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/timers.o.d" -o ${OBJECTDIR}/_ext/1276567923/timers.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/event_groups.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c  .generated_files/e66a827b04e01dc1dec35d58d3827f774edc214.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/event_groups.o.d" -o ${OBJECTDIR}/_ext/1276567923/event_groups.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/stream_buffer.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c  .generated_files/7daa3b3380661a9fa0049bba48979dd2f9eae0c0.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/stream_buffer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d" -o ${OBJECTDIR}/_ext/1276567923/stream_buffer.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/443942460/PID.o: ../MicroDrone-Firmware/src/Control/PID/PID.c  .generated_files/a788673c0c3dd08d690b4e07bbbef1f3ffbdac30.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/443942460" 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o.d 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/443942460/PID.o.d" -o ${OBJECTDIR}/_ext/443942460/PID.o ../MicroDrone-Firmware/src/Control/PID/PID.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o: ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c  .generated_files/276dcf33f4a3d0d096b25113d5269c55f51c7e64.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1035051164" 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" -o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o: ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c  .generated_files/2ec07e024b5cbf32463d72dbfef73f1b3f0a4a8c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" -o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/SensFusion.o: ../MicroDrone-Firmware/src/Control/SensFusion.c  .generated_files/97ba79e63a91b3b707cbc07ecf1346e03223ade1.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" -o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ../MicroDrone-Firmware/src/Control/SensFusion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o: ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c  .generated_files/37b0a9c5b642378368a9b5434d2c16ad09e2520c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" -o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/AttitudeController.o: ../MicroDrone-Firmware/src/Control/AttitudeController.c  .generated_files/d4f9acfcfcf66f1a38ebe39f51ff4b37336577bf.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" -o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ../MicroDrone-Firmware/src/Control/AttitudeController.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/149289479/MAVLinkSender.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c  .generated_files/4257900be068bafe4603a16f3224880f84e95393.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c  .generated_files/b87f430650a51ce9f8db3b818ebb022fb749c23b.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o: ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c  .generated_files/3a1f00481cb5407d132320410b34dd92201a0099.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1715884449" 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" -o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c  .generated_files/cbb72c01c72af0dff19b3f7a6caa80402cc26930.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c  .generated_files/37e62483fb2cd739e835560ce9ad435c0b75bea0.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c  .generated_files/911a071b624a724c361d39108d448527b51cc582.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o: ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c  .generated_files/48c5a224f5f369ad71a4d0e0aebd28f09ac894e4.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c  .generated_files/5fa22b72dd109b6b62f12aedbbe005000d1d7964.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1313821444/num.o: ../MicroDrone-Firmware/src/Utils/num.c  .generated_files/1889bd8099e9740b9113ad6711ec886093f6f2b8.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1313821444" 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o.d 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1313821444/num.o.d" -o ${OBJECTDIR}/_ext/1313821444/num.o ../MicroDrone-Firmware/src/Utils/num.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
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
