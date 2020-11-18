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
SOURCEFILES_QUOTED_IF_SPACED=../MicroDrone-Firmware/HAL/PIC_HV2/imu.c ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c ../MicroDrone-Firmware/libs/Math/Quaternion.c ../MicroDrone-Firmware/libs/Math/Vector3D.c ../src/main.c ../src/app.c ../src/mavlink_recv_task.c ../src/mavlink_send_task.c ../src/mavlink_status_task.c ../src/att_controller_task.c ../src/serialhandler.c ../src/imu_update_task.c ../MicroDrone-Firmware/src/Control/PID/PID.c ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c ../MicroDrone-Firmware/src/Control/SensFusion.c ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c ../MicroDrone-Firmware/src/Control/AttitudeController.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c ../MicroDrone-Firmware/src/Utils/num.c ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/system_config/default/fassert.c ../src/system_config/default/system_tasks.c ../src/system_config/default/system_interrupt_a.S ../src/system_config/default/rtos_hooks.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1650642999/imu.o ${OBJECTDIR}/_ext/1650642999/comms.o ${OBJECTDIR}/_ext/1650642999/tof.o ${OBJECTDIR}/_ext/1650642999/motors.o ${OBJECTDIR}/_ext/971049567/Quaternion.o ${OBJECTDIR}/_ext/971049567/Vector3D.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ${OBJECTDIR}/_ext/1360937237/serialhandler.o ${OBJECTDIR}/_ext/1360937237/imu_update_task.o ${OBJECTDIR}/_ext/443942460/PID.o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ${OBJECTDIR}/_ext/1313821444/num.o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ${OBJECTDIR}/_ext/1276567923/croutine.o ${OBJECTDIR}/_ext/1276567923/list.o ${OBJECTDIR}/_ext/1276567923/queue.o ${OBJECTDIR}/_ext/1276567923/tasks.o ${OBJECTDIR}/_ext/1276567923/timers.o ${OBJECTDIR}/_ext/1276567923/event_groups.o ${OBJECTDIR}/_ext/1276567923/stream_buffer.o ${OBJECTDIR}/_ext/457403440/heap_1.o ${OBJECTDIR}/_ext/1571139743/port.o ${OBJECTDIR}/_ext/1571139743/port_asm.o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1688732426/fassert.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1650642999/imu.o.d ${OBJECTDIR}/_ext/1650642999/comms.o.d ${OBJECTDIR}/_ext/1650642999/tof.o.d ${OBJECTDIR}/_ext/1650642999/motors.o.d ${OBJECTDIR}/_ext/971049567/Quaternion.o.d ${OBJECTDIR}/_ext/971049567/Vector3D.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1360937237/app.o.d ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d ${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d ${OBJECTDIR}/_ext/1360937237/serialhandler.o.d ${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d ${OBJECTDIR}/_ext/443942460/PID.o.d ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d ${OBJECTDIR}/_ext/1313821444/num.o.d ${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d ${OBJECTDIR}/_ext/121284916/sys_devcon.o.d ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o.d ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d ${OBJECTDIR}/_ext/1276567923/croutine.o.d ${OBJECTDIR}/_ext/1276567923/list.o.d ${OBJECTDIR}/_ext/1276567923/queue.o.d ${OBJECTDIR}/_ext/1276567923/tasks.o.d ${OBJECTDIR}/_ext/1276567923/timers.o.d ${OBJECTDIR}/_ext/1276567923/event_groups.o.d ${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d ${OBJECTDIR}/_ext/457403440/heap_1.o.d ${OBJECTDIR}/_ext/1571139743/port.o.d ${OBJECTDIR}/_ext/1571139743/port_asm.o.d ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d ${OBJECTDIR}/_ext/1688732426/system_init.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d ${OBJECTDIR}/_ext/1688732426/fassert.o.d ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1650642999/imu.o ${OBJECTDIR}/_ext/1650642999/comms.o ${OBJECTDIR}/_ext/1650642999/tof.o ${OBJECTDIR}/_ext/1650642999/motors.o ${OBJECTDIR}/_ext/971049567/Quaternion.o ${OBJECTDIR}/_ext/971049567/Vector3D.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ${OBJECTDIR}/_ext/1360937237/serialhandler.o ${OBJECTDIR}/_ext/1360937237/imu_update_task.o ${OBJECTDIR}/_ext/443942460/PID.o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ${OBJECTDIR}/_ext/1313821444/num.o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/121284916/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ${OBJECTDIR}/_ext/1276567923/croutine.o ${OBJECTDIR}/_ext/1276567923/list.o ${OBJECTDIR}/_ext/1276567923/queue.o ${OBJECTDIR}/_ext/1276567923/tasks.o ${OBJECTDIR}/_ext/1276567923/timers.o ${OBJECTDIR}/_ext/1276567923/event_groups.o ${OBJECTDIR}/_ext/1276567923/stream_buffer.o ${OBJECTDIR}/_ext/457403440/heap_1.o ${OBJECTDIR}/_ext/1571139743/port.o ${OBJECTDIR}/_ext/1571139743/port_asm.o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ${OBJECTDIR}/_ext/1688732426/system_init.o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ${OBJECTDIR}/_ext/1688732426/fassert.o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o

# Source Files
SOURCEFILES=../MicroDrone-Firmware/HAL/PIC_HV2/imu.c ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c ../MicroDrone-Firmware/libs/Math/Quaternion.c ../MicroDrone-Firmware/libs/Math/Vector3D.c ../src/main.c ../src/app.c ../src/mavlink_recv_task.c ../src/mavlink_send_task.c ../src/mavlink_status_task.c ../src/att_controller_task.c ../src/serialhandler.c ../src/imu_update_task.c ../MicroDrone-Firmware/src/Control/PID/PID.c ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c ../MicroDrone-Firmware/src/Control/SensFusion.c ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c ../MicroDrone-Firmware/src/Control/AttitudeController.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c ../MicroDrone-Firmware/src/Utils/num.c ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port_asm.S ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c ../src/system_config/default/framework/system/ports/src/sys_ports_static.c ../src/system_config/default/system_init.c ../src/system_config/default/system_interrupt.c ../src/system_config/default/system_exceptions.c ../src/system_config/default/fassert.c ../src/system_config/default/system_tasks.c ../src/system_config/default/system_interrupt_a.S ../src/system_config/default/rtos_hooks.c



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
	
${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o: ../src/system_config/default/system_interrupt_a.S  .generated_files/f682c6f8be67462b25929733e7f0735b686ff86c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ../src/system_config/default/system_interrupt_a.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
else
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
	
${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o: ../src/system_config/default/system_interrupt_a.S  .generated_files/b4dd3fd629651a55829c095ef547067509a7db83.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.err 
	${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o ../src/system_config/default/system_interrupt_a.S  -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d",-I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -I"../src/system_config/default" -mdfp="${DFP_DIR}"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1688732426/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1650642999/imu.o: ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c  .generated_files/e3e9807249c7c9da351fa0f270224825abeeb5c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/imu.o.d" -o ${OBJECTDIR}/_ext/1650642999/imu.o ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/comms.o: ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c  .generated_files/fed683617d48cb677c2c7703811f838ab2c7417d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/comms.o.d" -o ${OBJECTDIR}/_ext/1650642999/comms.o ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/tof.o: ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c  .generated_files/df237f14efcf70f77f4b1a1fe0d0a0439e9b251c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/tof.o.d" -o ${OBJECTDIR}/_ext/1650642999/tof.o ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/motors.o: ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c  .generated_files/5b439bf6075b75bff54f62aa8887d193ee9b8268.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/motors.o.d" -o ${OBJECTDIR}/_ext/1650642999/motors.o ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/971049567/Quaternion.o: ../MicroDrone-Firmware/libs/Math/Quaternion.c  .generated_files/7617082e08a7c054221957650bc7029a6bac9f27.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" -o ${OBJECTDIR}/_ext/971049567/Quaternion.o ../MicroDrone-Firmware/libs/Math/Quaternion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/971049567/Vector3D.o: ../MicroDrone-Firmware/libs/Math/Vector3D.c  .generated_files/20350926daf6e795e2b1de71aaa6e8abd5b6a73e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" -o ${OBJECTDIR}/_ext/971049567/Vector3D.o ../MicroDrone-Firmware/libs/Math/Vector3D.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  .generated_files/ab781bd2bd826e63c192be23b9eb108d053f6e40.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  .generated_files/e38f5deb417d58e3fe09af0199a092f242e7c888.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o: ../src/mavlink_recv_task.c  .generated_files/beb2ef1cb0d28e325d9045201d99526397799ab.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ../src/mavlink_recv_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o: ../src/mavlink_send_task.c  .generated_files/5541868994605574bb6a87a597275d0a7fa4aa.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ../src/mavlink_send_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o: ../src/mavlink_status_task.c  .generated_files/260e162c31b28f45d5f77f278fb159ab1e9a5c58.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ../src/mavlink_status_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/att_controller_task.o: ../src/att_controller_task.c  .generated_files/91b6c1075371a92d07fe49a2f7abf1dfbc0b963c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ../src/att_controller_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/serialhandler.o: ../src/serialhandler.c  .generated_files/55c9e1343744fe2009ec0f646b721f0bd0c0fa16.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/serialhandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/serialhandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/serialhandler.o.d" -o ${OBJECTDIR}/_ext/1360937237/serialhandler.o ../src/serialhandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/imu_update_task.o: ../src/imu_update_task.c  .generated_files/ea8a67b04b8c56687baac2a41037d886ceba06e5.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/imu_update_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/imu_update_task.o ../src/imu_update_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/443942460/PID.o: ../MicroDrone-Firmware/src/Control/PID/PID.c  .generated_files/6b31bcbd0d49f17184acd90576180c0308eb2bee.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/443942460" 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o.d 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/443942460/PID.o.d" -o ${OBJECTDIR}/_ext/443942460/PID.o ../MicroDrone-Firmware/src/Control/PID/PID.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o: ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c  .generated_files/6165d656e24f33ab7321be4a1e663851bf9cd585.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1035051164" 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" -o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o: ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c  .generated_files/e0907084fd3e5bcc4ae9dbd3c0be7cdc94c6d485.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" -o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/SensFusion.o: ../MicroDrone-Firmware/src/Control/SensFusion.c  .generated_files/d2157890cf8bfc211cbe3a422773149f8a060ae7.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" -o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ../MicroDrone-Firmware/src/Control/SensFusion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o: ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c  .generated_files/97e463dbbfab6678a8e82c6ffadb6c6382a7c800.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" -o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/AttitudeController.o: ../MicroDrone-Firmware/src/Control/AttitudeController.c  .generated_files/3398f82f3812a2a24dbc422f84b7b01e5c61909f.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" -o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ../MicroDrone-Firmware/src/Control/AttitudeController.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/149289479/MAVLinkSender.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c  .generated_files/f224801a50ce0ca38bb6152bde897ed3d36451b6.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c  .generated_files/de0c994a48d148496762b1aa70f8a1208b553a51.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o: ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c  .generated_files/10f331eda9972303da93bc637cb8eeb782af255e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1715884449" 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" -o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c  .generated_files/f5180f826ba2a6ea6c1a836ffe3d2ba9fe02b4a6.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c  .generated_files/5af40b2e647c08c4c4841eeeacc25f1722c9e07.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c  .generated_files/f3723263aa0ec21dfa252f044c2da104565aabff.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o: ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c  .generated_files/d239d2e14d47a17acf36fc5ceb8b0c684aebec70.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c  .generated_files/b77dff74c4cb9f539d3e79a97d6f323c8b0164d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1313821444/num.o: ../MicroDrone-Firmware/src/Utils/num.c  .generated_files/aa2cf16abacfe72cd6000c760146bbaf4b5b23ce.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1313821444" 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o.d 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1313821444/num.o.d" -o ${OBJECTDIR}/_ext/1313821444/num.o ../MicroDrone-Firmware/src/Utils/num.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1353086322/osal_freertos.o: ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c  .generated_files/a7965021ce9bd382c4a2ad07d37157c94f6a16a3.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1353086322" 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/121284916/sys_devcon.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c  .generated_files/de155235dfb553fde605216fe102a6da096a9c09.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c  .generated_files/819fff669036b8e7f09c0229e725c9785e2f4bdc.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o: ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c  .generated_files/1bc996bfde952a1ee08efd0865991b685f8afba3.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1317643790" 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/croutine.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c  .generated_files/d8eeb2a95d1bc3ea6e4cc5c1a7b4b198a2ca7d42.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/croutine.o.d" -o ${OBJECTDIR}/_ext/1276567923/croutine.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/list.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c  .generated_files/cedb8523e3f245a4f289f28cc986f77e62a30820.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/list.o.d" -o ${OBJECTDIR}/_ext/1276567923/list.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/queue.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c  .generated_files/b866e902afa7ea39edb5e0dfb1ce70deb72fb0af.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/queue.o.d" -o ${OBJECTDIR}/_ext/1276567923/queue.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/tasks.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c  .generated_files/90d5e2ee754008162df84d9c0cc9441002419b91.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/tasks.o.d" -o ${OBJECTDIR}/_ext/1276567923/tasks.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/timers.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c  .generated_files/4856ca1caa4fa17d75a2d31e63ef678e800a371d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/timers.o.d" -o ${OBJECTDIR}/_ext/1276567923/timers.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/event_groups.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c  .generated_files/e627c3455bb9f928f081d48c82eb48c07c03d986.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/event_groups.o.d" -o ${OBJECTDIR}/_ext/1276567923/event_groups.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/stream_buffer.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c  .generated_files/a050e2a2f27ae6ff90059330bbfba90c6008d35a.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/stream_buffer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d" -o ${OBJECTDIR}/_ext/1276567923/stream_buffer.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/457403440/heap_1.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c  .generated_files/e1e6d35f88ceb444c32430439851931472c62cf6.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/457403440" 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o.d 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/457403440/heap_1.o.d" -o ${OBJECTDIR}/_ext/457403440/heap_1.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1571139743/port.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c  .generated_files/1c89a9b958230cd8f8eb2fca38054a130f0c5e93.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1571139743" 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1571139743/port.o.d" -o ${OBJECTDIR}/_ext/1571139743/port.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c  .generated_files/47c673a010a8b95ad7ca11110c9a3edbfef2a3ec.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1047219354/drv_oc_static.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c  .generated_files/b441905cbc312b7275754e0220436ea79eb79d72.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c  .generated_files/bbac7fa8b403a7dff81605f1262446c3d262582d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c  .generated_files/bbd91f439b17f529fdbaa398053d118450bfa140.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c  .generated_files/fe179a441cfffab4b5e662def844ad46c8ea44f7.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_static.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c  .generated_files/36e0ae6433e1972f16b4d68b07163f880d29fae0.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c  .generated_files/8b3840eb05061c394dfad3195bf1d6e3e246e70d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o: ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c  .generated_files/aff1ac51ab5fb027b411c079af552326215c15d5.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  .generated_files/6166775e35826b9b5387fd0c677b25e8eac54990.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  .generated_files/5a8107129b8f64ed716e034e8f264ba44e9955a3.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  .generated_files/6fc1eba5b90dd39070947586d437ffde2f172b55.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  .generated_files/ba05efb66969c80096f18da1333cf92775d77979.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/fassert.o: ../src/system_config/default/fassert.c  .generated_files/66800a7373dd25afc15324be27b7e662792a4ecb.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/fassert.o.d" -o ${OBJECTDIR}/_ext/1688732426/fassert.o ../src/system_config/default/fassert.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  .generated_files/238bd3fa8a5d00e763c940811d2fc02707570a91.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/rtos_hooks.o: ../src/system_config/default/rtos_hooks.c  .generated_files/9cf685e16db93419e10e8434e867aa91d5a03950.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ../src/system_config/default/rtos_hooks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/_ext/1650642999/imu.o: ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c  .generated_files/12c7c97ec2d94ebab83f3884fd50aa4418ab0420.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/imu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/imu.o.d" -o ${OBJECTDIR}/_ext/1650642999/imu.o ../MicroDrone-Firmware/HAL/PIC_HV2/imu.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/comms.o: ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c  .generated_files/1f9bb61d9378792f0c6301db68daa15222539a20.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/comms.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/comms.o.d" -o ${OBJECTDIR}/_ext/1650642999/comms.o ../MicroDrone-Firmware/HAL/PIC_HV2/comms.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/tof.o: ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c  .generated_files/f27d67b51d1eb7ad3b331411a627e86cf884132.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/tof.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/tof.o.d" -o ${OBJECTDIR}/_ext/1650642999/tof.o ../MicroDrone-Firmware/HAL/PIC_HV2/tof.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1650642999/motors.o: ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c  .generated_files/412f6226ee8c2517c13994e1be37f019358ef945.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1650642999" 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650642999/motors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1650642999/motors.o.d" -o ${OBJECTDIR}/_ext/1650642999/motors.o ../MicroDrone-Firmware/HAL/PIC_HV2/motors.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/971049567/Quaternion.o: ../MicroDrone-Firmware/libs/Math/Quaternion.c  .generated_files/541c684360bd9436a4a154aa98f048f96950c1e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Quaternion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/971049567/Quaternion.o.d" -o ${OBJECTDIR}/_ext/971049567/Quaternion.o ../MicroDrone-Firmware/libs/Math/Quaternion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/971049567/Vector3D.o: ../MicroDrone-Firmware/libs/Math/Vector3D.c  .generated_files/f6636d06889da4f1753b045a48d3d5d76fc5e0e6.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/971049567" 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o.d 
	@${RM} ${OBJECTDIR}/_ext/971049567/Vector3D.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/971049567/Vector3D.o.d" -o ${OBJECTDIR}/_ext/971049567/Vector3D.o ../MicroDrone-Firmware/libs/Math/Vector3D.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  .generated_files/d5310030ea869e79e02046c32e2610edb077d17d.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  .generated_files/9961cb39e433a8eb0a359c9b22f8205f7820d3a0.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o: ../src/mavlink_recv_task.c  .generated_files/37357c1325407e5e8f60005838d385340a63e01e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_recv_task.o ../src/mavlink_recv_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o: ../src/mavlink_send_task.c  .generated_files/5b4ac8fe91703c0c2349306c7f724ca78679f09.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_send_task.o ../src/mavlink_send_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o: ../src/mavlink_status_task.c  .generated_files/e9fdcc39bf39e0324a23949576c7f9c5b3574809.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/mavlink_status_task.o ../src/mavlink_status_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/att_controller_task.o: ../src/att_controller_task.c  .generated_files/64acc4a957529f81b636bcf85292be1ec6a02752.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/att_controller_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/att_controller_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/att_controller_task.o ../src/att_controller_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/serialhandler.o: ../src/serialhandler.c  .generated_files/19b5917014f2a95628e29a1bd8fde8b8b43cd817.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/serialhandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/serialhandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/serialhandler.o.d" -o ${OBJECTDIR}/_ext/1360937237/serialhandler.o ../src/serialhandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1360937237/imu_update_task.o: ../src/imu_update_task.c  .generated_files/dc6da2aa4a739a0ec32eee2aa87fb0b51578b11a.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/imu_update_task.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/imu_update_task.o.d" -o ${OBJECTDIR}/_ext/1360937237/imu_update_task.o ../src/imu_update_task.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/443942460/PID.o: ../MicroDrone-Firmware/src/Control/PID/PID.c  .generated_files/a15d0c086aa789be4a96c345d35de6777a2312c9.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/443942460" 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o.d 
	@${RM} ${OBJECTDIR}/_ext/443942460/PID.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/443942460/PID.o.d" -o ${OBJECTDIR}/_ext/443942460/PID.o ../MicroDrone-Firmware/src/Control/PID/PID.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o: ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c  .generated_files/6e7cfc26c6b09a5864dd01da37f7ed0581d9c8fd.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1035051164" 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d 
	@${RM} ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o.d" -o ${OBJECTDIR}/_ext/1035051164/SixDOFQuadcopterModel.o ../MicroDrone-Firmware/src/Control/StateSpace/SixDOFQuadcopterModel.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o: ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c  .generated_files/1e529fc5e7496bc8253f45dfc21a4866f7779cdc.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o.d" -o ${OBJECTDIR}/_ext/1204493032/PositionEstimatorAltitude.o ../MicroDrone-Firmware/src/Control/PositionEstimatorAltitude.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/SensFusion.o: ../MicroDrone-Firmware/src/Control/SensFusion.c  .generated_files/f72b4c5d9c68286c0e730cc24fc4be01151970c4.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/SensFusion.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/SensFusion.o.d" -o ${OBJECTDIR}/_ext/1204493032/SensFusion.o ../MicroDrone-Firmware/src/Control/SensFusion.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o: ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c  .generated_files/a8f95679dd39a35ce31b7b6f6aefa2eeaa46bbcd.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o.d" -o ${OBJECTDIR}/_ext/1204493032/ComplementaryFilter.o ../MicroDrone-Firmware/src/Control/ComplementaryFilter.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1204493032/AttitudeController.o: ../MicroDrone-Firmware/src/Control/AttitudeController.c  .generated_files/4f52a387d1c73431141380c05050be5596cf0913.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1204493032" 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1204493032/AttitudeController.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1204493032/AttitudeController.o.d" -o ${OBJECTDIR}/_ext/1204493032/AttitudeController.o ../MicroDrone-Firmware/src/Control/AttitudeController.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/149289479/MAVLinkSender.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c  .generated_files/308e6db752251b4212a95099348f8f967b5b22e4.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkSender.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkSender.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkSender.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o: ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c  .generated_files/df198d617aa2e864c558797384172d0187fa8309.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/149289479" 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d 
	@${RM} ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o.d" -o ${OBJECTDIR}/_ext/149289479/MAVLinkHandler.o ../MicroDrone-Firmware/src/MAVLink/MAVLinkHandler.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o: ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c  .generated_files/6a493171292c50e60cb36931a60828086f4d6649.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1715884449" 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o.d" -o ${OBJECTDIR}/_ext/1715884449/AttitudeControllerTask.o ../MicroDrone-Firmware/src/Tasks/AttitudeControllerTask/AttitudeControllerTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c  .generated_files/9fa7a6c96e203cb9ff73b54daeeeb3b8e2ee0d5c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkSendTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkSendTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o: ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c  .generated_files/80ac5c3365b68983ea41c16ae7cefa011d53d796.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1158199258" 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o.d" -o ${OBJECTDIR}/_ext/1158199258/MAVLinkRecvTask.o ../MicroDrone-Firmware/src/Tasks/MAVLink/MAVLinkRecvTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c  .generated_files/cc1978ef637c392f20b66e3dbd7afa2e3d15b51.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/TOFUpdateTask.o ../MicroDrone-Firmware/src/Tasks/TOFUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o: ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c  .generated_files/62948857284a6f23c3c7e519711c3bb75d64335a.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/MAVStatusTask.o ../MicroDrone-Firmware/src/Tasks/MAVStatusTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o: ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c  .generated_files/146cc63bdd2573098213a77ff532ed494a42c0eb.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1315301415" 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d 
	@${RM} ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o.d" -o ${OBJECTDIR}/_ext/1315301415/IMUUpdateTask.o ../MicroDrone-Firmware/src/Tasks/IMUUpdateTask.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1313821444/num.o: ../MicroDrone-Firmware/src/Utils/num.c  .generated_files/feae4307d00a556681dea3762aea4b094b49c310.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1313821444" 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o.d 
	@${RM} ${OBJECTDIR}/_ext/1313821444/num.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1313821444/num.o.d" -o ${OBJECTDIR}/_ext/1313821444/num.o ../MicroDrone-Firmware/src/Utils/num.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1353086322/osal_freertos.o: ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c  .generated_files/4df78db1ceab1493314ad0e2adc3268b2eb75390.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1353086322" 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1353086322/osal_freertos.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1353086322/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/1353086322/osal_freertos.o ../../../../../microchip/harmony/v2_06/framework/osal/src/osal_freertos.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/121284916/sys_devcon.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c  .generated_files/632db12ad0e2cb192931a601e0c509514972d597.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o: ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c  .generated_files/a36bb547570265763294524f15ed9b7d7ba5a024.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/121284916" 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/121284916/sys_devcon_pic32mz.o ../../../../../microchip/harmony/v2_06/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o: ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c  .generated_files/f2c8db117e77c700f10e042ed2a78c0db57a2ff2.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1317643790" 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/1317643790/sys_int_pic32.o ../../../../../microchip/harmony/v2_06/framework/system/int/src/sys_int_pic32.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/croutine.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c  .generated_files/5389a97134270eddeec47df8b85ace1af4532344.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/croutine.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/croutine.o.d" -o ${OBJECTDIR}/_ext/1276567923/croutine.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/list.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c  .generated_files/b1f8b2e26ab04e7e792c644c9565818d6a26809.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/list.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/list.o.d" -o ${OBJECTDIR}/_ext/1276567923/list.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/queue.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c  .generated_files/c430c37b2d95328b85b68fad4b1d8b2ccf819e79.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/queue.o.d" -o ${OBJECTDIR}/_ext/1276567923/queue.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/tasks.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c  .generated_files/616ab0759d502243df746ffd8835c707b7d54fd1.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/tasks.o.d" -o ${OBJECTDIR}/_ext/1276567923/tasks.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/timers.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c  .generated_files/b9326a499c6689c65aaf312ae319e140df4e255f.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/timers.o.d" -o ${OBJECTDIR}/_ext/1276567923/timers.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/event_groups.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c  .generated_files/577e004b7efefb1f1c2bad95d67938f91aa3b50e.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/event_groups.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/event_groups.o.d" -o ${OBJECTDIR}/_ext/1276567923/event_groups.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1276567923/stream_buffer.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c  .generated_files/9ba9dbf04f41e3f05b5c5762ad4ca4e10948a309.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1276567923" 
	@${RM} ${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1276567923/stream_buffer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1276567923/stream_buffer.o.d" -o ${OBJECTDIR}/_ext/1276567923/stream_buffer.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/stream_buffer.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/457403440/heap_1.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c  .generated_files/56f15cec89535565a712aeaf16f4a1487da68238.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/457403440" 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o.d 
	@${RM} ${OBJECTDIR}/_ext/457403440/heap_1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/457403440/heap_1.o.d" -o ${OBJECTDIR}/_ext/457403440/heap_1.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1571139743/port.o: ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c  .generated_files/a4adf2d9ce9f9cf99e308f904c7e038a2fa286bc.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1571139743" 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1571139743/port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1571139743/port.o.d" -o ${OBJECTDIR}/_ext/1571139743/port.o ../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ/port.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c  .generated_files/c00952acc711584c1a67afa347dc6b26f806a050.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_mapping.o ../src/system_config/default/framework/driver/oc/src/drv_oc_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1047219354/drv_oc_static.o: ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c  .generated_files/9d39befc0133e7572e949e831d57a98755de19f6.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1047219354" 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1047219354/drv_oc_static.o.d" -o ${OBJECTDIR}/_ext/1047219354/drv_oc_static.o ../src/system_config/default/framework/driver/oc/src/drv_oc_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c  .generated_files/a193ed927b19725135088f1e6dd33c322c18d34.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_static.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o: ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c  .generated_files/2702fe282f8e1acc2592015eaed336ea432f8ae6.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1407244131" 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o.d" -o ${OBJECTDIR}/_ext/1407244131/drv_tmr_mapping.o ../src/system_config/default/framework/driver/tmr/src/drv_tmr_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c  .generated_files/3a62de368b2f46539a6f31d172a704b502d910cf.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_mapping.o ../src/system_config/default/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_static.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c  .generated_files/65b365785305ffa87e90a52ce5adfdf49d882041.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o: ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c  .generated_files/f01c1a9c456e8426fc18a0388d6bdbbb5e4be3ae.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/327000265" 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d 
	@${RM} ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o.d" -o ${OBJECTDIR}/_ext/327000265/drv_usart_static_byte_model.o ../src/system_config/default/framework/driver/usart/src/drv_usart_static_byte_model.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o: ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c  .generated_files/8c65dec3ec5e7abb7cc616e27e1d24062f6649ae.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/639803181" 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o.d" -o ${OBJECTDIR}/_ext/639803181/sys_clk_pic32mz.o ../src/system_config/default/framework/system/clk/src/sys_clk_pic32mz.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/822048611/sys_ports_static.o: ../src/system_config/default/framework/system/ports/src/sys_ports_static.c  .generated_files/b22104f2afbc2f275f45fd59977ed23a5b684ff8.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/822048611" 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/822048611/sys_ports_static.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/822048611/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/822048611/sys_ports_static.o ../src/system_config/default/framework/system/ports/src/sys_ports_static.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_init.o: ../src/system_config/default/system_init.c  .generated_files/da660421b0bc4b8a275512173d5c6023111222ea.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_init.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_init.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_init.o ../src/system_config/default/system_init.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_interrupt.o: ../src/system_config/default/system_interrupt.c  .generated_files/ee75dbd7c977f8c94215c47a0618ac2906790ad3.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_interrupt.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_interrupt.o ../src/system_config/default/system_interrupt.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_exceptions.o: ../src/system_config/default/system_exceptions.c  .generated_files/fe191879499cc68cb2536cdf55bb3e887bdd0d03.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_exceptions.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_exceptions.o ../src/system_config/default/system_exceptions.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/fassert.o: ../src/system_config/default/fassert.c  .generated_files/f1f7bf4d379dd07bdbb0ee8399ce87daa4f27cb.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/fassert.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/fassert.o.d" -o ${OBJECTDIR}/_ext/1688732426/fassert.o ../src/system_config/default/fassert.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/system_tasks.o: ../src/system_config/default/system_tasks.c  .generated_files/7a2baa3fad3f6630780e6c403d4e74f3ca3bbee2.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/system_tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1688732426/system_tasks.o ../src/system_config/default/system_tasks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1688732426/rtos_hooks.o: ../src/system_config/default/rtos_hooks.c  .generated_files/a3984620b6d74dad60e0bfb25747505fdd566f5c.flag .generated_files/15970c80474d7ccc9eb71b333afed3d7e6696f52.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1688732426" 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -mlong-calls -O1 -I"../MicroDrone-Firmware/HAL" -I"../MicroDrone-Firmware/libs/MAVLinkV2/common" -I"../MicroDrone-Firmware/libs/Math" -I"../MicroDrone-Firmware/src" -I"../src" -I"../src/system_config/default" -I"../src/default" -I"../../../../../microchip/harmony/v2_06/framework" -I"../src/system_config/default/framework" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/include" -I"../../../../../microchip/harmony/v2_06/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MZ" -MP -MMD -MF "${OBJECTDIR}/_ext/1688732426/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1688732426/rtos_hooks.o ../src/system_config/default/rtos_hooks.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
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
