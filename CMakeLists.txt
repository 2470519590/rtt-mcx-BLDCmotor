cmake_minimum_required(VERSION 3.22.0)

set(SdkRootDirPath "${CMAKE_CURRENT_LIST_DIR}/../../SDK_26_06_00_FRDM-MCXA346/SDK_26_06_00_FRDM-MCXA346/mcuxsdk"
    CACHE PATH "MCUXpresso SDK root")
set(board_root examples/_boards)
set(board frdmmcxa346)
set(device MCXA346)
set(soc_series MCXA)

include(${SdkRootDirPath}/cmake/extension/mcux.cmake)

project(test1 LANGUAGES C CXX ASM
        PROJECT_BOARD_PORT_PATH ${board_root}/${board}/demo_apps/hello_world)

include(${SdkRootDirPath}/CMakeLists.txt)

# Keep the kernel source list explicit.  The RT-Thread src directory also
# contains SMP-only and optional allocators; globbing it makes a small UP
# Cortex-M build pull in incompatible files such as cpu.c/scheduler_mp.c.
set(RT_KERNEL_SOURCES
    rtthread/src/clock.c
    rtthread/src/components.c
    rtthread/src/idle.c
    rtthread/src/ipc.c
    rtthread/src/irq.c
    rtthread/src/kservice.c
    rtthread/src/mem.c
    rtthread/src/mempool.c
    rtthread/src/object.c
    rtthread/src/scheduler_up.c
    rtthread/src/thread.c
    rtthread/src/timer.c
)

set(RT_PORT_SOURCES
    rtthread/libcpu/arm/cortex-m33/cpuport.c
    rtthread/libcpu/arm/cortex-m33/context_gcc.S
    rtthread/libcpu/arm/cortex-m33/trustzone.c
)

mcux_add_source(
    BASE_PATH ${CMAKE_CURRENT_SOURCE_DIR}
    SOURCES app/rtt_main.c
            app/rtthread_port.c
            app/board_i2c.c
            app/as5600.c
            app/motor_control.c
            app/speed_estimator.c
            app/nn_controller.c
            app/angle_shadow.c
            app/simplefoc_pwm.c
            app/host_cli.c
            app/uart_rx_queue.c
            ${RT_KERNEL_SOURCES}
            ${RT_PORT_SOURCES}
)

mcux_add_include(
    BASE_PATH ${CMAKE_CURRENT_SOURCE_DIR}
    INCLUDES app
             rtthread/include
             rtthread/libcpu/arm/cortex-m33
)

mcux_add_source(
    BASE_PATH ${SdkRootDirPath}
    SOURCES drivers/lpi2c/fsl_lpi2c.c
            drivers/pwm/fsl_pwm.c
)

mcux_add_include(
    BASE_PATH ${SdkRootDirPath}
    INCLUDES drivers/lpi2c
             drivers/pwm
)

mcux_add_macro(CC "-DCPU_MCXA346VLQ")
mcux_add_macro(CC "-DI2C_RETRY_TIMES=10000")
mcux_add_macro(CC "-DUART_RETRY_TIMES=1000000")
mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)
