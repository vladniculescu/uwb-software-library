cmd_src/modules/src/system.o := arm-none-eabi-gcc -Wp,-MD,src/modules/src/.system.o.d    -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/src -Isrc/modules/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/libdw1000/inc   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/config   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/platform/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/drivers/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/bosch/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/esp32/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/outlierfilter   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/cpx   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/p2pDTR   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/controller   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/estimator   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/kve   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/tdoa   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/FatFS   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/vl53l1   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/pbl/Desktop/Swarm/uwb-software-library/examples/ex1a_simple_ranging_ini/build/include/generated -fno-delete-null-pointer-checks -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/home/pbl/Desktop/Swarm/uwb-software-library/examples/ex1a_simple_ranging_ini/../../inc   -I/home/pbl/Desktop/Swarm/uwb-software-library/examples/ex1a_simple_ranging_ini/../../decawave_api -Wno-error   -c -o src/modules/src/system.o /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/src/system.c

source_src/modules/src/system.o := /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/src/system.c

deps_src/modules/src/system.o := \
    $(wildcard include/config/enable/cpx.h) \
    $(wildcard include/config/app/enable.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
    $(wildcard include/config/debug/print/on/uart1.h) \
    $(wildcard include/config/debug/print/on/uart1/baudrate.h) \
    $(wildcard include/config/estimator/kalman/enable.h) \
    $(wildcard include/config/estimator/ukf/enable.h) \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdbool.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stddef.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdint.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/config/trace.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/usec_time.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/cfassert.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/semphr.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/debug.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/config/config.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/console.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/eprintf.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdarg.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/version.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/param.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/param_logic.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/crtp.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/log.h \
    $(wildcard include/config/debug/log/enable.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/ledseq.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/led.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/pm.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/adc.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/syslink.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/interface/deck.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/interface/deck_core.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/estimator/estimator.h \
    $(wildcard include/config/estimator/oot.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/imu_types.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/interface/deck_constants.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/config/stm32fxxx.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/stm32f4xx.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/core_cm4.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_version.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/mpu_armv7.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/system_stm32f4xx.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/config/stm32f4xx_conf.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_crc.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dbgmcu.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dma.h \
    $(wildcard include/config/it.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_exti.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_flash.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_i2c.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_iwdg.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_pwr.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rtc.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_sdio.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_spi.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_syscfg.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_tim.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_usart.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_wwdg.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_misc.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_cryp.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_hash.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rng.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_can.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dac.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dcmi.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_fsmc.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/interface/deck_digital.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/interface/deck_analog.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/interface/deck_spi.h \
  /usr/include/newlib/string.h \
  /usr/include/newlib/_ansi.h \
  /usr/include/newlib/newlib.h \
  /usr/include/newlib/_newlib_version.h \
  /usr/include/newlib/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/include/newlib/machine/ieeefp.h \
  /usr/include/newlib/sys/features.h \
  /usr/include/newlib/sys/reent.h \
  /usr/include/newlib/_ansi.h \
  /usr/include/newlib/sys/_types.h \
  /usr/include/newlib/machine/_types.h \
  /usr/include/newlib/machine/_default_types.h \
  /usr/include/newlib/sys/lock.h \
  /usr/include/newlib/sys/cdefs.h \
  /usr/include/newlib/sys/_locale.h \
  /usr/include/newlib/strings.h \
  /usr/include/newlib/sys/string.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/system.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/platform/interface/platform.h \
    $(wildcard include/config/sensors/bmi088/bmp388.h) \
    $(wildcard include/config/sensors/bmi088/spi.h) \
    $(wildcard include/config/sensors/mpu9250/lps25h.h) \
    $(wildcard include/config/sensors/bosch.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/motors.h \
    $(wildcard include/config/motors/esc/protocol/oneshot125.h) \
    $(wildcard include/config/motors/esc/protocol/oneshot42.h) \
    $(wildcard include/config/motors/esc/protocol/dshot.h) \
    $(wildcard include/config/motors/dshot/pwm/150khz.h) \
    $(wildcard include/config/motors/dshot/pwm/300khz.h) \
    $(wildcard include/config/motors/dshot/pwm/600khz.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/storage.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/configblock.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/worker.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/freeRTOSdebug.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/uart_syslink.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/syslink.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/uart1.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/uart2.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/comm.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/stabilizer.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/commander.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/usblink.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/mem.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/crtp_mem.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/proximity.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/watchdog.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/queuemonitor.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/buzzer.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/sound.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/sysload.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/estimator/estimator_kalman.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/estimator/estimator_ukf.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/extrx.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/app.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/static_mem.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/peer_localization.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/math3d.h \
  /usr/include/newlib/math.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/i2cdev.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/i2c_drv.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/vcp_esc_passthrough.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/cpx/cpxlink.h \

src/modules/src/system.o: $(deps_src/modules/src/system.o)

$(deps_src/modules/src/system.o):
