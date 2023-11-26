cmd_src/init/main.o := arm-none-eabi-gcc -Wp,-MD,src/init/.main.o.d    -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/init -Isrc/init -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/libdw1000/inc   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/config   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/platform/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/drivers/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/bosch/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/esp32/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/outlierfilter   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/cpx   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/p2pDTR   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/controller   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/estimator   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/kve   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/tdoa   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/FatFS   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/vl53l1   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/pbl/Desktop/Swarm/uwb-software-library/examples/ex1a_simple_ranging_ini/build/include/generated -fno-delete-null-pointer-checks -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/home/pbl/Desktop/Swarm/uwb-software-library/examples/ex1a_simple_ranging_ini/../../inc   -I/home/pbl/Desktop/Swarm/uwb-software-library/examples/ex1a_simple_ranging_ini/../../decawave_api -Wno-error   -c -o src/init/main.o /home/pbl/Desktop/Swarm/crazyflie-firmware/src/init/main.c

source_src/init/main.o := /home/pbl/Desktop/Swarm/crazyflie-firmware/src/init/main.c

deps_src/init/main.o := \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdint.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdbool.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/config/trace.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/usec_time.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/cfassert.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stddef.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/config/config.h \
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
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/system.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/led.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/bootloader.h \

src/init/main.o: $(deps_src/init/main.o)

$(deps_src/init/main.o):
