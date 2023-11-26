cmd_src/deck/drivers/src/lpsTwrTag.o := arm-none-eabi-gcc -Wp,-MD,src/deck/drivers/src/.lpsTwrTag.o.d    -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/drivers/src -Isrc/deck/drivers/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/libdw1000/inc   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/config   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/platform/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/drivers/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/bosch/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/esp32/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/outlierfilter   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/cpx   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/p2pDTR   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/controller   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/estimator   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/kve   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/tdoa   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/FatFS   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/vl53l1   -I/home/pbl/Desktop/Swarm/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/pbl/Desktop/Swarm/uwb-software-library/examples/ex1a_simple_ranging_ini/build/include/generated -fno-delete-null-pointer-checks -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/home/pbl/Desktop/Swarm/uwb-software-library/examples/ex1a_simple_ranging_ini/../../inc   -I/home/pbl/Desktop/Swarm/uwb-software-library/examples/ex1a_simple_ranging_ini/../../decawave_api -Wno-error   -c -o src/deck/drivers/src/lpsTwrTag.o /home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/drivers/src/lpsTwrTag.c

source_src/deck/drivers/src/lpsTwrTag.o := /home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/drivers/src/lpsTwrTag.c

deps_src/deck/drivers/src/lpsTwrTag.o := \
    $(wildcard include/config/deck/loco/tdma/slots.h) \
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
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stddef.h \
  /usr/include/newlib/sys/_types.h \
  /usr/include/newlib/machine/_types.h \
  /usr/include/newlib/machine/_default_types.h \
  /usr/include/newlib/sys/lock.h \
  /usr/include/newlib/sys/cdefs.h \
  /usr/include/newlib/sys/_locale.h \
  /usr/include/newlib/strings.h \
  /usr/include/newlib/sys/string.h \
  /usr/include/newlib/math.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/drivers/interface/lpsTwrTag.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/drivers/interface/locodeck.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdint.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdbool.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/config/trace.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/usec_time.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/cfassert.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/libdw1000/inc/libdw1000.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/libdw1000/inc/libdw1000Spi.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/libdw1000/inc/dw1000.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/libdw1000/inc/libdw1000Types.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/hal/interface/imu_types.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/drivers/interface/mac.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/deck/drivers/interface/lpsTdma.h \
    $(wildcard include/config/deck/loco/tdma.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/log.h \
    $(wildcard include/config/debug/log/enable.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/crtp_localization_service.h \
    $(wildcard include/config/deck/lighthouse.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/lighthouse/pulse_processor.h \
    $(wildcard include/config/deck/lighthouse/max/n/bs.h) \
  /usr/include/newlib/stdlib.h \
  /usr/include/newlib/machine/stdlib.h \
  /usr/include/newlib/alloca.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/lighthouse/ootx_decoder.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_calibration.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_geometry.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/cf_math.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/arm_math.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/float.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include-fixed/limits.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/cfassert.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/estimator/estimator.h \
    $(wildcard include/config/estimator/kalman/enable.h) \
    $(wildcard include/config/estimator/ukf/enable.h) \
    $(wildcard include/config/estimator/oot.h) \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/physicalConstants.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/utils/interface/configblock.h \
  /home/pbl/Desktop/Swarm/crazyflie-firmware/src/modules/interface/static_mem.h \

src/deck/drivers/src/lpsTwrTag.o: $(deps_src/deck/drivers/src/lpsTwrTag.o)

$(deps_src/deck/drivers/src/lpsTwrTag.o):
