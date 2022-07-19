cmd_/module/examples/app_hello_world/src/hello_world.o := arm-none-eabi-gcc -Wp,-MD,/module/examples/app_hello_world/src/.hello_world.o.d     -I/module/examples/app_hello_world/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/module/vendor/CMSIS/CMSIS/Core/Include   -I/module/vendor/CMSIS/CMSIS/DSP/Include   -I/module/vendor/libdw1000/inc   -I/module/vendor/FreeRTOS/include   -I/module/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/module/src/config   -I/module/src/platform/interface   -I/module/src/deck/interface   -I/module/src/deck/drivers/interface   -I/module/src/drivers/interface   -I/module/src/drivers/bosch/interface   -I/module/src/drivers/esp32/interface   -I/module/src/hal/interface   -I/module/src/modules/interface   -I/module/src/modules/interface/kalman_core   -I/module/src/modules/interface/lighthouse   -I/module/src/utils/interface   -I/module/src/utils/interface/kve   -I/module/src/utils/interface/lighthouse   -I/module/src/utils/interface/tdoa   -I/module/src/lib/FatFS   -I/module/src/lib/CMSIS/STM32F4xx/Include   -I/module/src/lib/STM32_USB_Device_Library/Core/inc   -I/module/src/lib/STM32_USB_OTG_Driver/inc   -I/module/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/module/src/lib/vl53l1   -I/module/src/lib/vl53l1/core/inc   -I/module/examples/app_hello_world/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -c -o /module/examples/app_hello_world/src/hello_world.o /module/examples/app_hello_world/src/hello_world.c

source_/module/examples/app_hello_world/src/hello_world.o := /module/examples/app_hello_world/src/hello_world.c

deps_/module/examples/app_hello_world/src/hello_world.o := \
  /usr/local/arm-none-eabi/include/string.h \
  /usr/local/arm-none-eabi/include/_ansi.h \
  /usr/local/arm-none-eabi/include/newlib.h \
  /usr/local/arm-none-eabi/include/_newlib_version.h \
  /usr/local/arm-none-eabi/include/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/local/arm-none-eabi/include/machine/ieeefp.h \
  /usr/local/arm-none-eabi/include/sys/features.h \
  /usr/local/arm-none-eabi/include/sys/reent.h \
  /usr/local/arm-none-eabi/include/_ansi.h \
  /usr/local/lib/gcc/arm-none-eabi/9.3.1/include/stddef.h \
  /usr/local/arm-none-eabi/include/sys/_types.h \
  /usr/local/arm-none-eabi/include/machine/_types.h \
  /usr/local/arm-none-eabi/include/machine/_default_types.h \
  /usr/local/arm-none-eabi/include/sys/lock.h \
  /usr/local/arm-none-eabi/include/sys/cdefs.h \
  /usr/local/arm-none-eabi/include/sys/_locale.h \
  /usr/local/arm-none-eabi/include/strings.h \
  /usr/local/arm-none-eabi/include/sys/string.h \
  /usr/local/lib/gcc/arm-none-eabi/9.3.1/include/stdint.h \
  /usr/local/arm-none-eabi/include/stdint.h \
  /usr/local/arm-none-eabi/include/sys/_intsup.h \
  /usr/local/arm-none-eabi/include/sys/_stdint.h \
  /usr/local/lib/gcc/arm-none-eabi/9.3.1/include/stdbool.h \
  /module/src/modules/interface/app.h \
  /module/vendor/FreeRTOS/include/FreeRTOS.h \
  /module/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /module/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /module/src/drivers/interface/nrf24l01.h \
  /module/src/drivers/interface/nRF24L01reg.h \
  /module/src/config/trace.h \
  /module/src/hal/interface/usec_time.h \
  /module/src/utils/interface/cfassert.h \
  /module/src/config/stm32fxxx.h \
  /module/src/lib/CMSIS/STM32F4xx/Include/stm32f4xx.h \
  /module/vendor/CMSIS/CMSIS/Core/Include/core_cm4.h \
  /module/vendor/CMSIS/CMSIS/Core/Include/cmsis_version.h \
  /module/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /module/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /module/vendor/CMSIS/CMSIS/Core/Include/mpu_armv7.h \
  /module/src/lib/CMSIS/STM32F4xx/Include/system_stm32f4xx.h \
  /module/src/config/stm32f4xx_conf.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_crc.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dbgmcu.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dma.h \
    $(wildcard include/config/it.h) \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_exti.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_flash.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_i2c.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_iwdg.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_pwr.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rtc.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_sdio.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_spi.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_syscfg.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_tim.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_usart.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_wwdg.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_misc.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_cryp.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_hash.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rng.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_can.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dac.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dcmi.h \
  /module/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_fsmc.h \
  /module/src/modules/interface/console.h \
  /module/src/utils/interface/eprintf.h \
  /usr/local/lib/gcc/arm-none-eabi/9.3.1/include/stdarg.h \
  /module/vendor/FreeRTOS/include/projdefs.h \
  /module/vendor/FreeRTOS/include/portable.h \
  /module/vendor/FreeRTOS/include/deprecated_definitions.h \
  /module/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /module/vendor/FreeRTOS/include/mpu_wrappers.h \
  /module/vendor/FreeRTOS/include/task.h \
  /module/vendor/FreeRTOS/include/list.h \
  /module/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /module/src/config/config.h \

/module/examples/app_hello_world/src/hello_world.o: $(deps_/module/examples/app_hello_world/src/hello_world.o)

$(deps_/module/examples/app_hello_world/src/hello_world.o):
