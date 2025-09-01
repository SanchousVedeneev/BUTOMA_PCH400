DEBUG = 1
# optimization
OPT = -Og
#######################################
ASM_SOURCES =  \
startup_stm32g474xx.s
#######################################
#######################################
# cpu
CPU = -mcpu=cortex-m4
# fpu
FPU = -mfpu=fpv4-sp-d16
# float-abi
FLOAT-ABI = -mfloat-abi=hard
# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)
# macros for gcc
# AS defines
AS_DEFS = 
# C defines
C_DEFS =  \
-DUSE_FULL_LL_DRIVER \
-DUSE_HAL_DRIVER \
-DSTM32G474xx
# AS includes
AS_INCLUDES = 
# C includes
#######################################
# link script
LDSCRIPT = STM32G474VETx_FLASH.ld
# libraries
CUBE_C_FILES = \
Core/Src/main.c \
Core/Src/gpio.c \
Core/Src/adc.c \
Core/Src/comp.c \
Core/Src/cordic.c \
Core/Src/dac.c \
Core/Src/dma.c \
Core/Src/fdcan.c \
Core/Src/i2c.c \
Core/Src/tim.c \
Core/Src/usart.c \
Core/Src/stm32g4xx_it.c \
Core/Src/stm32g4xx_hal_msp.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_utils.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_exti.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_gpio.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_dma.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_comp.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_cordic.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c \
Core/Src/system_stm32g4xx.c \
Core/Src/spi.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c \
Core/Src/hrtim.c \
Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_hrtim.c
# ASM sources
CUBE_C_INCLUDES = \
-ICore/Inc \
-IDrivers/STM32G4xx_HAL_Driver/Inc \
-IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
-IDrivers/CMSIS/Include
# compile gcc flags
