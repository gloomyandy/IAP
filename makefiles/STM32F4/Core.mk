CORE_DIR = $(CORESTM_DIR)


#Core
CORE_SRC_DIRS  = src src/STM32 src/STM32/usb src/STM32/usb/cdc
CORE_SRC_DIRS += src/STM32/system src/STM32/system/STM32F4xx src/STM32/system/Middlewares/ST/STM32_USB_Device_Library/Core/Inc src/STM32/system/Middlewares/ST/STM32_USB_Device_Library/Core/Src 
CORE_SRC_DIRS += src/STM32/system/Drivers/CMSIS/Device/ST/STM32F4xx/Include src/STM32/system/Drivers/CMSIS/Device/ST/STM32F4xx/Source
CORE_SRC_DIRS += src/STM32/system/Drivers/STM32F4xx_HAL_Driver/Inc src/STM32/system/Drivers/STM32F4xx_HAL_Driver/Src
CORE_SRC_DIRS += src/STM32/system/CMSIS/CMSIS/Core/Include src/STM32/system/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc
CORE_SRC_DIRS += src/STM32/variants/STM32F407 src/STM32/SharedSPI src/STM32/SDIO src/STM32/CRC32

CORE_SRC = $(CORE_DIR) $(addprefix $(CORE_DIR)/, $(CORE_SRC_DIRS))
CORE_INCLUDES = $(addprefix -I, $(CORE_SRC))

#Find all c and c++ files for Core
CORE_OBJ_SRC_C    += $(foreach src, $(CORE_SRC), $(wildcard $(src)/*.c))
CORE_OBJ_SRC_CXX   += $(foreach src, $(CORE_SRC), $(wildcard $(src)/*.cpp))
CORE_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(CORE_OBJ_SRC_C)) $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CORE_OBJ_SRC_CXX))
CORE_OBJS += $(BUILD_DIR)/CoreN2G/src/STM32/startup_stm32yyxx.o
