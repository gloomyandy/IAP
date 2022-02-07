#---IAP---
IAP_SRC_BASE  = $(IAP_DIR)/src

IAP_SRC_DIRS = Hardware/SharedSpi
ifeq ($(CONFIG),IAP_BOOT_LOADER)
IAP_SRC_DIRS += FatFS
endif
#Find the c and cpp source files
IAP_SRC = $(IAP_SRC_BASE) $(addprefix $(IAP_SRC_BASE)/, $(IAP_SRC_DIRS))
IAP_OBJ_SRC_C	   += $(foreach src, $(IAP_SRC), $(wildcard $(src)/*.c) ) 
IAP_OBJ_SRC_CXX   += $(foreach src, $(IAP_SRC), $(wildcard $(src)/*.cpp) )

IAP_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(IAP_OBJ_SRC_C)) $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(IAP_OBJ_SRC_CXX))

IAP_INCLUDES = $(addprefix -I, $(IAP_SRC))

#end IAP


