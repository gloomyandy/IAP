#---RepRapFirmware---
RRF_SRC_BASE  = $(REPRAPFIRMWARE_DIR)/src

#end RRF

#Libc and libcpp in RRF
RRFLIBC_SRC_DIRS = libc
RRFLIBC_SRC = $(addprefix $(RRF_SRC_BASE)/, $(RRFLIBC_SRC_DIRS))
RRFLIBC_OBJ_SRC_C	  += $(foreach src, $(RRFLIBC_SRC), $(wildcard $(src)/*.c) ) 
RRFLIBC_OBJ_SRC_CXX   += $(foreach src, $(RRFLIBC_SRC), $(wildcard $(src)/*.cpp) )
RRFLIBC_OBJ_SRC_CC    += $(foreach src, $(RRFLIBC_SRC), $(wildcard $(src)/*.cc) )

RRFLIBC_INCLUDES = $(addprefix -I, $(RRFLIBC_SRC))

RRFLIBC_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(RRFLIBC_OBJ_SRC_C)) $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(RRFLIBC_OBJ_SRC_CXX))
RRFLIBC_OBJS += $(patsubst %.cc,$(BUILD_DIR)/%.o,$(RRFLIBC_OBJ_SRC_CC))


#


