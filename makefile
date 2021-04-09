###############################################################################
## Target (project/filename)
TARGET = blink-heartbeat

## Compile-time options
OPTIONS	= -Og

## Debugger optons, must be empty or GDB
DEBUG = 1
CONSOLE = 0

###############################################################################
## STATIC PATHS
GCC_PATH = 't:\tools-drivers-firmware-configs\programming\xpack-arm-none-eabi-gcc-10.2.1-1.1'
GCC_BIN = $(GCC_PATH)/bin
GCC_INCDIR = $(GCC_PATH)/arm-none-eabi/include

###############################################################################
## Working directories
#ROOT		 = $(dir $(lastword $(MAKEFILE_LIST)))
ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
BUILD_DIR 	 = build

SOURCEDIR 	 = ./src
LIBSDIR    	 = ./lib
DRVDIR		 = ./drivers
INCDIR 		 = ./inc

CORELIBDIR 	 = $(DRVDIR)/CMSIS
COREINCDIR	 = $(CORELIBDIR)/Include

DEVDIR  	 = $(DRVDIR)/CMSIS/Device/ST/STM32F4xx
DEVINCSRC 	 = $(DEVDIR)/Source
DEVINCDIR 	 = $(DEVDIR)/Include

STMLLDIR     = $(DRVDIR)/STM32F4xx_HAL_Driver
STMLLSRCDDIR = $(STMLLDIR)/Src
STMLLINCDDIR = $(STMLLDIR)/Inc

## Custom library folders
#TMDIR = $(LIBSDIR)/TM

###############################################################################
#
# Source files

## CPP Main source files common to all targets
#CPP_SRC = $(SOURCEDIR)/main.cpp

## C Main source files common to all targets
C_SRC = $(SOURCEDIR)/main.c
C_SRC += $(SOURCEDIR)/system_stm32f4xx.c
C_SRC += $(SOURCEDIR)/stm32f4xx_it.c

## used parts of the STM-ll-Library
C_SRC += $(wildcard $(STMLLSRCDDIR)/stm32f4xx_ll_*.c)

## used parts of the STM-hal-Library
#C_SRC += $(wildcard $(STMLLSRCDDIR)stm32f4xx_hal_*.c)

# ASM sources
ASM_SRC = $(SOURCEDIR)/startup_stm32f407xx.s

# C Custom library source files
#C_SRV += $(wildcard $(TMDIR)*.c)

###############################################################################
#
# Things that might need changing to use different tools
#
PREFIX = arm-none-eabi-

# Tool names
CC  = $(GCC_BIN)/$(PREFIX)gcc
PP 	= $(GCC_BIN)/$(PREFIX)g++
AS  = $(GCC_BIN)/$(PREFIX)gcc -x assembler-with-cpp
CP  = $(GCC_BIN)/$(PREFIX)objcopy
SZ 	= $(GCC_BIN)/$(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

###############################################################################
#
# Tool options.
#
          
# CFLAGS
#cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

###############################################################################
# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DSTM32F407xx \
-DUSE_FULL_LL_DRIVER \
-DHSE_VALUE=8000000 \
-DHSE_STARTUP_TIMEOUT=100 \
-DLSE_STARTUP_TIMEOUT=5000 \
-DLSE_VALUE=32768 \
-DEXTERNAL_CLOCK_VALUE=12288000 \
-DHSI_VALUE=16000000 \
-DLSI_VALUE=32000 \
-DVDD_VALUE=3300 \
-DPREFETCH_ENABLE=1 \
-DINSTRUCTION_CACHE_ENABLE=1 \
-DDATA_CACHE_ENABLE=1

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES = \
			-I$(INCDIR) \
			-I$(COREINCDIR) \
			-I$(STMLLINCDDIR) \
			-I$(DEVINCDIR) \
			-I$(GCC_INCDIR)
			#-I$(LIBSDIR) #\
			#-I$(TMDIR) 

###############################################################################
# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# XXX Map/crossref output?
#LD_SCRIPT	 = ./linker/STM32F407VETx_FLASH.ld
#LDFLAGS		 = -lm \
#		   $(ARCH_FLAGS) \	
#		   -static \
#		   -nostartfiles \
#		   -Wl,-gc-sections \
#		   -T$(LD_SCRIPT)

# link script
LDSCRIPT = ./ld/STM32F407VETx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

###############################################################################
ifeq ($(DEBUG), 1)
BUILD_DIR = debug
CFLAGS += -g -gdwarf-2
#C_DEFS += -DTRACE
endif
ifeq ($(CONSOLE), 1)
LDFLAGS = $(MCU) -specs=rdimon.specs -lc -lrdimon -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
endif


###############################################################################
# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

###############################################################################
# No user-serviceable parts below
###############################################################################
# list of C objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SRC:.c=.o)))
vpath %.c $(sort $(dir $(C_SRC)))
# list of CPP objects
#OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SRC:.cpp=.o)))
#vpath %.cpp $(sort $(dir $(CPP_SRC)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SRC:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SRC)))

#$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	#$(PP) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)/
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***