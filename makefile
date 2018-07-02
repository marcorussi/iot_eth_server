#The MIT License (MIT)
#
#Copyright (c) 2018 Marco Russi
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.


PROJECT_NAME := iot_server

LD_FOLDER = ldscripts

TOOLS_FOLDER = tools

LINKER_SCRIPT_LIB := $(LD_FOLDER)/libs.ld
LINKER_SCRIPT_MEM := $(LD_FOLDER)/mem.ld
LINKER_SCRIPT_SEC := $(LD_FOLDER)/sections.ld

LD_PATH = $(LD_FOLDER)

TOOLS_PATH = $(TOOLS_FOLDER)

GNU_INSTALL_ROOT := /opt/gcc-arm-none-eabi-7-2017-q4-major

APP_PATH := app
CFG_PATH := config
MID_PATH := midware
RTOS_PATH := rtos
SYS_PATH := system
TCPIP_PATH := tcpip

export OUTPUT_FILENAME

GNU_PREFIX := arm-none-eabi

MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

MK := mkdir
RM := rm -rf

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE 		:= '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

OUTPUT_FILENAME = $(PROJECT_NAME)_out

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

C_SOURCE_FILES += \
$(wildcard $(APP_PATH)/src/*.c) \
$(wildcard $(MID_PATH)/src/*.c) \
$(wildcard $(RTOS_PATH)/src/*.c) \
$(wildcard $(SYS_PATH)/src/cmsis/*.c) \
$(wildcard $(SYS_PATH)/src/cortexm/*.c) \
$(wildcard $(SYS_PATH)/src/diag/*.c) \
$(wildcard $(SYS_PATH)/src/newlib/*.c) \
$(wildcard $(SYS_PATH)/src/stm32f4-hal/*.c) \
$(wildcard $(TCPIP_PATH)/src/*c) \

#assembly files common to all targets
#ASM_SOURCE_FILES = $(abspath $(INCDEV_PATH)/src/startup_stm32f411xe.s)

INC_PATHS = -I$(abspath $(CFG_PATH))
INC_PATHS += -I$(abspath $(APP_PATH)/inc)
INC_PATHS += -I$(abspath $(MID_PATH)/inc)
INC_PATHS += -I$(abspath $(RTOS_PATH)/inc)
INC_PATHS += -I$(abspath $(SYS_PATH)/inc)
INC_PATHS += -I$(abspath $(SYS_PATH)/inc/cmsis)
INC_PATHS += -I$(abspath $(SYS_PATH)/inc/cortexm)
INC_PATHS += -I$(abspath $(SYS_PATH)/inc/diag)
INC_PATHS += -I$(abspath $(SYS_PATH)/inc/newlib)
INC_PATHS += -I$(abspath $(SYS_PATH)/inc/stm32f4-hal)
INC_PATHS += -I$(abspath $(TCPIP_PATH)/inc)

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

#ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
#ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
#ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
#vpath %.s $(ASM_PATHS)

CDEFS       = -DSTM32F411xE
CDEFS      += -DUSE_FULL_ASSERT
CDEFS      += -DDEBUG
CDEFS      += -DUSE_HAL_DRIVER 
CDEFS      += -DHSE_VALUE=8000000
MCUFLAGS    = -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16
COMMONFLAGS = -Og -g3 -Wall -Wextra -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -fsigned-char -fmessage-length=0

# C flags
CFLAGS   = $(COMMONFLAGS) $(MCUFLAGS) $(CDEFS)

# Assembler flags
#ASMFLAGS  = -x assembler-with-cpp
#ASMFLAGS += -DSTM32F411xE

# Linker flags
LDFLAGS  = $(MCUFLAGS)      
LDFLAGS += $(COMMONFLAGS)
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -specs=nano.specs -nostartfiles
LDFLAGS += -L $(LD_PATH) -T$(LINKER_SCRIPT_LIB) -T$(LINKER_SCRIPT_MEM)  -T$(LINKER_SCRIPT_SEC) 

#$(ASM_OBJECTS)
OBJECTS = $(C_OBJECTS) 


default: clean stm32

all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e stm32

help:
	@echo ginger

stm32: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).elf
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files. WAS $(NO_ECHO)$(AS) $(ASMFLAGS) -o $@ $<
#$(OBJECT_DIRECTORY)/%.o: %.s
#	@echo Assembly file: $(notdir $<)
#	$(NO_ECHO)$(AS) -o $@ $<

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).elf
	-@echo ''

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

clean:
	$(RM) $(BUILD_DIRECTORIES)

flash:
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
	$(TOOLS_FOLDER)/megaprog.sh --flash-hex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex







