include FromCubeMX.mk

HEX_DIR=hex_dir

MODULE_PATHS:=App UserDrivers/ModbusSS UserDrivers/BSP UserDrivers/DSP UserDrivers/FlashSaver UserDrivers/FFT

MODULES_C:=$(shell find ${MODULE_PATHS} -type f -name "*.c")
MODULES_H_DIR:= $(addprefix -I,$(shell find "${MODULE_PATHS}" -type d) ${MODULE_PATHS})

$(info MODULES_C: $(MODULES_C))
$(info MODULES_H_DIR: $(MODULES_H_DIR))


######################################
# Modules
######################################
USER_MODULES_SRC = \
${MODULES_C}

# $(info src list: ${USER_MODULES_SRC})

USER_MODULES_INC = \
${MODULES_H_DIR}

# $(info USER_MODULES_INC: $(USER_MODULES_INC))


INCLUDE_MODULE_PATHS :=  $(addprefix -I,$(dir $(realpath $(MAKEFILE_LIST))))

#----------------------------------------------------- C
C_SOURCES = \
${CUBE_C_FILES} \
${USER_MODULES_SRC}


C_INCLUDES = \
${CUBE_C_INCLUDES} \
${USER_MODULES_INC} \
${INCLUDE_MODULE_PATHS}


#----------------------------------------------------- ALL INCLUDES
ALL_INCLUDES = \
$(C_INCLUDES) \
$(AS_INCLUDES)


#######################################
# tools
#######################################
PREFIX = arm-none-eabi-
#it can be added to the PATH environment variable.
GCC = $(PREFIX)gcc
ASM = $(PREFIX)gcc
LD = $(PREFIX)gcc
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
# Получение листинга из объектного файла
OBJDUMP = $(PREFIX)objdump

#$(patsubst %,-I%,$(include_paths))

# C_STANDARD = -std=c11 #-std=c99
# CPP_STANDARD = -std=c++11

# compile gcc flags
#----------------------------------------ASM
ASM_FLAGS = $(MCU) $(AS_DEFS) $(ALL_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

# ifeq ($(DEBUG), 1)
# ASM_FLAGS += -g -gdwarf-2  #Добавление отладочной информации в объекты (тест)
# endif

#----------------------------------------C
C_FLAGS += $(MCU) $(C_DEFS) $(ALL_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
ifeq ($(DEBUG), 1)
C_FLAGS += -g -gdwarf-2  #Добавление отладочной информации в объекты
endif

# Generate dependency information
#C_FLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
C_FLAGS += $(C_STANDARD)


# libraries
#LIBS = --specs=rdimon.specs #-specs=rdimon.specs  -lc -lm -lnosys
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections


listEcho=$(foreach var,$1,echo $(var);)


dbgMsg1=\
echo ------------------------------------------------- dbgMsg1 -------------------------------------------------;\
echo Work with makefile: $(MAKE_FILE_NAME);\
echo Target is: $(TARGET);\
echo "";

debug_echo_modules=\
echo ---------------------------debug_echo_modules----------------------------------;\
echo object files count: $(words $(OBJECTS_C));\
${call listEcho, $(OBJECTS_C)} \
echo ---------------------------debug_echo_modules END----------------------------------;\
echo ;

dbgEnd=\
echo ------------------------------------------------- END -------------------------------------------------;\
echo "";

# ----------------- Дополнительные команды make от Алексея Олеговича-----------------#
.PHONY: clean

#######################################
# flash
#######################################
# flash:
# 	openocd -f ./openocd.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"
flash:
	openocd -f ./stlink.cfg  -f ./stm32g4x.cfg  -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"


# *** EOF ***

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

debug_inc_paths:
	echo ---------------------------debug_inc_paths----------------------------------;\
	echo inc files count: $(words $(USER_MODULES_CPP));\
	echo source_files: $(USER_MODULES_CPP);\
	echo ---------------------------debug_inc_paths END----------------------------------;\
	echo ;

$(HEX_DIR):
	echo create path $@
	mkdir $@
