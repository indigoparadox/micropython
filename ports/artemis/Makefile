include ../../py/mkenv.mk

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h

# include py core make definitions
include $(TOP)/py/py.mk

CROSS_COMPILE = arm-none-eabi-

HAL_DIR = /home/michael/projects/AmbiqSuite-Rel2.2.0
CMSIS_DIR = /home/michael/projects/AmbiqSuite-Rel2.2.0/CMSIS

INC += -I.
INC += -I$(TOP)
INC += -I$(BUILD)
INC += -I$(HAL_DIR)/mcu/apollo3
INC += -I$(HAL_DIR)/mcu/apollo3/hal
INC += -I$(HAL_DIR)/mcu/appolo3/regs
INC += -I$(CMSIS_DIR)/AmbiqMicro/Include
INC += -I$(CMSIS_DIR)/ARM/Include

DFU = $(TOP)/tools/dfu.py
PYDFU = $(TOP)/tools/pydfu.py
CFLAGS_CORTEX_M4 = -mthumb -mtune=cortex-m4 -mcpu=cortex-m4 -msoft-float -fsingle-precision-constant -Wdouble-promotion
CFLAGS = $(INC) -Wall -Werror -std=c99 -nostdlib $(CFLAGS_CORTEX_M4) $(COPT)
LDFLAGS = -nostdlib -T artemis_sbl_svl_app.ld -Map=$@.map --cref --gc-sections

# Tune for Debugging or Optimization
ifeq ($(DEBUG), 1)
CFLAGS += -O0 -ggdb
else
CFLAGS += -Os -DNDEBUG
CFLAGS += -fdata-sections -ffunction-sections
endif

LIBS =

SRC_C = \
	main.c \
	uart_core.c \
	startup_gcc.c \
	lib/utils/printf.c \
	lib/utils/stdout_helpers.c \
	lib/utils/pyexec.c \
	lib/libc/string0.c \
	lib/mp-readline/readline.c \
	$(BUILD)/_frozen_mpy.c \

SRC_C += $(wildcard $(HAL_DIR)/mcu/apollo3/hal/*.c)

OBJ = $(PY_CORE_O) $(addprefix $(BUILD)/, $(SRC_C:.c=.o))

all: $(BUILD)/firmware.bin

$(BUILD)/_frozen_mpy.c: frozentest.mpy $(BUILD)/genhdr/qstrdefs.generated.h
	$(ECHO) "MISC freezing bytecode"
	$(Q)$(TOP)/tools/mpy-tool.py -f -q $(BUILD)/genhdr/qstrdefs.preprocessed.h -mlongint-impl=none $< > $@

$(BUILD)/firmware.elf: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LD) $(LDFLAGS) -o $@ $^ $(LIBS)
	$(Q)$(SIZE) $@

$(BUILD)/firmware.bin: $(BUILD)/firmware.elf
	$(Q)$(OBJCOPY) -O binary -j .isr_vector -j .text -j .data $^ $(BUILD)/firmware.bin

deploy: $(BUILD)/firmware.bin
	$(ECHO) "Writing $< to the board"
	~/.arduino15/packages/SparkFun/hardware/apollo3/1.0.2/tools/artemis/linux/artemis_svl -f $< /dev/ttyUSB0

include $(TOP)/py/mkrules.mk
