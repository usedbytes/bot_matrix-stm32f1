TARGET = main

SOURCES = main.c spi.c util.c systick.c pwm.c counter.c hbridge.c period_counter.c controller.c queue.c motor.c log.c vl53l0x.c i2c.c
#SOURCES += log_stdio.c
SOURCES += log_spi.c

VL53L0X_SOURCES = vl53l0x/core/src/vl53l0x_api.c \
	vl53l0x/core/src/vl53l0x_api_calibration.c \
	vl53l0x/core/src/vl53l0x_api_core.c \
	vl53l0x/core/src/vl53l0x_api_ranging.c \
	vl53l0x/core/src/vl53l0x_api_strings.c

VL53L0X_INC = -Ivl53l0x/core/inc -Ivl53l0x/platform/inc

SOURCES += $(VL53L0X_SOURCES)

LINKER_SCRIPT=stm32f103-bl20.ld

OPENCM3 ?= ./libopencm3

##############################################################################

OBJDIR = obj
OBJECTS = $(patsubst %.c,$(OBJDIR)/%.o,$(SOURCES))

###############################################################################

DEBUG = -g
OPT = -Os
FP_FLAGS ?= -msoft-float
ARCH_FLAGS = -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd
CFLAGS += $(DEBUG) $(OPT)
CFLAGS += $(ARCH_FLAGS)
CFLAGS += $(VL53L0X_INC)
CFLAGS += -Wall -Wextra -Wshadow -Wimplicit-function-declaration
CFLAGS += -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
CFLAGS += -fno-common -ffunction-sections -fdata-sections -fno-strict-aliasing

###############################################################################

LIBNAME = opencm3_stm32f1
DEFS += -DSTM32F1
DEFS += -I$(OPENCM3)/include
LDFLAGS += -L$(OPENCM3)/lib
LDFLAGS += --static -nostartfiles
LDFLAGS += -T$(LINKER_SCRIPT)
LDFLAGS += $(ARCH_FLAGS)
LDFLAGS += -Wl,-Map=$(*).map
LDFLAGS += -Wl,-Map=main.map
LDFLAGS += -Wl,--gc-sections
LDLIBS += -l$(LIBNAME)
LDLIBS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

CFLAGS += $(DEFS)

###############################################################################

CROSS = arm-none-eabi-
CC = $(CROSS)gcc
AS = $(CROSS)as
LD = $(CROSS)ld
OBJDUMP = $(CROSS)objdump
OBJCOPY = $(CROSS)objcopy
SIZE = $(CROSS)size

###############################################################################
.PHONY: all
all: $(TARGET).bin $(TARGET).elf $(TARGET).lss

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -R .stack -R .bss -O binary -S $(TARGET).elf $(TARGET).bin

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -R .stack -R .bss -O ihex $(TARGET).elf $(TARGET).hex

$(TARGET).elf: $(OBJECTS) $(LINKER_SCRIPT)
	$(CC) $(LDFLAGS) $(OBJECTS) $(LDLIBS) -o $(TARGET).elf

$(TARGET).lss: $(TARGET).elf
	$(OBJDUMP) -h -S $< > $@

$(OBJDIR)/%.o : %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: stats
stats: $(TARGET).elf
	$(OBJDUMP) -th $<
	$(SIZE) $<

.PHONY: clean
clean:
	rm -r $(OBJDIR)
	rm -f $(TARGET).elf
	rm -f $(TARGET).hex
	rm -f $(TARGET).bin
	rm -f $(TARGET).lss

.PHONY: flash
flash: $(TARGET).bin
	dfu-util -R -a 2 -D $<
