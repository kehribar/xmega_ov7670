###############################################################################
#
# Example makefile
#
###############################################################################

### Project name (also used for output file name)
PROJECT = main

### Source files and search directory
CSRC    = main.c xprintf.c bitbang_i2c.c
ASRC    =
VPATH   = 
DEVICE  = atxmega32e5
F_OSC   = 32000000
FLASH   = sudo avrdude -c avrispmkii -p x32e5 -U flash:w:./obj/main.hex
PORT 	= /dev/ttyUSB0
UPLOAD  = tealoader -p $(PORT) -f ./obj/main.hex -i

## Use the Atmel's latest GCC toolchain, not the Crosspack or any other version.
TOOLCHAIN_PATH = /home/kehribar/avr8-gnu-toolchain-linux_x86_64/bin/

###############################################################################
#
# Don't change anything below
#
###############################################################################

BOOTSTART = 0

### Include dirs, library dirs and definitions
LIBS	=
LIBDIRS	=
INCDIRS	=
DEFS	= F_CPU=$(F_OSC) BOOTSTART=$(BOOTSTART)
ADEFS	= F_CPU=$(F_OSC)

### Optimization level (0, 1, 2, 3, 4 or s)
OPTIMIZE = s

### C Standard level (c89, gnu89, c99 or gnu99)
CSTD = gnu89

### Warning contorls
WARNINGS = all extra

### Output directory
OBJDIR = obj

### Output file format (ihex, bin or both) and debugger type
OUTPUT	= ihex
DEBUG	= # dwarf-2

### Programs to build porject
CC      = $(TOOLCHAIN_PATH)avr-gcc
OBJCOPY = $(TOOLCHAIN_PATH)avr-objcopy
OBJDUMP = $(TOOLCHAIN_PATH)avr-objdump
SIZE    = $(TOOLCHAIN_PATH)avr-size
NM      = $(TOOLCHAIN_PATH)avr-nm

# Define all object files
COBJ      = $(CSRC:.c=.o) 
AOBJ      = $(ASRC:.S=.o)
COBJ      := $(addprefix $(OBJDIR)/,$(COBJ))
AOBJ      := $(addprefix $(OBJDIR)/,$(AOBJ))
PROJECT   := $(OBJDIR)/$(PROJECT)

# Flags for C files
CFLAGS += -Wno-deprecated-declarations -D__PROG_TYPES_COMPAT__
CFLAGS += -fdata-sections -ffunction-sections 

# http://sourceforge.net/apps/mediawiki/mobilechessboar/index.php?title=Avr-gcc 
# CFLAGS += -mstrict-X -fno-inline-small-functions -fno-move-loop-invariants -fno-optimize-sibling-calls -maccumulate-args
# CFLAGS += -fno-split-wide-types -fno-tree-loop-optimize -fno-tree-switch-conversion -maccumulate-args
# CFLAGS += -fno-dse -fno-caller-saves -fno-tree-ter

CFLAGS += -std=$(CSTD)
CFLAGS += -g$(DEBUG)
CFLAGS += -mmcu=$(DEVICE)
CFLAGS += -O$(OPTIMIZE) -mcall-prologues
CFLAGS += $(addprefix -W,$(WARNINGS))
CFLAGS += $(addprefix -I,$(INCDIRS))
CFLAGS += $(addprefix -D,$(DEFS))
CFLAGS += -Wp,-M,-MP,-MT,$(OBJDIR)/$(*F).o,-MF,$(OBJDIR)/$(*F).d

# Assembler flags
ASFLAGS += $(addprefix -D,$(ADEFS)) -Wa,-gstabs,-g$(DEBUG)
ALL_ASFLAGS = -mmcu=$(DEVICE) -I. -x assembler-with-cpp $(ASFLAGS) -fdata-sections -ffunction-sections

# Linker flags
CFLAGS	 += -Wl,--section-start=.text=$(BOOTSTART) 
CFLAGS 	 += -Wl,-lm -Wl,--gc-sections,-Map,$(PROJECT).map

# Default target.
all: version build size

ifeq ($(OUTPUT),ihex)
build: elf hex lst sym
hex: $(PROJECT).hex
else
ifeq ($(OUTPUT),binary)
build: elf bin lst sym
bin: $(PROJECT).bin
else
ifeq ($(OUTPUT),both)
build: elf hex bin lst sym
hex: $(PROJECT).hex
bin: $(PROJECT).bin
else
$(error "Invalid format: $(OUTPUT)")
endif
endif
endif

elf: $(PROJECT).elf
lst: $(PROJECT).lst 
sym: $(PROJECT).sym


# Display compiler version information.
version :
	@$(CC) --version

# Create final output file (.hex or .bin) from ELF output file.
%.hex: %.elf
	@echo
	$(OBJCOPY) -O ihex $< $@

%.bin: %.elf
	@echo
	$(OBJCOPY) -O binary $< $@

# Create extended listing file from ELF output file.
%.lst: %.elf
	@echo
	$(OBJDUMP) -h -S -C $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	$(NM) -n $< > $@

# Display size of file.
size:
	@echo
	$(SIZE) -C --mcu=$(DEVICE) $(PROJECT).elf

# Link: create ELF output file from object files.
%.elf:  $(AOBJ) $(COBJ)
	@echo
	@echo Linking...
	$(CC) $(CFLAGS) $(AOBJ) $(COBJ) --output $@

# Compile: create object files from C source files. ARM or Thumb(-2)
$(COBJ) : $(OBJDIR)/%.o : %.c
	@echo
	@echo $< :
	$(CC) -c $(CFLAGS) $< -o $@

# Assemble: create object files from assembler source files. ARM or Thumb(-2)
$(AOBJ) : $(OBJDIR)/%.o : %.S
	@echo
	@echo $< :
	$(CC) -c $(ALL_ASFLAGS) $< -o $@

# Target: clean project.
clean:
	@echo
	rm -f -r $(OBJDIR) | exit 0

# Include the dependency files.
-include $(shell mkdir $(OBJDIR) 2>/dev/null) $(wildcard $(OBJDIR)/*.d)

flash:
	$(FLASH)

upload:
	$(UPLOAD)

iterate:
	make clean
	make 
	make upload
