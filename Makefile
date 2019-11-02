# Target file name (without extension).
TARGET = led_timer

# MCU name
MCU = attiny85
F_CPU = 32768
PORT = /dev/tty.usbserial-A907OGFE
LOGFILE = $(TARGET).log
BAUD_RATE = 115200

FUSE_L = 0xE2
FUSE_H = 0xDF
FUSE_E = 0xFF

# Output format. (can be srec, ihex, binary)
FORMAT = ihex 

# Optimization level, can be [0, 1, 2, 3, s]. 0 turns off optimization.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s

# List C source files here. (C dependencies are automatically generated.)

SRC = \
	  wiper_main.c

CXXSRC = 

# List Assembler source files here.
# Make them always end in a capital .S.  Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC = 


# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
EXTRAINCDIRS = 


# Optional compiler flags.
#  -g:        generate debugging information (for GDB, or for COFF conversion)
#  -O*:       optimization level
#  -f...:     tuning, see gcc manual and avr-libc documentation
#  -Wall...:  warning level
#  -Wa,...:   tell GCC to pass this to the assembler.
#    -ahlms:  create assembler listing
CFLAGS = -g -O$(OPT) \
-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums \
-Wall -Wstrict-prototypes \
-DF_CPU=$(F_CPU) \
-Wa,-adhlns=$(subst $(suffix $<),.lst,obj/$<)  \
$(patsubst %,-I%,$(EXTRAINCDIRS)) 


# Set a "language standard" compiler flag.
#   Unremark just one line below to set the language standard to use.
#   gnu99 = C99 + GNU extensions. See GCC manual for more information.
#CFLAGS += -std=c89
#CFLAGS += -std=gnu89
#CFLAGS += -std=c99
CFLAGS += -std=gnu99



# Optional assembler flags.
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -ahlms:    create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
ASFLAGS = -Wa,-adhlns=$(subst $(suffix $<),.lst,obj/$<),-gstabs 



# Optional linker flags.
#  -Wl,...:   tell GCC to pass this to linker.
#  -Map:      create map file
#  --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=obj/$(TARGET).map,--cref



# Additional libraries

# Minimalistic printf version
#LDFLAGS += -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires -lm below)
#LDFLAGS += -Wl,-u,vfprintf -lprintf_flt

# -lm = math library
#LDFLAGS += -lm



# Programming support using avrdude. Settings and variables.

# Programming hardware: alf avr910 avrisp bascom bsd 
# dt006 pavr picoweb pony-stk200 sp12 stk200 stk500
#
# Type: avrdude -c ?
# to get a full listing.
#
AVRDUDE_PROGRAMMER = avrispmkII


AVRDUDE_PORT = usb # programmer connected to serial device
AVRDUDE_SERIAL_PORT = /dev/arduino2

AVRDUDE_WRITE_FLASH = -U flash:w:obj/$(TARGET).hex
AVRDUDE_WRITE_EEPROM = -U eeprom:w:obj/$(TARGET).eep

AVRDUDE_FLAGS = -p$(MCU) -P$(AVRDUDE_PORT) -c$(AVRDUDE_PROGRAMMER) -V

# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE += -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
#AVRDUDE_FLAGS += -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude> 
# to submit bug reports.
AVRDUDE_FLAGS += -v 

AVRDUDE_FUSE_FLAGS = $(AVRDUDE_FLAGS) -B500
AVRDUDE_FLAGS += -B300


# ---------------------------------------------------------------------------

# Define directories, if needed.
DIRAVR = c:/winavr
DIRAVRBIN = $(DIRAVR)/bin
DIRAVRUTILS = $(DIRAVR)/utils/bin
DIRINC = .
DIRLIB = $(DIRAVR)/avr/lib


# Define programs and commands.
#
AVRPATH = /usr/local/bin
SHELL = sh

CC = $(AVRPATH)/avr-gcc
CXX = $(AVRPATH)/avr-g++

OBJCOPY = $(AVRPATH)/avr-objcopy
OBJDUMP = $(AVRPATH)/avr-objdump
SIZE = $(AVRPATH)/avr-size


# Programming support using avrdude.
AVRDUDE = avrdude


REMOVE = rm -f
COPY = cp

HEXSIZE = $(SIZE) --target=$(FORMAT) obj/$(TARGET).hex
ELFSIZE = $(SIZE) -A obj/$(TARGET).elf


# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_COFF = Converting to AVR COFF:
MSG_EXTENDED_COFF = Converting to AVR Extended COFF:
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:

INCLUDE = -I. $(EXTRAINCDIRS)

# Define all object files.
OBJ = $(SRC:%.c=obj/%.o) $(ASRC:%.S=obj/%.o) $(CXXSRC:%.cpp=obj/%.o)

# Define all listing files.
LST = $(ASRC:%.S=obj/%.lst) $(SRC:%.c=obj/%.lst) $(CXXSRC:%.cpp=obj/%.lst)

ALL = $(OBJ) $(LST)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) $(INCLUDE) $(CFLAGS)  -lm
ALL_ASFLAGS = -mmcu=$(MCU) $(INCLUDE) -x assembler-with-cpp $(ASFLAGS)  -lm



# Default target.
all: begin gccversion sizebefore obj/$(TARGET).elf obj/$(TARGET).hex obj/$(TARGET).eep \
	obj/$(TARGET).lss obj/$(TARGET).sym sizeafter finished end

full: obj/$(TARGET).hex obj/$(TARGET).eep eepromburn
	$(AVRDUDE) $(AVRDUDE_FLAGS) -Uflash:w:$<

eepromburn: $(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) -Ueeprom:w:$<

burn-fuses: 
	$(AVRDUDE) $(AVRDUDE_FUSE_FLAGS) -u -U lfuse:w:$(FUSE_L):m -U hfuse:w:$(FUSE_H):m -U efuse:w:$(FUSE_E):m

reset:
	$(AVRDUDE) $(AVRDUDE_FLAGS) 

# Eye candy.
# AVR Studio 3.x does not check make's exit code but relies on
# the following magic strings to be generated by the compile job.
begin:
	@echo
	@echo $(MSG_BEGIN)

finished:
	@echo $(MSG_ERRORS_NONE); \
		$(HEXSIZE)

end:
	@echo $(MSG_END)
	@echo


# Display size of file.
sizebefore:
	@if [ -f $(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); echo; fi

sizeafter:
	@if [ -f $(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi



# Display compiler version information.
gccversion : 
	@$(CC) --version



# Program the device.  
program: obj/$(TARGET).hex obj/$(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS)  -Uflash:w:$< 


# Create final output files (.hex, .eep) from ELF output file.
obj/%.hex: obj/%.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

obj/%.eep: obj/%.elf
	@echo
	@echo $(MSG_EEPROM) $@
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
obj/%.lss: obj/%.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
obj/%.sym: obj/%.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	avr-nm -n $< > $@


# Link: create ELF output file from object files.
.SECONDARY : obj/$(TARGET).elf
.PRECIOUS : $(OBJ)
obj/%.elf: $(OBJ)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(ALL_CFLAGS) $(OBJ) --output $@ $(LDFLAGS) 

# Compile: create object files from C++ source files.
obj/%.o : %.cpp
	$(CXX) -c $(ALL_CFLAGS) $< -o $@ 


# Compile: create object files from C source files.
obj/%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(ALL_CFLAGS) -DTXMODE=$(TXMODE)   $< -o $@ 


# Compile: create assembler files from C source files.
obj/%.s : %.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@ 


# Assemble: create object files from assembler source files.
obj/%.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@ 

# Target: clean project.
clean: begin clean_list end

debug: 
	./readserial -B$(BAUD_RATE) -l$(LOGFILE) $(PORT)

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) -v $(ALL) obj/$(TARGET).hex obj/$(TARGET).eep obj/$(SRC:.c=.s) obj/$(SRC:.c=.d) obj/$(TARGET).elf obj/$(TARGET).lss obj/$(TARGET).sym obj/$(TARGET).map

# Automatically generate C source code dependencies. 
# (Code originally taken from the GNU make user manual and modified 
# (See README.txt Credits).)
#
# Note that this will work with sh (bash) and sed that is shipped with WinAVR
# (see the SHELL variable defined above).
# This may not work with other shells or other seds.

obj/%.d: %.c
	set -e; $(CC) -MM $(ALL_CFLAGS) $< \
	| sed 's,\(.*\)\.o[ :]*,\1.o \1.d : ,g' > $@; \
	[ -s $@ ] || rm -f $@


# Remove the '-' if you want to see the dependency files generated.
#include $(SRC:.c=.d)

# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion coff extcoff \
	clean clean_list program
