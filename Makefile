#
# Customized for the Atmega328p and OrangeRX (XMEGA) multi 4-in-1 boards.
# Both are AVR boards but need different compiler and upload flags and parameters.
#
# For more info:
# https://github.com/arduino/Arduino/wiki/Arduino-IDE-1.5---3rd-party-Hardware-specification

name=Multi 4-in-1 AVR
version=1.0.4

compiler.warning_flags=-w
compiler.warning_flags.none=-w
compiler.warning_flags.default=
compiler.warning_flags.more=-Wall
compiler.warning_flags.all=-Wall -Wextra

# Default "compiler.path" is correct, change only if you want to override the initial value
compiler.path={runtime.tools.avr-gcc.path}/bin/
compiler.c.cmd=avr-gcc
compiler.c.flags=-c -g -Os ${compiler.warning_flags.all} -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto
compiler.c.elf.flags=${compiler.warning_flags.all} ${board.compiler.c.elf.flags}
compiler.c.elf.cmd=avr-gcc
compiler.S.flags={board.compiler.S.flags}
compiler.cpp.cmd=avr-g++
compiler.cpp.flags=-c -g -Os ${compiler.warning_flags.all} -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto
compiler.cpp.extra_flags=
compiler.ar.cmd=avr-gcc-ar
compiler.ar.flags=rcs
compiler.objcopy.cmd=avr-objcopy
compiler.objcopy.eep.flags=-O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0
compiler.elf2hex.flags=-O ihex -R .eeprom
compiler.elf2hex.cmd=avr-objcopy
compiler.ldflags=
compiler.size.cmd=avr-size

## Create archives
# archive_file_path is needed for backwards compatibility with IDE 1.6.5 or older, IDE 1.6.6 or newer overrides this value
compiler.ar.cmd=avr-gcc-ar
compiler.ar.flags=rcs
compiler.ar.extra_flags=
archive_file_name=${build.mcu}.a
archive_file_path=${build.path}/${archive_file_name}

compiler.c.elf.cmd=avr-gcc
board.compiler.c.elf.flags=-Os -flto -Wl,--gc-sections
compiler.c.elf.flags=${compiler.warning_flags.all} ${board.compiler.c.elf.flags}
compiler.c.elf.extra_flags=

TARGET := multi-orx



# various programs
#CC = "$(ARDUINO_DIR)hardware/tools/avr/bin/avr-gcc"
#CPP = "$(ARDUINO_DIR)hardware/tools/avr/bin/avr-g++"
#AR = "$(ARDUINO_DIR)hardware/tools/avr/bin/avr-ar"
#OBJ_COPY = "$(ARDUINO_DIR)hardware/tools/avr/bin/avr-objcopy"

SRC_DIR := src
BUILD_DIR := build
build.path := build
CORE_DIR := package_multi_4in1_avr_board_v1.0.4/cores/xmega
CORE_SRC = $(wildcard $(CORE_DIR)/*.c) $(wildcard $(CORE_DIR)/*.cpp) $(wildcard $(CORE_DIR)/avr-libc/*.c) $(wildcard $(CORE_DIR)/avr-libc/*.cpp)
CORE_OBJ = $(patsubst $(CORE_DIR)/%,$(BUILD_DIR)/core/%.o, $(CORE_SRC))

## Location of include files

INC_CORE_DIRS = -Ipackage_multi_4in1_avr_board_v1.0.4/cores/xmega
INC_CORE_DIRS += -Ipackage_multi_4in1_avr_board_v1.0.4/variants/xmega32d4
INCLUDE_DIRS =-I$(SRC_DIR) $(INC_CORE_DIRS)

INCLUDE_FILES = needed.h


build.f_cpu=32000000L
build.mcu=atxmega32e5
build.board=MULTI_ORANGERX=104

multixmega32d4.name=Multi 4-in-1 (OrangeRX)
multixmega32d4.build.board=MULTI_ORANGERX=104
multixmega32d4.build.mcu=atxmega32d4
multixmega32d4.build.f_cpu=32000000L
multixmega32d4.build.core=xmega
multixmega32d4.build.variant=xmega32d4

runtime.ide.version=10808
build.board=MULTI_ORANGERX=104
build.arch=AVR

DEFINES = -DARDUINO=${runtime.ide.version} -DARDUINO_${build.board} -DARDUINO_ARCH_${build.arch}


SOURCES := $(wildcard $(SRC_DIR)/*.ino)
# Contruct preferred sequence of files to handle header files and reduce dependency on function prototypes.
NEW_ORDER = $(SRC_DIR)/Multiprotocol.ino
NEW_ORDER +=$(SRC_DIR)/SPI.ino
NEW_ORDER += $(SRC_DIR)/A7105_SPI.ino
NEW_ORDER +=$(SRC_DIR)/CC2500_SPI.ino
NEW_ORDER +=$(SRC_DIR)/CYRF6936_SPI.ino
NEW_ORDER +=$(SRC_DIR)/NRF24l01_SPI.ino

MINUS = $(filter-out $(NEW_ORDER), $(SOURCES))

# Default target.
all: begin gccversion $(BUILD_DIR)/Multiprotocol.all fprotogen compile corelib link end


# Concatenate all sources files in one big file.
$(BUILD_DIR)/Multiprotocol.all: $(SOURCES)
	@echo Generating $@
	@echo '' > $@
	@for file in $(NEW_ORDER); \
	  do (echo ''; echo "# 1 \"$$file\" // Helps debugging !"; cat $$file; echo '') >> $@; \
	done 
	@for file in $(MINUS); \
	  do (echo ''; echo "# 1 \"$$file\" // Helps debugging !"; cat $$file; echo '') >> $@; \
	done


preproc.cpp.flags=-c -g -Os ${compiler.warning_flags.none} -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD -flto
preproc.macros.flags=-x c++ -E -CC
.PHONY: fprotogen
fprotogen: $(BUILD_DIR)/Multiprotocol.all  $(BUILD_DIR)/fprotos.h

$(BUILD_DIR)/fprotos.h: ## $(BUILD_DIR)/Multiprotocol.all
	@echo Making $@
	${compiler.cpp.cmd} -c -g -Os -x c++ -w  -std=gnu++11  -E -P  -mmcu=$(build.mcu) -DF_CPU=$(build.f_cpu) -DORANGE_TX -DARDUINO_AVR_XMEGA32D4  $(INCLUDE_DIRS)  $(BUILD_DIR)/Multiprotocol.all  -o $(BUILD_DIR)/Multiprotocol.pre
# ${compiler.cpp.cmd} ${preproc.cpp.flags} ${preproc.macros.flags} -mmcu=${build.mcu} -DF_CPU=${build.f_cpu} ${DEFINES} ${INCLUDE_DIRS}  $@  -o $@.pre 

#	./ctags -u --language-force=c++ -f - --c++-kinds=svpf --fields=KSTtzns --line-directives  $@.pre2 > tags.txt

# Generate function prototypes and variable names.
#### use cproto to feed compiler
#	./cproto.exe -ceisv -E0 -f2 $(BUILD_DIR)/Multiprotocol.pre > $(BUILD_DIR)/fprotos.h


################################
## Compile Multiprotocol.ino.cpp
.PHONY: compile
compile: $(BUILD_DIR)/Multiprotocol.ino.cpp

$(BUILD_DIR)/Multiprotocol.ino.cpp: $(BUILD_DIR)/Multiprotocol.all
	@echo Compiling sketch...
	cp $(BUILD_DIR)/Multiprotocol.all $(BUILD_DIR)/Multiprotocol.ino.cpp
#	$(CC) -mmcu=$(build.mcu) -DF_CPU=$(build.f_cpu) -c   -g -Os -Wall -DORANGE_TX -DARDUINO_AVR_XMEGA32D4  $(INCLUDE_DIRS) -include $(INCLUDE_FILES) -ffunction-sections -fdata-sections  -flto  allsrc.cpp  -o allsrc.o
	${compiler.cpp.cmd} ${compiler.cpp.flags} -mmcu=${build.mcu} -DF_CPU=${build.f_cpu} ${DEFINES} ${INCLUDE_DIRS} -include $(INCLUDE_FILES) $(BUILD_DIR)/Multiprotocol.ino.cpp  -o $(BUILD_DIR)/Multiprotocol.ino.cpp.o
#-flto enable the link time optimiser.

##########
## Corelib
.PHONY: corelib
corelib: ${archive_file_path}

${archive_file_path}: ${CORE_OBJ}
	@echo Adding to archive $@
	${compiler.ar.cmd} ${compiler.ar.flags} ${compiler.ar.extra_flags} ${archive_file_path}  $?

$(BUILD_DIR)/core/%.c.o: $(CORE_DIR)/%.c
	@echo Compiling archive $@
	${compiler.c.cmd} ${compiler.c.flags} -mmcu=${build.mcu} -DF_CPU=${build.f_cpu}  ${DEFINES} ${INC_CORE_DIRS}  $<  -o $@

$(BUILD_DIR)/core/%.cpp.o: $(CORE_DIR)/%.cpp
	@echo Compiling archive $@
	${compiler.cpp.cmd} ${compiler.cpp.flags} -mmcu=${build.mcu} -DF_CPU=${build.f_cpu}  ${DEFINES} ${INC_CORE_DIRS}  $<  -o $@


#######
## Link
.PHONY: link
link: $(TARGET).elf

$(TARGET).elf: $(BUILD_DIR)/Multiprotocol.ino.cpp.o  ${archive_file_path}
	@echo Linking everything together...
	${compiler.c.elf.cmd} ${compiler.c.elf.flags} -mmcu=${build.mcu} ${compiler.c.elf.extra_flags} -o $@  $^ -Lhome/sloped-soarer/arduino-1.8.8/hardware/tools/avr/avr/lib/avrxmega2  -lm
	
.PHONY: hex eep lss size

hex: $(TARGET).elf
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $<  $(TARGET).hex

eep: $(TARGET).elf
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex  $<  $(TARGET).eep || exit 0

lss: $(TARGET).elf
	avr-objdump -h -S  $<  > $(TARGET).lss

size: $(TARGET).elf
	avr-size -d -t $(TARGET).elf
	avr-size -A $(TARGET).elf


.PHONY: gccversion begin end clean

# Display compiler version information.
gccversion:
	@ make -v
	@$(CPP) --version

begin:
	@echo
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)
	@echo	

clean:
	-rm -f ${archive_file_path} 
	-rm -f $(BUILD_DIR)/Multiprotocol.ino.cpp
