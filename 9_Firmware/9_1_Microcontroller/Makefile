# ============================================================================
# AERIS-10 STM32F746ZGT7 firmware build (PR 1: compile-only)
# ============================================================================
# Part:      STM32F746ZGT7 (Cortex-M7, 1 MB flash, 320 KB RAM, 216 MHz, LQFP144)
# Toolchain: arm-none-eabi-gcc 15.x (brew --cask gcc-arm-embedded)
#
# PR 1 scope: compile every source file to an object.  Linking is expected to
# fail because STM32CubeIDE-generated glue (main.c entry, SystemClock_Config,
# MX_*_Init, stm32f7xx_it.c, stm32f7xx_hal_msp.c, linker script, USB CDC
# glue) is missing.  PR 2 will hand-write those files and close the link.
#
# Targets:
#   make              -- compile all .c/.cpp/.s to objects (default)
#   make compile      -- same
#   make link         -- attempt to link (WILL FAIL in PR 1; left here so the
#                        missing-symbol list drives PR 2 scope)
#   make clean        -- remove build/
# ============================================================================

# ---- toolchain -------------------------------------------------------------
PREFIX  := arm-none-eabi-
CC      := $(PREFIX)gcc
CXX     := $(PREFIX)g++
AS      := $(PREFIX)gcc -x assembler-with-cpp
LD      := $(PREFIX)g++
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

# ---- tree ------------------------------------------------------------------
BUILD   := build
OBJDIR  := $(BUILD)/obj

VENDOR  := vendor
APP     := 9_1_3_C_Cpp_Code
LIB     := 9_1_1_C_Cpp_Libraries

# ---- target part + CPU flags ----------------------------------------------
MCU     := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard
DEFS    := -DSTM32F746xx -DUSE_HAL_DRIVER -DARM_MATH_CM7 -D__FPU_PRESENT=1

# ---- include paths (order matters; app overrides vendor on ambiguity) -----
INC := \
  -I$(APP) \
  -I$(LIB) \
  -I$(VENDOR)/cmsis_core/Include \
  -I$(VENDOR)/cmsis_device_f7/Include \
  -I$(VENDOR)/stm32f7xx_hal_driver/Inc \
  -I$(VENDOR)/stm32_mw_usb_device/Core/Inc \
  -I$(VENDOR)/stm32_mw_usb_device/Class/CDC/Inc

# ---- warnings / optimisation ----------------------------------------------
OPT      := -Og -g3
WARN     := -Wall -Wextra -Wno-unused-parameter -Wno-unused-function \
            -Wno-missing-field-initializers \
            -Wno-error=incompatible-pointer-types \
            -Wno-error=discarded-qualifiers \
            -Wno-error=int-conversion \
            -Wno-error=implicit-function-declaration
COMMON   := $(MCU) $(DEFS) $(INC) $(OPT) $(WARN) -ffunction-sections -fdata-sections -fno-common

CFLAGS   := $(COMMON) -std=gnu11
CXXFLAGS := $(COMMON) -std=gnu++17 -fno-exceptions -fno-rtti -fno-threadsafe-statics
ASFLAGS  := $(MCU) $(DEFS) -g3

# ---- sources ---------------------------------------------------------------
# Application C/C++
# main.cpp needs Cube-generated usb_device.h / usbd_cdc_if.h — PR 2 target.
# Exclude from PR 1 compile; um982_gps.c/.h compile cleanly and stay in.
APP_C       := $(wildcard $(APP)/*.c)
APP_CPP_ALL := $(wildcard $(APP)/*.cpp)
APP_CPP     := $(filter-out $(APP)/main.cpp, $(APP_CPP_ALL))
LIB_C   := $(wildcard $(LIB)/*.c)
# gps_handler.cpp includes usb_device.h (Cube glue) — deferred to PR 2
LIB_CPP := $(filter-out $(LIB)/gps_handler.cpp, $(wildcard $(LIB)/*.cpp))

# Vendor: all enabled HAL modules compile cleanly because each _hal_<x>.c is
# guarded by `#ifdef HAL_<X>_MODULE_ENABLED` in stm32f7xx_hal_conf.h.
# Build every .c in Src/ and let the guard do the filtering.
# Exclude *_template.c — ST ships these as user-copy stubs, not compile targets
HAL_C   := $(filter-out %_template.c, $(wildcard $(VENDOR)/stm32f7xx_hal_driver/Src/*.c))
# USB Device Library requires a user-written usbd_conf.h (Cube glue, PR 2).
# PR 1 leaves this empty so the vendor tree compiles without that config.
USB_C   :=
SYS_C   := $(VENDOR)/cmsis_device_f7/Source/Templates/system_stm32f7xx.c
STARTUP := $(VENDOR)/cmsis_device_f7/Source/Templates/gcc/startup_stm32f746xx.s

SRCS_C   := $(APP_C) $(LIB_C) $(HAL_C) $(USB_C) $(SYS_C)
SRCS_CPP := $(APP_CPP) $(LIB_CPP)
SRCS_S   := $(STARTUP)

OBJS := \
  $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS_C)) \
  $(patsubst %.cpp,$(OBJDIR)/%.o,$(SRCS_CPP)) \
  $(patsubst %.s,$(OBJDIR)/%.o,$(SRCS_S))

DEPS := $(OBJS:.o=.d)

# ---- link (PR 2) -----------------------------------------------------------
# Linker script is hand-written in PR 2 at linker/STM32F746ZGTx_FLASH.ld
LDSCRIPT := linker/STM32F746ZGTx_FLASH.ld
LDFLAGS  := $(MCU) -T$(LDSCRIPT) -Wl,--gc-sections -Wl,-Map=$(BUILD)/firmware.map \
            -specs=nano.specs -specs=nosys.specs -static

ELF := $(BUILD)/firmware.elf
BIN := $(BUILD)/firmware.bin
HEX := $(BUILD)/firmware.hex

# ---- rules -----------------------------------------------------------------
.PHONY: all compile link clean size

all: compile

compile: $(OBJS)
	@echo "compiled $(words $(OBJS)) objects"

link: $(ELF)

$(ELF): $(OBJS) $(LDSCRIPT)
	$(LD) $(LDFLAGS) $(OBJS) -o $@ -lm
	$(SIZE) $@
	$(OBJCOPY) -O binary $@ $(BIN)
	$(OBJCOPY) -O ihex   $@ $(HEX)

$(OBJDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(OBJDIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD)

-include $(DEPS)
