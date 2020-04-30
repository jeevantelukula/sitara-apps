#
#  Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  *  Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#  *  Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#  *  Neither the name of Texas Instruments Incorporated nor the names of
#     its contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
#  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#
#  ======== Makefile ========
#

EXBASE = ./
include $(EXBASE)/products.mak

.PHONY: all debug release

PROC_VER = 7R5
VFP_VER = vfpv3d16
OBJEXT = obj

# Source directories
SRC_DIR_BASE                = Source
SRC_DIR_BASIC_MATH          = $(SRC_DIR_BASE)/BasicMathFunctions
SRC_DIR_COMMON_TABLES       = $(SRC_DIR_BASE)/CommonTables
SRC_DIR_COMPLEX_MATH        = $(SRC_DIR_BASE)/ComplexMathFunctions
SRC_DIR_CONTROLLER          = $(SRC_DIR_BASE)/ControllerFunctions
SRC_DIR_FAST_MATH           = $(SRC_DIR_BASE)/FastMathFunctions
SRC_DIR_FILTERING           = $(SRC_DIR_BASE)/FilteringFunctions
SRC_DIR_MATRIX              = $(SRC_DIR_BASE)/MatrixFunctions
SRC_DIR_STATISTICS          = $(SRC_DIR_BASE)/StatisticsFunctions
SRC_DIR_SUPPORT             = $(SRC_DIR_BASE)/SupportFunctions
SRC_DIR_TRANSFORM           = $(SRC_DIR_BASE)/TransformFunctions

# Source files
ifeq ($(CMSIS_VER),CMSIS_05_04_00)
SRCS_BASIC_MATH             = $(wildcard $(SRC_DIR_BASIC_MATH)/*.c)   # CMSIS 5.4.0
SRCS_COMMON_TABLES          = $(wildcard $(SRC_DIR_COMMON_TABLES)/*.c) 
SRCS_COMPLEX_MATH           = $(wildcard $(SRC_DIR_COMPLEX_MATH)/*.c)
SRCS_CONTROLLER             = $(wildcard $(SRC_DIR_CONTROLLER)/*.c) 
SRCS_FAST_MATH              = $(wildcard $(SRC_DIR_FAST_MATH)/*.c)
SRCS_FILTERING              = $(wildcard $(SRC_DIR_FILTERING)/*.c)
SRCS_MATRIX                 = $(wildcard $(SRC_DIR_MATRIX)/*.c)
SRCS_STATISTICS             = $(wildcard $(SRC_DIR_STATISTICS)/*.c)
SRCS_SUPPORT                = $(wildcard $(SRC_DIR_SUPPORT)/*.c)
SRCS_TRANSFORM              = $(wildcard $(SRC_DIR_TRANSFORM)/*.c)
ASM_SRCS_TRANSFORM			= $(wildcard $(SRC_DIR_TRANSFORM)/*.S)
else
SRCS_BASIC_MATH             = $(wildcard $(SRC_DIR_BASIC_MATH)/arm*.c) # CMSIS 5.6.0
SRCS_COMMON_TABLES          = $(wildcard $(SRC_DIR_COMMON_TABLES)/arm*.c)
SRCS_COMPLEX_MATH           = $(wildcard $(SRC_DIR_COMPLEX_MATH)/arm*.c)
SRCS_CONTROLLER             = $(wildcard $(SRC_DIR_CONTROLLER)/arm*.c)
SRCS_FAST_MATH              = $(wildcard $(SRC_DIR_FAST_MATH)/arm*.c)
SRCS_FILTERING              = $(wildcard $(SRC_DIR_FILTERING)/arm*.c)
SRCS_MATRIX                 = $(wildcard $(SRC_DIR_MATRIX)/arm*.c)
SRCS_STATISTICS             = $(wildcard $(SRC_DIR_STATISTICS)/arm*.c)
SRCS_SUPPORT                = $(wildcard $(SRC_DIR_SUPPORT)/arm*.c)
SRCS_TRANSFORM              = $(wildcard $(SRC_DIR_TRANSFORM)/arm*.c)
ASM_SRCS_TRANSFORM			= $(wildcard $(SRC_DIR_TRANSFORM)/arm*.S)
endif

# Object file base directory
OBJ_DIR_BASE = bin/$(PROFILE)/obj

# Object & dependency file directories
OBJ_DIR_BASIC_MATH          = $(addsuffix /$(subst $(SRC_DIR_BASE)/,,$(SRC_DIR_BASIC_MATH)),$(OBJ_DIR_BASE))
OBJ_DIR_COMMON_TABLES       = $(addsuffix /$(subst $(SRC_DIR_BASE)/,,$(SRC_DIR_COMMON_TABLES)),$(OBJ_DIR_BASE))
OBJ_DIR_COMPLEX_MATH        = $(addsuffix /$(subst $(SRC_DIR_BASE)/,,$(SRC_DIR_COMPLEX_MATH)),$(OBJ_DIR_BASE))
OBJ_DIR_CONTROLLER          = $(addsuffix /$(subst $(SRC_DIR_BASE)/,,$(SRC_DIR_CONTROLLER)),$(OBJ_DIR_BASE))
OBJ_DIR_FAST_MATH           = $(addsuffix /$(subst $(SRC_DIR_BASE)/,,$(SRC_DIR_FAST_MATH)),$(OBJ_DIR_BASE))
OBJ_DIR_FILTERING           = $(addsuffix /$(subst $(SRC_DIR_BASE)/,,$(SRC_DIR_FILTERING)),$(OBJ_DIR_BASE))
OBJ_DIR_MATRIX              = $(addsuffix /$(subst $(SRC_DIR_BASE)/,,$(SRC_DIR_MATRIX)),$(OBJ_DIR_BASE))
OBJ_DIR_STATISTICS          = $(addsuffix /$(subst $(SRC_DIR_BASE)/,,$(SRC_DIR_STATISTICS)),$(OBJ_DIR_BASE))
OBJ_DIR_SUPPORT             = $(addsuffix /$(subst $(SRC_DIR_BASE)/,,$(SRC_DIR_SUPPORT)),$(OBJ_DIR_BASE))
OBJ_DIR_TRANSFORM           = $(addsuffix /$(subst $(SRC_DIR_BASE)/,,$(SRC_DIR_TRANSFORM)),$(OBJ_DIR_BASE))
OBJ_DIRS = $(OBJ_DIR_BASIC_MATH) \
    $(OBJ_DIR_COMMON_TABLES) \
    $(OBJ_DIR_COMPLEX_MATH) \
    $(OBJ_DIR_CONTROLLER) \
    $(OBJ_DIR_FAST_MATH) \
    $(OBJ_DIR_FILTERING) \
    $(OBJ_DIR_MATRIX) \
    $(OBJ_DIR_STATISTICS) \
    $(OBJ_DIR_SUPPORT) \
    $(OBJ_DIR_TRANSFORM)

# Object files
OBJS_BASIC_MATH         = $(addprefix $(OBJ_DIR_BASIC_MATH)/,$(patsubst %.c,%.$(OBJEXT),$(notdir $(SRCS_BASIC_MATH))))
OBJS_COMMON_TABLES      = $(addprefix $(OBJ_DIR_COMMON_TABLES)/,$(patsubst %.c,%.$(OBJEXT),$(notdir $(SRCS_COMMON_TABLES))))
OBJS_COMPLEX_MATH       = $(addprefix $(OBJ_DIR_COMPLEX_MATH)/,$(patsubst %.c,%.$(OBJEXT),$(notdir $(SRCS_COMPLEX_MATH))))
OBJS_CONTROLLER         = $(addprefix $(OBJ_DIR_CONTROLLER)/,$(patsubst %.c,%.$(OBJEXT),$(notdir $(SRCS_CONTROLLER))))
OBJS_FAST_MATH          = $(addprefix $(OBJ_DIR_FAST_MATH)/,$(patsubst %.c,%.$(OBJEXT),$(notdir $(SRCS_FAST_MATH))))
OBJS_FILTERING          = $(addprefix $(OBJ_DIR_FILTERING)/,$(patsubst %.c,%.$(OBJEXT),$(notdir $(SRCS_FILTERING))))
OBJS_MATRIX             = $(addprefix $(OBJ_DIR_MATRIX)/,$(patsubst %.c,%.$(OBJEXT),$(notdir $(SRCS_MATRIX))))
OBJS_STATISTICS         = $(addprefix $(OBJ_DIR_STATISTICS)/,$(patsubst %.c,%.$(OBJEXT),$(notdir $(SRCS_STATISTICS))))
OBJS_SUPPORT            = $(addprefix $(OBJ_DIR_SUPPORT)/,$(patsubst %.c,%.$(OBJEXT),$(notdir $(SRCS_SUPPORT))))
OBJS_TRANSFORM          = $(addprefix $(OBJ_DIR_TRANSFORM)/,$(patsubst %.c,%.$(OBJEXT),$(notdir $(SRCS_TRANSFORM))))
ASM_OBJS_TRANSFORM	= $(addprefix $(OBJ_DIR_TRANSFORM)/,$(patsubst %.S,%.$(OBJEXT),$(notdir $(ASM_SRCS_TRANSFORM))))
OBJS = $(OBJS_BASIC_MATH) \
    $(OBJS_COMMON_TABLES) \
    $(OBJS_COMPLEX_MATH) \
    $(OBJS_CONTROLLER) \
    $(OBJS_FAST_MATH) \
    $(OBJS_FILTERING) \
    $(OBJS_MATRIX) \
    $(OBJS_STATISTICS) \
    $(OBJS_SUPPORT) \
    $(OBJS_TRANSFORM)
OBJS += $(ASM_OBJS_TRANSFORM)

# Include generated dependency files
-include $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_BASIC_MATH))
-include $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_COMMON_TABLES))
-include $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_COMPLEX_MATH))
-include $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_CONTROLLER))
-include $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_FAST_MATH))
-include $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_FILTERING))
-include $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_MATRIX))
-include $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_STATISTICS))
-include $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_SUPPORT))
-include $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_TRANSFORM))

.PRECIOUS: %/compiler.opt %/linker.cmd

all: release
  
    
#  ======== rule for debug build configuration ========
debug:
	$(MAKE) PROFILE=debug ti_math_Cortex_R5_lspf.x

#  ======== rule for release build configuration ========
release:
	$(MAKE) PROFILE=release ti_math_Cortex_R5_lspf.x

    
#  ======== rule for archiver ========
ti_math_Cortex_R5_lspf.x: bin/$(PROFILE)/ti_math_Cortex_R5_lspf.lib
bin/$(PROFILE)/ti_math_Cortex_R5_lspf.lib: $(OBJS)
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(AR) $(ARFLAGS) $@ $^
    
#
#  ======== rules for compilations of source files ========
#
$(OBJ_DIR_BASIC_MATH)/%.$(OBJEXT): $(SRC_DIR_BASIC_MATH)/%.c | $(OBJ_DIR_BASIC_MATH)
	@$(ECHO) "#"
	@$(ECHO) "# Making $< $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fc $<

$(OBJ_DIR_COMMON_TABLES)/%.$(OBJEXT): $(SRC_DIR_COMMON_TABLES)/%.c | $(OBJ_DIR_COMMON_TABLES)
	@$(ECHO) "#"
	@$(ECHO) "# Making $< $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fc $<
    
$(OBJ_DIR_COMPLEX_MATH)/%.$(OBJEXT): $(SRC_DIR_COMPLEX_MATH)/%.c | $(OBJ_DIR_COMPLEX_MATH)
	@$(ECHO) "#"
	@$(ECHO) "# Making $< $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fc $<

$(OBJ_DIR_CONTROLLER)/%.$(OBJEXT): $(SRC_DIR_CONTROLLER)/%.c | $(OBJ_DIR_CONTROLLER)
	@$(ECHO) "#"
	@$(ECHO) "# Making $< $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fc $<

$(OBJ_DIR_FAST_MATH)/%.$(OBJEXT): $(SRC_DIR_FAST_MATH)/%.c | $(OBJ_DIR_FAST_MATH)
	@$(ECHO) "#"
	@$(ECHO) "# Making $< $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fc $<

$(OBJ_DIR_FILTERING)/%.$(OBJEXT): $(SRC_DIR_FILTERING)/%.c | $(OBJ_DIR_FILTERING)
	@$(ECHO) "#"
	@$(ECHO) "# Making $< $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fc $<

$(OBJ_DIR_MATRIX)/%.$(OBJEXT): $(SRC_DIR_MATRIX)/%.c | $(OBJ_DIR_MATRIX)
	@$(ECHO) "#"
	@$(ECHO) "# Making $< $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fc $<

$(OBJ_DIR_STATISTICS)/%.$(OBJEXT): $(SRC_DIR_STATISTICS)/%.c | $(OBJ_DIR_STATISTICS)
	@$(ECHO) "#"
	@$(ECHO) "# Making $< $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fc $<

$(OBJ_DIR_SUPPORT)/%.$(OBJEXT): $(SRC_DIR_SUPPORT)/%.c | $(OBJ_DIR_SUPPORT)
	@$(ECHO) "#"
	@$(ECHO) "# Making $< $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fc $<

$(OBJ_DIR_TRANSFORM)/%.$(OBJEXT): $(SRC_DIR_TRANSFORM)/%.S | $(OBJ_DIR_TRANSFORM)
	@$(ECHO) "#"
	@$(ECHO) "# Making $< $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fa $<

$(OBJ_DIR_TRANSFORM)/%.$(OBJEXT): $(SRC_DIR_TRANSFORM)/%.c | $(OBJ_DIR_TRANSFORM)
	@$(ECHO) "#"
	@$(ECHO) "# Making $< $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fc $<
	

#  ======== rule for creation of output directories ========
$(OBJ_DIRS):
	@echo "mkdir $@"
	$(shell $(MKDIR) -p $@)

install:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@$(MKDIR) $(EXEC_DIR)/debug
	$(CP) bin/debug/ti_math_Cortex_R5_lspf.lib $(EXEC_DIR)/debug
	@$(MKDIR) $(EXEC_DIR)/release
	$(CP) bin/release/ti_math_Cortex_R5_lspf.lib $(EXEC_DIR)/release

help:
	@$(ECHO) "make                   # build executable"
	@$(ECHO) "make clean             # clean everything"

clean::
	$(RMDIR) bin

    
#  ======== install validation ========
ifeq (install,$(MAKECMDGOALS))
ifeq (,$(EXEC_DIR))
$(error must specify EXEC_DIR)
endif
endif

#  ======== tool chain macros ========
CGTOOLS = $(ti_targets_elf_R5F)

CC = $(CGTOOLS)/bin/armcl
AR = $(CGTOOLS)/bin/armar

# compiler flags
CPPFLAGS =
#CFLAGS = -k -mv$(PROC_VER) --code_state=32 --float_support=$(VFP_VER) --fp_mode=strict --vectorize=off --little_endian --enum_type=packed -diag_warning=225 --display_error_number -ppd=$@.dep -ppa $(CPREDEFS) $(CCPROFILE_$(PROFILE)) $(CINCDIRS)
# asm flag
#CFLAGS = -k -mv$(PROC_VER) --code_state=32 --float_support=$(VFP_VER) --fp_mode=strict --vectorize=off --little_endian --enum_type=packed -diag_warning=225 --display_error_number -ppd=$@.dep -ppa $(CPREDEFS) $(CCPROFILE_$(PROFILE)) $(CINCDIRS)  --asm_extension=.S
CFLAGS = -mv$(PROC_VER) --code_state=32 --float_support=$(VFP_VER) --fp_mode=strict --vectorize=off --little_endian --enum_type=packed -diag_warning=225 --display_error_number -ppd=$@.dep -ppa $(CPREDEFS) $(CCPROFILE_$(PROFILE)) $(CINCDIRS)  --asm_extension=.S

#CPREDEFS = --define=ARM_MATH_MATRIX_CHECK --define=ARM_MATH_ROUNDING --define=__FPU_PRESENT=1 --define=ARM_MATH_CM4
# TI asm flag
CPREDEFS = --define=ARM_MATH_MATRIX_CHECK --define=ARM_MATH_ROUNDING --define=__FPU_PRESENT=1 --define=ARM_MATH_CM4 --define=__TI_ARM__ --define=__COMPILER_BARRIER

CINCDIRS = -I"$(CGTOOLS)/include" \
    -I"$(CMSIS_ROOT)/Core/Include" \
    -I"$(CMSIS_ROOT)/DSP/PrivateInclude" \
    -I"Include"
    
CCPROFILE_debug = --symdebug:dwarf
CCPROFILE_release = -O3 --opt_for_speed=5

# archiver flags
ARFLAGS = r

#  ======== standard macros ========
ifneq (,$(wildcard $(XDC_INSTALL_DIR)/bin/echo.exe))
    # use these on Windows
    CP      = $(XDC_INSTALL_DIR)/bin/cp
    ECHO    = $(XDC_INSTALL_DIR)/bin/echo
    MKDIR   = $(XDC_INSTALL_DIR)/bin/mkdir -p
    RM      = $(XDC_INSTALL_DIR)/bin/rm -f
    RMDIR   = $(XDC_INSTALL_DIR)/bin/rm -rf
else
    # use these on Linux
    CP      = cp
    ECHO    = echo
    MKDIR   = mkdir -p
    RM      = rm -f
    RMDIR   = rm -rf
endif
    
#  ======== debug, rule to show build variables ========      
.show_build_vars:
	@echo Object file base directory:
	@echo $(OBJ_DIR_BASE)
	@echo #
	@echo Source directories
	@echo $(SRC_DIR_BASIC_MATH)
	@echo $(SRC_DIR_COMMON_TABLES)
	@echo $(SRC_DIR_COMPLEX_MATH)
	@echo $(SRC_DIR_CONTROLLER)
	@echo $(SRC_DIR_FAST_MATH)
	@echo $(SRC_DIR_FILTERING)
	@echo $(SRC_DIR_MATRIX)
	@echo $(SRC_DIR_STATISTICS)
	@echo $(SRC_DIR_SUPPORT)
	@echo $(SRC_DIR_TRANSFORM)
	@echo #
	@echo Source files
	@echo $(SRCS_BASIC_MATH)
	@echo $(SRCS_COMMON_TABLES)
	@echo $(SRCS_COMPLEX_MATH)
	@echo $(SRCS_CONTROLLER)
	@echo $(SRCS_FAST_MATH)
	@echo $(SRCS_FILTERING)
	@echo $(SRCS_MATRIX)
	@echo $(SRCS_STATISTICS)
	@echo $(SRCS_SUPPORT)
	@echo $(SRCS_TRANSFORM)
	@echo $(ASM_SRCS_TRANSFORM)
	@echo Object directories
	@echo $(OBJ_DIRS)
	@echo #
	@echo Object files
	@echo $(OBJS)
	@echo #
	@echo #
	@echo Dependency files
	@echo $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_BASIC_MATH))
	@echo $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_COMMON_TABLES))
	@echo $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_COMPLEX_MATH))
	@echo $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_CONTROLLER))
	@echo $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_FAST_MATH))
	@echo $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_FILTERING))
	@echo $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_MATRIX))
	@echo $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_STATISTICS))
	@echo $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_SUPPORT))
	@echo $(patsubst %.$(OBJEXT),%.$(OBJEXT).dep,$(OBJS_TRANSFORM))
