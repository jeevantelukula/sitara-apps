#
#  Copyright (c) 2012-2020 Texas Instruments Incorporated - http://www.ti.com
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
#  ======== products.mak ========
#

.PHONY: .show

# Processor-SDK-RTOS version
#   PRSDK_06_03_00_106
export PRSDK_VER = PRSDK_06_03_00_106
# CMSIS version
#   CMSIS_05_06_00
export CMSIS_VER = CMSIS_05_06_00

#### SW root installation directory ####
ifneq ($(OS),Windows_NT)
DEPOT = /home/minglocal/ti
else
DEPOT = c:/ti
endif

#### Tools installation directory ####
TOOLS = /home/minglocal/ti-processor-sdk-am65xx-evm-0.1A/rtos

#### CMSIS_5 installation directory ####
ifneq ($(OS),Windows_NT)
export CMSIS_ROOT       ?= $(DEPOT)/CMSIS_5/CMSIS
else
export CMSIS_ROOT       ?= $(DEPOT)/CMSIS_5/CMSIS
endif

#### Dependencies ####
XDC_INSTALL_DIR         ?= $(TOOLS)/xdctools_3_55_02_22_core
ti_targets_elf_R5F      ?= $(TOOLS)/ti-cgt-arm_18.12.5.LTS

# Use this goal to print your product variables.
.show:
	@echo "DEPOT                = $(DEPOT)"
	@echo "TOOLS                = $(TOOLS)"
	@echo "CMSIS_ROOT           = $(CMSIS_ROOT)"
	@echo "XDC_INSTALL_DIR      = $(XDC_INSTALL_DIR)"
	@echo "ti_targets_elf_R5F   = $(ti_targets_elf_R5F)"
