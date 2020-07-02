
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), NO_OS))

# Begin the concerto module declarations by includng the "PRELUDE"
include $(PRELUDE)

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := app_libs_r5f_no_os_mpu_cfg
TARGETTYPE  := library

# Define library root directory
LIBDIR := $(abspath $(SDIR)/..)

CSOURCES    := $(call all-c-files)

# End concerto module declarations
include $(FINALE)

endif
