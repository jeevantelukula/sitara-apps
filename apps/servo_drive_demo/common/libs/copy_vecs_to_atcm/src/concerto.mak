# This is a TI RTOS (SYSBIOS) based application developed specifically for the
# R5F CPU architecture. This must be filtered here so that concerto does not
# attempt any other combinations.
ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),R5F)

# Begin the concerto module declarations by including the "PRELUDE"
include $(PRELUDE)

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := app_libs_copy_vecs_to_atcm
TARGETTYPE  := library
ASSEMBLY    := copy_vecs_to_atcm.S

# End concerto module declarations
include $(FINALE)

endif
endif
