include Rules.make

# Set default target
all:

clean:

install:

# Included fragments can map to the above targets
#include $(wildcard makerules/*.mak)
ifeq ($(TARGET_PLATFORM),AM65X)
include makerules/benchmark_demo.mak
include makerules/ipc_rpmsg_lib.mak
include makerules/timesync_example.mak
endif
include makerules/servo_drive_demo.mak
