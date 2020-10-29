include Rules.make

# Set default target
all:

clean:

install:

# Included fragments can map to the above targets
#include $(wildcard makerules/*.mak)
include makerules/timesync_example.mak
include makerules/benchmark_demo.mak
include makerules/ipc_rpmsg_lib.mak
include makerules/servo_drive_demo.mak
