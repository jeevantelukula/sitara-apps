include Rules.make

# Set default target
all:

clean:

install:

# Included fragments can map to the above targets
#include $(wildcard makerules/*.mak)
include makerules/benchmark_demo.mak

