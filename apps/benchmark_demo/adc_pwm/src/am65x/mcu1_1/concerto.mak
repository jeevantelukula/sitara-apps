# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.


# Begin the concerto module declarations by includng the "PRELUDE"
include $(PRELUDE)

MCUNUM := 1_1
include ${SDIR}/../../adc_concerto_inc.mak

# End concerto module declarations
include $(FINALE)
