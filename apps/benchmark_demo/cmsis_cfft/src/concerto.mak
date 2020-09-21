# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.


# Begin the concerto module declarations by includng the "PRELUDE"
MCUNUM := 1_0
_MODULE = MCU$(MCUNUM)

include $(PRELUDE)

include ${SDIR}/cfft_concerto_inc.mak

# End concerto module declarations
include $(FINALE)


# Begin the concerto module declarations by includng the "PRELUDE"
MCUNUM := 1_1
_MODULE = MCU$(MCUNUM)

include $(PRELUDE)

include ${SDIR}/cfft_concerto_inc.mak

# End concerto module declarations
include $(FINALE)

ifeq ($(TARGET_PLATFORM),AM64X)
# Begin the concerto module declarations by includng the "PRELUDE"
MCUNUM := 2_0
_MODULE = MCU$(MCUNUM)

include $(PRELUDE)

include ${SDIR}/cfft_concerto_inc.mak

# End concerto module declarations
include $(FINALE)


# Begin the concerto module declarations by includng the "PRELUDE"
MCUNUM := 2_1
_MODULE = MCU$(MCUNUM)

include $(PRELUDE)

include ${SDIR}/cfft_concerto_inc.mak

# End concerto module declarations
include $(FINALE)
endif
