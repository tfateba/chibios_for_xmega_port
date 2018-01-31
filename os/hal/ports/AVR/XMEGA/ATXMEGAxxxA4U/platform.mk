# List of all the AVR platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/AVR/XMEGA/ATXMEGAxxxA4U/hal_lld.c \

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/ports/AVR/XMEGA/ATXMEGAxxxA4U

# Drivers compatible with the platform.
#include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/ADCv1/driver.mk
#include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/DACv1/driver.mk
#include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/DMAv1/driver.mk
#include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/EXTv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/GPIOv1/driver.mk
#include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/I2Cv1/driver.mk
#include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/OTGv1/driver.mk
#include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/RTCv1/driver.mk
#include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/SPIv1/driver.mk
include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/TIMv1/driver.mk
#include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/USARTv1/driver.mk
#include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/DMAv1/driver.mk
#include $(CHIBIOS)/os/hal/ports/AVR/XMEGA/LLD/WDGv1/driver.mk

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
