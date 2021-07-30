TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# private headers
USR_CPPFLAGS += -I$(TOP)/warpApp/src

TESTPROD_HOST += testwarp
TESTS += testwarp

testwarp_SRCS += testwarp.cpp
testwarp_LIBS += NDPluginWarp NDPlugin ADBase asyn Com

#=============================
# Build the IOC application

PROD_IOC = test
# test.dbd will be created and installed
DBD += test.dbd

# test_registerRecordDeviceDriver.cpp derives from test.dbd
test_SRCS += test_registerRecordDeviceDriver.cpp
test_SRCS += testMain.cpp

test_DBD  += NDPluginWarp.dbd
test_LIBS += NDPluginWarp

ifneq ($(ADEXAMPLE),)
test_DBD += simDetectorSupport.dbd
test_LIBS += simDetector
endif

ifneq ($(ARAVISGIGE),)
test_DBD += aravisCameraSupport.dbd
test_LIBS += aravisCamera
USR_LDFLAGS += $(shell pkg-config --libs-only-L glib-2.0 aravis-0.6)
USR_SYS_LIBS += $(patsubst -l%,%,$(shell pkg-config --libs-only-l glib-2.0 aravis-0.6))
endif

# used by commonDriverMakefile
PROD_NAME = test

include $(firstword $(wildcard $(ADCORE)/configure/commonDriverMakefile $(ADCORE)/ADApp/commonDriverMakefile))

ifneq ($(CAPUTLOG),)
test_DBD += caPutLog.dbd
test_LIBS += caPutLog
endif

ifneq ($(RECSYNC),)
test_DBD += reccaster.dbd
test_LIBS += reccaster
endif

TESTSCRIPTS_HOST += $(TESTS:%=%.t)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

