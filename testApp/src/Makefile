TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# private headers
USR_CPPFLAGS += -I$(TOP)/warpApp/src

TESTPROD_HOST += testwarp

testwarp_SRCS += testwarp.cpp
testwarp_LIBS += NDPluginWarp NDPlugin ADBase asyn Com

#=============================
# Build the IOC application

PROD_IOC = test
# test.dbd will be created and installed
DBD += test.dbd

# test.dbd will be made up from these files:
test_DBD += base.dbd
test_DBD += ADSupport.dbd
test_DBD += NDPluginSupport.dbd
test_DBD += NDPluginWarp.dbd

# Include dbd files from all support applications:
#test_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
test_LIBS += NDPluginWarp NDPlugin ADBase asyn

# test_registerRecordDeviceDriver.cpp derives from test.dbd
test_SRCS += test_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
test_SRCS_DEFAULT += testMain.cpp
test_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#test_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
test_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

