TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = warp
# warp.dbd will be created and installed
DBD += warp.dbd

# warp.dbd will be made up from these files:
warp_DBD += base.dbd

# Include dbd files from all support applications:
#warp_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
#warp_LIBS += xxx

# warp_registerRecordDeviceDriver.cpp derives from warp.dbd
warp_SRCS += warp_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
warp_SRCS_DEFAULT += warpMain.cpp
warp_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#warp_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
warp_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

