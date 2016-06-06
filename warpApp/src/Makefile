TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

LIBRARY += NDPluginWarp

DBD += NDPluginWarp.dbd

# headers are not considered public

# Add all the support libraries needed by this IOC
NDPluginWarp_LIBS += NDPlugin ADBase asyn

NDPluginWarp_SRCS += NDPluginWarp.cpp

# Finally link to the EPICS Base libraries
NDPluginWarp_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

