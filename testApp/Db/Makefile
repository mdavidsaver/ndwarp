TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

ifneq ($(ADEXAMPLE),)
DB_INSTALLS += $(ADEXAMPLE)/db/simDetector.template
endif
DB_INSTALLS += $(ADCORE)/db/ADBase.template
DB_INSTALLS += $(ADCORE)/db/NDArrayBase.template
DB_INSTALLS += $(ADCORE)/db/NDPluginBase.template
DB_INSTALLS += $(ADCORE)/db/NDStats.template
DB_INSTALLS += $(ADCORE)/db/NDStdArrays.template

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

