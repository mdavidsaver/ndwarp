TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

ifneq ($(ADEXAMPLE),)
DB_INSTALLS += $(ADEXAMPLE)/db/simDetector.template
endif
ifneq ($(ARAVISGIGE),)
DB_INSTALLS += $(ARAVISGIGE)/db/aravisCamera.template
DB_INSTALLS += $(ARAVISGIGE)/db/Basler_acA1600_20gm.template
DB_INSTALLS += $(ARAVISGIGE)/db/TIS_dmk_33GX174.template
endif
DB_INSTALLS += $(ADCORE)/db/ADBase.template
DB_INSTALLS += $(ADCORE)/db/NDArrayBase.template
DB_INSTALLS += $(ADCORE)/db/NDPluginBase.template
DB_INSTALLS += $(ADCORE)/db/NDStdArrays.template

DB_INSTALLS += $(ADCORE)/db/NDStats.template
DB_INSTALLS += $(ADCORE)/db/NDFile.template
DB_INSTALLS += $(ADCORE)/db/NDFileTIFF.template
DB_INSTALLS += $(ADCORE)/db/NDROI.template
DB_INSTALLS += $(ADCORE)/db/NDROIStat.template
DB_INSTALLS += $(ADCORE)/db/NDROIStatN.template
DB_INSTALLS += $(ADCORE)/db/NDProcess.template
DB_INSTALLS += $(ADCORE)/db/NDOverlay.template
DB_INSTALLS += $(ADCORE)/db/NDOverlayN.template

ifneq ($(DEVIOCSTATS),)
DB_INSTALLS += $(DEVIOCSTATS)/db/iocAdminSoft.db
endif
ifneq ($(AUTOSAVE),)
DB_INSTALLS += $(AUTOSAVE)/db/save_restoreStatus.db
endif

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
