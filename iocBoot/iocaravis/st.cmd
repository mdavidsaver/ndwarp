#!../../bin/linux-x86_64-debug/test

epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "6000100")

# Prefix for all records
epicsEnvSet("PREFIX", "13ARV1:")
# The port name for the detector
epicsEnvSet("PORT",   "ARV1")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximim image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "2000")
# The maximim image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "1500")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")

# libaravis camera name (use arv-tool to list accessible cameras)
epicsEnvSet("ARVCAM", "The Imaging Source Europe GmbH-39510105")

epicsEnvSet("EPICS_DB_INCLUDE_PATH", "../../db")

dbLoadDatabase("../../dbd/test.dbd")
test_registerRecordDeviceDriver(pdbbase) 

aravisCameraConfig("$(PORT)", "$(ARVCAM)")
dbLoadRecords("aravisCamera.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")
dbLoadRecords("TIS_dmk_33GX174.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

NDWarpConfigure("WARP1", 3, 0, "$(PORT)", 0)
dbLoadRecords("NDWarp.template","P=$(PREFIX),R=warp1:,PORT=WARP1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")
dbLoadRecords("NDWarp-frib-angle.db", "N=$(PREFIX)warp1:,P=$(PREFIX),R=warp1:")

NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=Warp1,TYPE=Int16,FTVL=SHORT,NELEMENTS=3000000")

#- From commonPlugins.cmd

# Create a TIFF file saving plugin
NDFileTIFFConfigure("FileTIFF1", $(QSIZE), 0, "Warp1", 0)
dbLoadRecords("NDFileTIFF.template",  "P=$(PREFIX),R=TIFF1:,PORT=FileTIFF1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

NDROIConfigure("ROI1", $(QSIZE), 0, "$(PORT)", 0, 0, 0)
dbLoadRecords("NDROI.template",       "P=$(PREFIX),R=ROI1:,  PORT=ROI1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")
NDROIConfigure("ROI2", $(QSIZE), 0, "$(PORT)", 0, 0, 0)
dbLoadRecords("NDROI.template",       "P=$(PREFIX),R=ROI2:,  PORT=ROI2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

NDROIStatConfigure("ROISTAT1", $(QSIZE), 0, "$(PORT)", 0, 8, 0, 0)
dbLoadRecords("NDROIStat.template",   "P=$(PREFIX),R=ROIStat1:  ,PORT=ROISTAT1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template",  "P=$(PREFIX),R=ROIStat1:1:,PORT=ROISTAT1,ADDR=0,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template",  "P=$(PREFIX),R=ROIStat1:2:,PORT=ROISTAT1,ADDR=1,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template",  "P=$(PREFIX),R=ROIStat1:3:,PORT=ROISTAT1,ADDR=2,TIMEOUT=1,NCHANS=$(NCHANS)")

NDProcessConfigure("PROC1", $(QSIZE), 0, "$(PORT)", 0, 0, 0)
dbLoadRecords("NDProcess.template",   "P=$(PREFIX),R=Proc1:,  PORT=PROC1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

NDStatsConfigure("STATS1", $(QSIZE), 0, "$(PORT)", 0, 0, 0)
dbLoadRecords("NDStats.template",     "P=$(PREFIX),R=Stats1:,  PORT=STATS1,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(XSIZE),YSIZE=$(YSIZE),NCHANS=$(NCHANS),NDARRAY_PORT=$(PORT)")
NDStatsConfigure("STATS2", $(QSIZE), 0, "ROI1", 0, 0, 0)
dbLoadRecords("NDStats.template",     "P=$(PREFIX),R=Stats2:,  PORT=STATS2,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(XSIZE),YSIZE=$(YSIZE),NCHANS=$(NCHANS),NDARRAY_PORT=$(PORT)")

NDOverlayConfigure("OVER1", $(QSIZE), 0, "$(PORT)", 0, 8, 0, 0)
dbLoadRecords("NDOverlay.template", "P=$(PREFIX),R=Over1:, PORT=OVER1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")
dbLoadRecords("NDOverlayN.template","P=$(PREFIX),R=Over1:1:,NAME=ROI1,   SHAPE=1,O=Over1:,XPOS=$(PREFIX)ROI1:MinX_RBV,YPOS=$(PREFIX)ROI1:MinY_RBV,XSIZE=$(PREFIX)ROI1:SizeX_RBV,YSIZE=$(PREFIX)ROI1:SizeY_RBV,PORT=OVER1,ADDR=0,TIMEOUT=1")
dbLoadRecords("NDOverlayN.template","P=$(PREFIX),R=Over1:2:,NAME=ROI2,   SHAPE=1,O=Over1:,XPOS=$(PREFIX)ROI2:MinX_RBV,YPOS=$(PREFIX)ROI2:MinY_RBV,XSIZE=$(PREFIX)ROI2:SizeX_RBV,YSIZE=$(PREFIX)ROI2:SizeY_RBV,PORT=OVER1,ADDR=1,TIMEOUT=1")
dbLoadRecords("NDOverlayN.template","P=$(PREFIX),R=Over1:3:,NAME=ROI3,   SHAPE=1,O=Over1:,XPOS=$(PREFIX)ROI3:MinX_RBV,YPOS=$(PREFIX)ROI3:MinY_RBV,XSIZE=$(PREFIX)ROI3:SizeX_RBV,YSIZE=$(PREFIX)ROI3:SizeY_RBV,PORT=OVER1,ADDR=2,TIMEOUT=1")
dbLoadRecords("NDOverlayN.template","P=$(PREFIX),R=Over1:4:,NAME=ROI4,   SHAPE=1,O=Over1:,XPOS=$(PREFIX)ROI4:MinX_RBV,YPOS=$(PREFIX)ROI4:MinY_RBV,XSIZE=$(PREFIX)ROI4:SizeX_RBV,YSIZE=$(PREFIX)ROI4:SizeY_RBV,PORT=OVER1,ADDR=3,TIMEOUT=1")

dbLoadRecords("iocAdminSoft.db", "IOC=$(PREFIX)")

save_restoreSet_status_prefix("$(PREFIX)")
dbLoadRecords("save_restoreStatus.db", "P=$(PREFIX)")

#- end commonPlugins.cmd

iocInit()

#dbpf "$(PREFIX)cam1:LEFTSHIFT" "No"

dbpf "$(PREFIX)cam1:ImageMode" "Single"
dbpf "$(PREFIX)cam1:ArrayCallbacks" "Enable"
dbpf "$(PREFIX)warp1:EnableCallbacks" "Enable"
dbpf "$(PREFIX)Stats1:EnableCallbacks" "Enable"
dbpf "$(PREFIX)ROI1:EnableCallbacks" "Enable"
dbpf "$(PREFIX)Over1:EnableCallbacks" "Enable"
dbpf "$(PREFIX)image1:EnableCallbacks" "Enable"

dbpf "$(PREFIX)warp1:CenterX" "1000"
dbpf "$(PREFIX)warp1:CenterY" "850"
dbpf "$(PREFIX)cam1:AcquirePeriod" "0.1"

dbpf "$(PREFIX)Over1:1:Use" "Yes"
dbpf "$(PREFIX)Over1:1:Green" "4095"
dbpf "$(PREFIX)Over1:1:DrawMode" "Set"
