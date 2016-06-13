#!../../bin/linux-x86_64-debug/test

epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "3000100")

# Prefix for all records
epicsEnvSet("PREFIX", "13SIM1:")
# The port name for the detector
epicsEnvSet("PORT",   "SIM1")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximim image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "2000")
# The maximim image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "1500")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")

epicsEnvSet("EPICS_DB_INCLUDE_PATH", "../../db")

dbLoadDatabase("../../dbd/test.dbd")
test_registerRecordDeviceDriver(pdbbase) 

simDetectorConfig("$(PORT)", $(XSIZE), $(YSIZE), 1, 0, 0)
dbLoadRecords("simDetector.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

NDWarpConfigure("Warp1", 3, 0, "$(PORT)", 0)
dbLoadRecords("NDWarp.template","P=$(PREFIX),R=warp1:,PORT=Warp1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

NDStatsConfigure("STATS1", $(QSIZE), 0, "Warp1", 0, 0, 0)
dbLoadRecords("NDStats.template",     "P=$(PREFIX),R=Stats1:,  PORT=STATS1,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(XSIZE),YSIZE=$(YSIZE),NCHANS=$(NCHANS),NDARRAY_PORT=Warp1")

NDStdArraysConfigure("Image1", 3, 0, "Warp1", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=Warp1,TYPE=Int8,FTVL=UCHAR,NELEMENTS=3000000")

# Create 4 ROI plugins
NDROIConfigure("ROI1", $(QSIZE), 0, "$(PORT)", 0, 0, 0)
dbLoadRecords("NDROI.template",       "P=$(PREFIX),R=ROI1:,  PORT=ROI1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")
NDROIConfigure("ROI2", $(QSIZE), 0, "$(PORT)", 0, 0, 0)
dbLoadRecords("NDROI.template",       "P=$(PREFIX),R=ROI2:,  PORT=ROI2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")
NDROIConfigure("ROI3", $(QSIZE), 0, "$(PORT)", 0, 0, 0)
dbLoadRecords("NDROI.template",       "P=$(PREFIX),R=ROI3:,  PORT=ROI3,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")
NDROIConfigure("ROI4", $(QSIZE), 0, "$(PORT)", 0, 0, 0)
dbLoadRecords("NDROI.template",       "P=$(PREFIX),R=ROI4:,  PORT=ROI4,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

# Create 8 ROIStat plugins
NDROIStatConfigure("ROISTAT1", $(QSIZE), 0, "$(PORT)", 0, 8, 0, 0)
dbLoadRecords("NDROIStat.template",   "P=$(PREFIX),R=ROIStat1:  ,PORT=ROISTAT1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template",  "P=$(PREFIX),R=ROIStat1:1:,PORT=ROISTAT1,ADDR=0,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template",  "P=$(PREFIX),R=ROIStat1:2:,PORT=ROISTAT1,ADDR=1,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template",  "P=$(PREFIX),R=ROIStat1:3:,PORT=ROISTAT1,ADDR=2,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template",  "P=$(PREFIX),R=ROIStat1:4:,PORT=ROISTAT1,ADDR=3,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template",  "P=$(PREFIX),R=ROIStat1:5:,PORT=ROISTAT1,ADDR=4,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template",  "P=$(PREFIX),R=ROIStat1:6:,PORT=ROISTAT1,ADDR=5,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template",  "P=$(PREFIX),R=ROIStat1:7:,PORT=ROISTAT1,ADDR=6,TIMEOUT=1,NCHANS=$(NCHANS)")
dbLoadRecords("NDROIStatN.template",  "P=$(PREFIX),R=ROIStat1:8:,PORT=ROISTAT1,ADDR=7,TIMEOUT=1,NCHANS=$(NCHANS)")

# Create an overlay plugin with 8 overlays
NDOverlayConfigure("OVER1", $(QSIZE), 0, "$(PORT)", 0, 8, 0, 0)
dbLoadRecords("NDOverlay.template", "P=$(PREFIX),R=Over1:, PORT=OVER1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")
dbLoadRecords("NDOverlayN.template","P=$(PREFIX),R=Over1:1:,NAME=ROI1,   SHAPE=1,O=Over1:,XPOS=$(PREFIX)ROI1:MinX_RBV,YPOS=$(PREFIX)ROI1:MinY_RBV,XSIZE=$(PREFIX)ROI1:SizeX_RBV,YSIZE=$(PREFIX)ROI1:SizeY_RBV,PORT=OVER1,ADDR=0,TIMEOUT=1")
dbLoadRecords("NDOverlayN.template","P=$(PREFIX),R=Over1:2:,NAME=ROI2,   SHAPE=1,O=Over1:,XPOS=$(PREFIX)ROI2:MinX_RBV,YPOS=$(PREFIX)ROI2:MinY_RBV,XSIZE=$(PREFIX)ROI2:SizeX_RBV,YSIZE=$(PREFIX)ROI2:SizeY_RBV,PORT=OVER1,ADDR=1,TIMEOUT=1")
dbLoadRecords("NDOverlayN.template","P=$(PREFIX),R=Over1:3:,NAME=ROI3,   SHAPE=1,O=Over1:,XPOS=$(PREFIX)ROI3:MinX_RBV,YPOS=$(PREFIX)ROI3:MinY_RBV,XSIZE=$(PREFIX)ROI3:SizeX_RBV,YSIZE=$(PREFIX)ROI3:SizeY_RBV,PORT=OVER1,ADDR=2,TIMEOUT=1")
dbLoadRecords("NDOverlayN.template","P=$(PREFIX),R=Over1:4:,NAME=ROI4,   SHAPE=1,O=Over1:,XPOS=$(PREFIX)ROI4:MinX_RBV,YPOS=$(PREFIX)ROI4:MinY_RBV,XSIZE=$(PREFIX)ROI4:SizeX_RBV,YSIZE=$(PREFIX)ROI4:SizeY_RBV,PORT=OVER1,ADDR=3,TIMEOUT=1")
dbLoadRecords("NDOverlayN.template","P=$(PREFIX),R=Over1:5:,NAME=Cursor1,SHAPE=1,O=Over1:,XPOS=junk,                  YPOS=junk,                  XSIZE=junk,                   YSIZE=junk,                   PORT=OVER1,ADDR=4,TIMEOUT=1")
dbLoadRecords("NDOverlayN.template","P=$(PREFIX),R=Over1:6:,NAME=Cursor2,SHAPE=1,O=Over1:,XPOS=junk,                  YPOS=junk,                  XSIZE=junk,                   YSIZE=junk,                   PORT=OVER1,ADDR=5,TIMEOUT=1")
dbLoadRecords("NDOverlayN.template","P=$(PREFIX),R=Over1:7:,NAME=Box1,   SHAPE=1,O=Over1:,XPOS=junk,                  YPOS=junk,                  XSIZE=junk,                   YSIZE=junk,                   PORT=OVER1,ADDR=6,TIMEOUT=1")
dbLoadRecords("NDOverlayN.template","P=$(PREFIX),R=Over1:8:,NAME=Box2,   SHAPE=1,O=Over1:,XPOS=junk,                  YPOS=junk,                  XSIZE=junk,                   YSIZE=junk,                   PORT=OVER1,ADDR=7,TIMEOUT=1")

iocInit()

dbpf "$(PREFIX)cam1:ImageMode" "Single"
dbpf "$(PREFIX)cam1:ArrayCallbacks" "Enable"
dbpf "$(PREFIX)warp1:EnableCallbacks" "Enable"
dbpf "$(PREFIX)image1:EnableCallbacks" "Enable"

dbpf "$(PREFIX)warp1:CenterX" "1000"
dbpf "$(PREFIX)warp1:CenterY" "850"
dbpf "$(PREFIX)cam1:AcquirePeriod" "0.1"
