#!../../bin/linux-x86_64-debug/test

epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "3000100")

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

# libaravis amera name (use arv-tool to list accessible cameras)
epicsEnvSet("ARVCAM", "The Imaging Source Europe GmbH-43510084")

epicsEnvSet("EPICS_DB_INCLUDE_PATH", "../../db")

dbLoadDatabase("../../dbd/test.dbd")
test_registerRecordDeviceDriver(pdbbase) 

aravisCameraConfig("$(PORT)", "$(ARVCAM)")
dbLoadRecords("$(ARAVISGIGE)/db/aravisCamera.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

NDWarpConfigure("Warp1", 3, 0, "$(PORT)", 0)
dbLoadRecords("NDWarp.template","P=$(PREFIX),R=warp1:,PORT=Warp1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT)")

NDStatsConfigure("STATS1", $(QSIZE), 0, "Warp1", 0, 0, 0)
dbLoadRecords("NDStats.template",     "P=$(PREFIX),R=Stats1:,  PORT=STATS1,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(XSIZE),YSIZE=$(YSIZE),NCHANS=$(NCHANS),NDARRAY_PORT=Warp1")

NDStdArraysConfigure("Image1", 3, 0, "Warp1", 0)
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=Warp1,TYPE=Int8,FTVL=UCHAR,NELEMENTS=3000000")

iocInit()

dbpf "$(PREFIX)cam1:ImageMode" "Single"
dbpf "$(PREFIX)cam1:ArrayCallbacks" "Enable"
dbpf "$(PREFIX)warp1:EnableCallbacks" "Enable"
dbpf "$(PREFIX)image1:EnableCallbacks" "Enable"

dbpf "$(PREFIX)warp1:CenterX" "1000"
dbpf "$(PREFIX)warp1:CenterY" "850"
dbpf "$(PREFIX)cam1:AcquirePeriod" "0.1"
