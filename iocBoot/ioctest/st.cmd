#!../../bin/linux-x86_64-debug/warp

#- You may have to change warp to something else
#- everywhere it appears in this file

#< envPaths

## Register all support components
dbLoadDatabase("../../dbd/warp.dbd",0,0)
warp_registerRecordDeviceDriver(pdbbase) 

## Load record instances
dbLoadRecords("../../db/warp.db","user=mdavidsaver")

iocInit()

## Start any sequence programs
#seq sncwarp,"user=mdavidsaver"
