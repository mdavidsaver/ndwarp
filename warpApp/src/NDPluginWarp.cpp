/*
 * This software is Copyright by the Board of Trustees of Michigan
 * State University (c) Copyright 2016.
 *
 * Author: Michael Davidsaver <mdavidsaver@gmail.com>
 */
#include <algorithm>

#include <epicsMath.h>

#include <iocsh.h>
#include <epicsExport.h>

#include <NDPluginWarp.h>

static
bool sameShape(const NDArrayInfo& lhs, const NDArrayInfo& rhs)
{
    // assumes previous check ndim is 2 or 3
    return lhs.nElements==rhs.nElements
            && lhs.xSize==rhs.xSize
            && lhs.ySize==rhs.ySize
            && lhs.colorSize==rhs.colorSize;
}

NDPluginWarp::NDPluginWarp(const char *portName, int queueSize, int blockingCallbacks,
             const char *NDArrayPort, int NDArrayAddr,
             int maxBuffers, size_t maxMemory,
             int priority, int stackSize)
    : NDPluginDriver(portName, queueSize, blockingCallbacks,
                   NDArrayPort, NDArrayAddr, 1, NUM_NDPLUGIN_WARP_PARAMS, maxBuffers, maxMemory,
                   asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
                   asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
                   0, 1, priority, stackSize)
{
    lastinfo.nElements = (size_t)-1; // spoil

    createParam(NDWarpRunTimeString, asynParamFloat64, &NDWarpRunTime);
    createParam(NDWarpModeString, asynParamInt32, &NDWarpMode);
    createParam(NDWarpOutputString, asynParamInt32, &NDWarpOutput);

    setIntegerParam(NDWarpMode, 0); // nearest neighbor

    createParam(NDWarpAxisString, asynParamInt32, &NDWarpAxis);
    createParam(NDWarpFactorString, asynParamFloat64, &NDWarpFactor);
    createParam(NDWarpCenterXString, asynParamInt32, &NDWarpCenterX);
    createParam(NDWarpCenterYString, asynParamInt32, &NDWarpCenterY);

    setStringParam(NDPluginDriverPluginType, "NDPluginWarp");

    setDoubleParam(NDWarpFactor, 0.0); // initialize w/ no-op
}

NDPluginWarp::~NDPluginWarp() {
    assert(false); // never called (asyn ports can't be destory'd)
}

namespace {
template<typename T>
void warpit(NDArray *atemp,
            NDArray *output,
            const NDPluginWarp::Sample *S,
            size_t nElements,
            unsigned samp_per_pixel)
{
    const T * const  I = (const T*)atemp->pData;
    T       *        O = (T*)output->pData,
            * const OE = O+nElements;

    for(; O<OE; O++) {
        double val = 0.0;
        for(unsigned j=0; j<samp_per_pixel; j++, S++) {
            val += S->weight * I[S->index];
        }
        *O = (T)val;
    }

}
} // namespace

void
NDPluginWarp::processCallbacks(NDArray *pArray)
{
    // pArray is borrowed reference.  Caller will release()

    NDPluginDriver::processCallbacks(pArray);

    NDArrayInfo info;
    (void)pArray->getInfo(&info);

    if(pArray->ndims!=2 || info.xSize==0 || info.ySize==0) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:: 2D non-empty expected",
                  this->portName);
        return;
    }
    switch(pArray->dataType) {
#define CASE(TYPE) case ND ## TYPE:
    CASE(Int8)
    CASE(UInt8)
    CASE(Int16)
    CASE(UInt16)
    CASE(Int32)
    CASE(UInt32)
    CASE(Float32)
    CASE(Float64)
#undef CASE
        break;
    default:
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:: Unsupported type %u",
                  this->portName, (unsigned)pArray->dataType);
        return;
    }
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s: %s ndarray=%p\n", this->portName, __PRETTY_FUNCTION__, pArray);

    try {
        epicsTimeStamp before;
        epicsTimeGetCurrent(&before);

        if(!sameShape(info, lastinfo)) {
            asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: input shape changes, recompute mapping\n", this->portName);

            lastinfo = info;
            recalculate_transform(info);

            assert(mapping.size() == samp_per_pixel*lastinfo.nElements);
        }

        int outputmode=0;
        getIntegerParam(NDWarpOutput, &outputmode);

        ndarray_ptr output;

        if(outputmode==0) {
            // output transformed image

            output.reset(cloneArray(pNDArrayPool, pArray));

            switch(pArray->dataType) {
#define CASE(TYPE) case ND ## TYPE: warpit<epics ## TYPE>(pArray, output.get(), &mapping[0], lastinfo.nElements, samp_per_pixel); break;
            CASE(Int8)
            CASE(UInt8)
            CASE(Int16)
            CASE(UInt16)
            CASE(Int32)
            CASE(UInt32)
            CASE(Float32)
            CASE(Float64)
#undef CASE
                    // no default: (error check is above) meant to trigger compiler warning if new types are added
            }

        } else if(outputmode>=1 && outputmode<=3) {
            // output mapping distance, x, or y  (to help debug mappings)
            size_t dims[2] = {lastmap.sizex(), lastmap.sizey()};

            output.reset(pNDArrayPool->alloc(2, dims, NDFloat64, 0, NULL));
            double *arr = (double*)output->pData;
            NDArrayInfo_t info;
            output->getInfo(&info);

            for(size_t oy=0; oy<lastmap.sizey(); oy++) {
                for(size_t ox=0; ox<lastmap.sizey(); ox++) {
                    double ix = lastmap.x(ox, oy),
                           iy = lastmap.y(ox, oy),
                           dx = ox-ix,
                           dy = oy-iy;

                    double result = 0.0;

                    switch(outputmode) {
                    case 1: result = sqrt(dx*dx+dy*dy); break;
                    case 2: result = dx; break;
                    case 3: result = dy; break;
                    }

                    arr[ox*info.xStride + oy*info.yStride] = result;
                }
            }

        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:: unknown output mode",
                      this->portName);
        }

        epicsTimeStamp after;
        epicsTimeGetCurrent(&after);
        double delta = epicsTimeDiffInSeconds(&after, &before);
        setDoubleParam(NDWarpRunTime, delta);

        if(output.get()) {
            aPDUnlock U(*this);
            doCallbacksGenericPointer(output.get(), NDArrayData, 0);
        }

        callParamCallbacks();

        // release() both atemp and output
    }catch(std::exception& e){
        lastinfo.nElements = (size_t)-1; // spoil

        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:: unhandled exception in %s : %s",
                  portName, __PRETTY_FUNCTION__, e.what());
    }
}

asynStatus NDPluginWarp::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    asynStatus ret;
    int function = pasynUser->reason;

    if(function<FIRST_NDPLUGIN_WARP_PARAM)
        return NDPluginDriver::writeFloat64(pasynUser, value);

    int addr = 0;
    ret = getAddress(pasynUser, &addr);

    if(!ret) ret = setDoubleParam(addr, function, value);

    try {
        if(ret) {
            // no-op
        } else if(function==NDWarpFactor) {
            lastinfo.nElements = (size_t)-1; // spoil
        } else {
            ret = asynError;
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s:: status=%d, addr=%d function=%d, value=%g : Write to read-only parameter",
                          __PRETTY_FUNCTION__, ret, addr, function, value);
        }
    }catch(std::exception& e) {
        ret = asynError;
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "%s:: status=%d, addr=%d function=%d, value=%g : %s",
                      __PRETTY_FUNCTION__, ret, addr, function, value, e.what());

        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s : Unhandled exception\n",
                  pasynUser->errorMessage);
    }

    (void)callParamCallbacks(addr, addr);

    return ret;
}

asynStatus NDPluginWarp::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    asynStatus ret;
    int function = pasynUser->reason;

    if(function<FIRST_NDPLUGIN_WARP_PARAM)
        return NDPluginDriver::writeInt32(pasynUser, value);

    int addr = 0;
    ret = getAddress(pasynUser, &addr);

    if(!ret) ret = setIntegerParam(addr, function, value);

    try {
        if(ret || function==NDWarpOutput) {
            // no-op
        } else if(function==NDWarpAxis
                  || function==NDWarpCenterX || function==NDWarpCenterY
                  || function==NDWarpMode) {
            lastinfo.nElements = (size_t)-1; // spoil
        } else {
            ret = asynError;
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s:: status=%d, addr=%d function=%d, value=%d : Write to read-only parameter",
                          __PRETTY_FUNCTION__, ret, addr, function, value);
        }
    }catch(std::exception& e) {
        ret = asynError;
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "%s:: status=%d, addr=%d function=%d, value=%d : %s",
                      __PRETTY_FUNCTION__, ret, addr, function, value, e.what());

        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s : Unhandled exception\n",
                  pasynUser->errorMessage);
    }

    (void)callParamCallbacks(addr, addr);

    return ret;
}

void NDPluginWarp::recalculate_transform(const NDArrayInfo& info)
{
    int rawmode=0;

    getIntegerParam(NDWarpMode, &rawmode);

    cur_mode = (mode_t)rawmode;

    switch(cur_mode) {
    case Nearest:  samp_per_pixel = 1; break;
    case Bilinear: samp_per_pixel = 4; break;
    default:
        throw std::runtime_error("Invalid interpolation mode");
    }
    mapping.resize(samp_per_pixel*info.nElements);

    Mapping M;
    M.resize(info);

    fill_mapping(M);

    // sanitize mapping
    // clip to range [0, size-1]
    for(size_t j=0; j<M.sizey(); j++) {

        for(size_t i=0; i<M.sizex(); i++) {
            double x=M.x(i,j);

            if(x>=M.sizex()-1)  // handles +inf
                M.x(i,j) = M.sizex()-1;
            else if(x<0.0 || !isfinite(x)) // handles -inf and nan
                M.x(i,j) = 0.0;

            double y=M.y(i,j);

            if(y<0.0 || !isfinite(y))
                M.y(i,j) = 0.0;
            else if(y>=M.sizey()-1)
                M.y(i,j) = M.sizey()-1;
        }
    }

    // interpolate and collapse user 2D Mapping to 1D 'mapping' (from offset to offset)

    Sample * const S = &mapping[0];

    switch(cur_mode) {
    case Nearest: {
        assert(samp_per_pixel==1);
        for(size_t j=0; j<M.sizey(); j++) {

            for(size_t i=0; i<M.sizex(); i++) {
                const size_t Ooffset = i*lastinfo.xStride + j*lastinfo.yStride;
                Sample * const SX = &S[Ooffset];

                const size_t Ioffset = round(M.x(i,j))*lastinfo.xStride
                                     + round(M.y(i,j))*lastinfo.yStride;

                SX->weight = 1.0;
                SX->index = Ioffset;
            }
        }
    }
        break;
    case Bilinear: {
        assert(samp_per_pixel==4);
        for(size_t j=0; j<M.sizey(); j++) {

            for(size_t i=0; i<M.sizex(); i++) {
                const size_t Ooffset = 4*(i*lastinfo.xStride + j*lastinfo.yStride);
                Sample * const SX = &S[Ooffset];

                double x=M.x(i,j), y=M.y(i,j),
                       fx=floor(x), fy=floor(y),
                       cx=ceil(x), cy=ceil(y),
                       xfact = cx==fx ? 0.0 : (x-fx)/(cx-fx),
                       yfact = cy==fy ? 0.0 : (y-fy)/(cy-fy);

                // for degenerate cases (f_==c_) then
                // _fact=0.0 or 1.0 gives the same result.
                // we choose 0.0 arbitrarily

                // should be sanitized above
                assert(cx < M.sizex());
                assert(cy < M.sizey());

                SX[0].weight = (1.0-xfact)*(1.0-yfact);
                SX[0].index  = fx*lastinfo.xStride + fy*lastinfo.yStride;

                SX[1].weight = xfact*(1.0-yfact);
                SX[1].index  = cx*lastinfo.xStride + fy*lastinfo.yStride;

                SX[2].weight = (1.0-xfact)*yfact;
                SX[2].index  = fx*lastinfo.xStride + cy*lastinfo.yStride;

                SX[3].weight = xfact*yfact;
                SX[3].index  = cx*lastinfo.xStride + cy*lastinfo.yStride;
            }
        }
    }
        break;
    }

    lastmap.swap(M);
}

void NDPluginWarp::fill_mapping(Mapping &M)
{
    double factor = 0.0;
    int center[2] = {0, 0};
    int axis = 0;

    getIntegerParam(NDWarpAxis, &axis);
    getIntegerParam(NDWarpCenterX, &center[0]);
    getIntegerParam(NDWarpCenterY, &center[1]);
    getDoubleParam(NDWarpFactor, &factor);
    //printf("# fill_mapping w/ %d about %d x %d\n", axis, center[0], center[1]);

    // iterate through output image coordinates.
    // fill in each with input image coordinate
    for(epicsUInt16 x=0; x<M.sizex(); x++) {
        for(epicsUInt16 y=0; y<M.sizey(); y++) {
            epicsInt32 xc = x-center[0];
            epicsInt32 yc = y-center[1];
            xc = axis==0 ? -xc :  xc;
            yc = axis==0 ?  yc : -yc;
            M.x(x,y) = xc+center[0];
            M.y(x,y) = yc+center[1];
        }
    }
}

extern "C" int NDWarpConfigure(const char *portName, int queueSize, int blockingCallbacks,
                                const char *NDArrayPort, int NDArrayAddr,
                                int maxBuffers, size_t maxMemory)
{
    new NDPluginWarp(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr,
                     maxBuffers, maxMemory, 0, 2000000);
    return(asynSuccess);
}

/* EPICS iocsh shell commands */
static const iocshArg initArg0 = { "portName",iocshArgString};
static const iocshArg initArg1 = { "frame queue size",iocshArgInt};
static const iocshArg initArg2 = { "blocking callbacks",iocshArgInt};
static const iocshArg initArg3 = { "NDArrayPort",iocshArgString};
static const iocshArg initArg4 = { "NDArrayAddr",iocshArgInt};
static const iocshArg initArg5 = { "maxBuffers",iocshArgInt};
static const iocshArg initArg6 = { "maxMemory",iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0,
                                            &initArg1,
                                            &initArg2,
                                            &initArg3,
                                            &initArg4,
                                            &initArg5,
                                            &initArg6};
static const iocshFuncDef initFuncDef = {"NDWarpConfigure",7,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    NDWarpConfigure(args[0].sval, args[1].ival, args[2].ival,
                     args[3].sval, args[4].ival, args[5].ival,
                     args[6].ival);
}

static void NDWarpRegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

extern "C" {
epicsExportRegistrar(NDWarpRegister);
}
