/*
 * This software is Copyright by the Board of Trustees of Michigan
 * State University (c) Copyright 2016.
 *
 * Author: Michael Davidsaver <mdavidsaver@gmail.com>
 */
#include <algorithm>
#include <limits>

#include <epicsMath.h>

#include <iocsh.h>
#include <epicsExport.h>

#include <NDPluginWarp.h>

// pi/180
#define PI_180 0.017453292519943295

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
    : NDPluginDriver(portName,
                     queueSize,
                     blockingCallbacks,
                     NDArrayPort,
                     NDArrayAddr,
                     1, // maxAddr
                     maxBuffers,
                     maxMemory,
                     asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
                     asynInt32ArrayMask | asynFloat64ArrayMask | asynGenericPointerMask,
                     0, // asynFlags
                     1, // autoConnect
                     priority,
                     stackSize,
                     1) // maxThreads
{
    lastinfo.nElements = (size_t)-1; // spoil

    createParam(NDWarpRunTimeString, asynParamFloat64, &NDWarpRunTime);
    createParam(NDWarpModeString, asynParamInt32, &NDWarpMode);
    createParam(NDWarpOutputString, asynParamInt32, &NDWarpOutput);

    setIntegerParam(NDWarpMode, 0); // nearest neighbor

    createParam(NDWarpAngleString, asynParamFloat64, &NDWarpAngle);
    createParam(NDWarpFactorXString, asynParamFloat64, &NDWarpFactorX);
    createParam(NDWarpFactorYString, asynParamFloat64, &NDWarpFactorY);
    createParam(NDWarpCenterXString, asynParamInt32, &NDWarpCenterX);
    createParam(NDWarpCenterYString, asynParamInt32, &NDWarpCenterY);

    setStringParam(NDPluginDriverPluginType, "NDPluginWarp");

    setDoubleParam(NDWarpFactorX, 0.0); // initialize w/ no-op
    setDoubleParam(NDWarpFactorY, 0.0);
    setDoubleParam(NDWarpAngle, 0.0);
    setIntegerParam(NDWarpCenterX, 0);
    setIntegerParam(NDWarpCenterY, 0);
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
        bool valid = true;
        for(unsigned j=0; j<samp_per_pixel; j++, S++) {
            valid &= S->valid;
            if(S->valid)
                val += S->weight * I[S->index];
        }
        if(!valid) val = 0.0;
        *O = (T)val;
    }

}
} // namespace

void
NDPluginWarp::processCallbacks(NDArray *pArray)
{
    // pArray is borrowed reference.  Caller will release()

    NDPluginDriver::beginProcessCallbacks(pArray);

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
    CASE(Int64)
    CASE(UInt64)
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
            CASE(Int64)
            CASE(UInt64)
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
            NDPluginDriver::endProcessCallbacks(output.release(), false, true);
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
        if(ret==asynSuccess) {
            lastinfo.nElements = (size_t)-1; // spoil
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
        if(ret==asynSuccess) {
            lastinfo.nElements = (size_t)-1; // spoil
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
    // replace outside range [0, size-1] with NaN
    for(size_t j=0; j<M.sizey(); j++) {

        for(size_t i=0; i<M.sizex(); i++) {
            double x=M.x(i,j), y=M.y(i,j);

            if(x>M.sizex()-1 || x<0.0 || !isfinite(x))
                M.x(i,j) = std::numeric_limits<double>::quiet_NaN();

            if(y>M.sizey()-1 || y<0.0 || !isfinite(y))
                M.y(i,j) = std::numeric_limits<double>::quiet_NaN();
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
                double x=round(M.x(i,j)), y=round(M.y(i,j));

                const size_t Ioffset = x*lastinfo.xStride
                                     + y*lastinfo.yStride;

                SX->weight = 1.0;
                SX->index = Ioffset;
                SX->valid = isfinite(x) && isfinite(y);
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
                bool valid = isfinite(x) && isfinite(y);

                // for degenerate cases (f_==c_) then
                // _fact=0.0 or 1.0 gives the same result.
                // we choose 0.0 arbitrarily

                SX[0].weight = (1.0-xfact)*(1.0-yfact);
                SX[0].index  = fx*lastinfo.xStride + fy*lastinfo.yStride;
                SX[0].valid  = valid;

                SX[1].weight = xfact*(1.0-yfact);
                SX[1].index  = cx*lastinfo.xStride + fy*lastinfo.yStride;
                SX[1].valid  = valid;

                SX[2].weight = (1.0-xfact)*yfact;
                SX[2].index  = fx*lastinfo.xStride + cy*lastinfo.yStride;
                SX[2].valid  = valid;

                SX[3].weight = xfact*yfact;
                SX[3].index  = cx*lastinfo.xStride + cy*lastinfo.yStride;
                SX[3].valid  = valid;
            }
        }
    }
        break;
    }

    lastmap.swap(M);
}

void NDPluginWarp::fill_mapping(Mapping &M)
{
    double angle = 0.0;
    double factor[2] = {0.0, 0.0};
    int center[2] = {0, 0};

    getDoubleParam(NDWarpAngle, &angle);
    getIntegerParam(NDWarpCenterX, &center[0]);
    getIntegerParam(NDWarpCenterY, &center[1]);
    getDoubleParam(NDWarpFactorX, &factor[0]);
    getDoubleParam(NDWarpFactorY, &factor[1]);
//    printf("# fill_mapping. center: [%d, %d] angle: %f F: [%f, %f]\n",
//           center[0], center[1], angle, factor[0], factor[1]);

    const double sina=sin(angle*PI_180), cosa=cos(angle*PI_180);

    // iterate through output image coordinates.
    // fill in each with input image coordinate
    for(epicsUInt16 x=0; x<M.sizex(); x++) {
        for(epicsUInt16 y=0; y<M.sizey(); y++) {
            double xc = x-center[0];
            double yc = y-center[1];

            // rotation about center point (Z axis)
            double temp = xc;
            xc =  xc*cosa - yc*sina;
            yc =  temp*sina + yc*cosa;

            // "keystone" correction (rotation about X and/or Y axis)
            temp = xc;
            xc += xc*xc*factor[0]   + xc*yc*factor[1];
            yc += yc*temp*factor[0] + yc*yc*factor[1];

            M.x(x,y) = xc+center[0];
            M.y(x,y) = yc+center[1];
        }
    }
}

extern "C" int NDWarpConfigure(const char *portName, int queueSize, int blockingCallbacks,
                                const char *NDArrayPort, int NDArrayAddr,
                                int maxBuffers, size_t maxMemory)
{
    return (new NDPluginWarp(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr,
                     maxBuffers, maxMemory, 0, 2000000))->start();
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
