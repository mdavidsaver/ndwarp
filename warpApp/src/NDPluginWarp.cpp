/*
 * This software is Copyright by the Board of Trustees of Michigan
 * State University (c) Copyright 2016.
 *
 * Author: Michael Davidsaver <mdavidsaver@gmail.com>
 * 
 * #############################################################
 * #############################################################
 * 
 * Contribution RGB and AutoResize 
 * Author: Guilherme Rodrigues de Lima
 * Email: guilherme.lima@lnls.br 
 * Date: 06/22/2023
 */

#include <algorithm>
#include <limits>

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
            && lhs.colorSize==rhs.colorSize
            && lhs.colorMode==rhs.colorMode;
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

    createParam(NDWarpAutoResizeString, asynParamInt32, &NDWarpAutoResize); // Auto Resize Mode

    setStringParam(NDPluginDriverPluginType, "NDPluginWarp");

    setDoubleParam(NDWarpFactorX, 0.0); // initialize w/ no-op
    setDoubleParam(NDWarpFactorY, 0.0);
    setDoubleParam(NDWarpAngle, 0.0);
    setIntegerParam(NDWarpCenterX, 0);
    setIntegerParam(NDWarpCenterY, 0);
    setIntegerParam(NDWarpAutoResize, 0);
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

    if(info.colorMode == 1 || info.colorMode > 4 || info.xSize==0 || info.ySize==0) {
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

        int autoresize=0;
        getIntegerParam(NDWarpAutoResize, &autoresize);
        
        if(!sameShape(info, lastinfo) || lastautoresize != autoresize) {
            asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: input shape changes, recompute mapping\n", this->portName);

            lastinfo = info;
            lastautoresize = autoresize;
            recalculate_transform(info);

            assert(mapping.size() == samp_per_pixel*AutoResize.nElements);
        }

        int outputmode=0;
        getIntegerParam(NDWarpOutput, &outputmode);

        ndarray_ptr output;

        if(outputmode==0) {
            // output transformed image

            //output.reset(cloneArray(pNDArrayPool, pArray));

            size_t dims[ND_ARRAY_MAX_DIMS];
            if (lastinfo.colorMode == 0) {
                dims[0] = AutoResize.xSize;
                dims[1] = AutoResize.ySize;
            } 
            else if (lastinfo.colorMode == 2) {
                dims[0] = 3; 
                dims[1] = AutoResize.xSize;
                dims[2] = AutoResize.ySize;
            }
            else if (lastinfo.colorMode == 3) {
                dims[0] = AutoResize.xSize;
                dims[1] = 3; 
                dims[2] = AutoResize.ySize;
            } 
            else if (lastinfo.colorMode == 4) {
                dims[0] = AutoResize.xSize;
                dims[1] = AutoResize.ySize;
                dims[2] = 3; 
            }

            output.reset(pNDArrayPool->alloc(pArray->ndims, dims, pArray->dataType, 0, NULL));

            switch(pArray->dataType) {
#define CASE(TYPE) case ND ## TYPE: warpit<epics ## TYPE>(pArray, output.get(), &mapping[0], AutoResize.nElements, samp_per_pixel); break;
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
                for(size_t ox=0; ox<lastmap.sizex(); ox++) {
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

void NDPluginWarp::auto_resize(double a) {

    AutoResize.xSize = round(cos(abs(a)*PI_180)*lastinfo.xSize + sin(abs(a)*PI_180)*lastinfo.ySize);
    AutoResize.ySize = round(cos(abs(a)*PI_180)*lastinfo.ySize + sin(abs(a)*PI_180)*lastinfo.xSize);

    switch(lastinfo.colorMode) {
        case 0: {
            AutoResize.nElements = AutoResize.xSize * AutoResize.ySize;
            AutoResize.xStride = 1;
            AutoResize.yStride = AutoResize.xSize;
        } break;
        case 2: {
            AutoResize.nElements = AutoResize.xSize * AutoResize.ySize * 3;
            AutoResize.xStride = 3;
            AutoResize.yStride = AutoResize.xSize*3;
        } break;
        case 3: {
            AutoResize.nElements = AutoResize.xSize * AutoResize.ySize * 3;
            AutoResize.xStride = 1;
            AutoResize.yStride = AutoResize.xSize*3;
        } break;
        case 4: {
            AutoResize.nElements = AutoResize.xSize * AutoResize.ySize * 3;
            AutoResize.xStride = 1;
            AutoResize.yStride = AutoResize.xSize;
        } break;
    }
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

    getDoubleParam(NDWarpAngle, &angle);

    if (lastautoresize == 1) {
        auto_resize(angle);
    } else {
        auto_resize(0);
    }

    mapping.resize(samp_per_pixel*AutoResize.nElements);

    Mapping M;
    M.resize(AutoResize.xSize,AutoResize.ySize);

    fill_mapping(M);

    // sanitize mapping
    // replace outside range [0, size-1] with NaN
    for(size_t j=0; j<M.sizey(); j++) {
        for(size_t i=0; i<M.sizex(); i++) {
            double x=M.x(i,j), y=M.y(i,j);
            if(x>lastinfo.xSize-1 || x<0.0 || !isfinite(x)) {
                M.x(i,j) = std::numeric_limits<double>::quiet_NaN();
            }
            if(y>lastinfo.ySize-1 || y<0.0 || !isfinite(y)) {
                M.y(i,j) = std::numeric_limits<double>::quiet_NaN();
            }
        }
    }
    // interpolate and collapse user 2D Mapping to 1D 'mapping' (from offset to offset)

    Sample * const S = &mapping[0];

    switch(cur_mode) {
    case Nearest: {
        assert(samp_per_pixel==1);
        for(size_t j=0; j<M.sizey(); j++) {
            for(size_t i=0; i<M.sizex(); i++) {

                if (lastinfo.colorMode == 0) {
                    const size_t Ooffset = i*AutoResize.xStride + j*AutoResize.yStride;
                    Sample * const SX = &S[Ooffset];
                    double x=round(M.x(i,j)), y=round(M.y(i,j));
                    const size_t Ioffset = x*lastinfo.xStride + y*lastinfo.yStride;
                    SX->weight = 1.0;
                    SX->index = Ioffset;
                    SX->valid = isfinite(x) && isfinite(y);

                }
                else {
                    for(size_t k=0; k<3; k++) {
                        size_t Ooffset;
                        size_t Ioffset;
                        double x=round(M.x(i,j)), y=round(M.y(i,j));
                        switch(lastinfo.colorMode) {
                            case 2: {
                                Ooffset = i*AutoResize.xStride + j*AutoResize.yStride + k;
                                Ioffset = x*lastinfo.xStride+y*lastinfo.yStride + k;
                            } break;
                            case 3: {
                                Ooffset = i*AutoResize.xStride + j*AutoResize.yStride + k*AutoResize.xSize;
                                Ioffset = x*lastinfo.xStride + y*lastinfo.yStride + k*lastinfo.xSize;
                            } break;
                            case 4: {
                                Ooffset = i*AutoResize.xStride + j*AutoResize.yStride + k*AutoResize.xSize*AutoResize.ySize;
                                Ioffset = x*lastinfo.xStride+y*lastinfo.yStride + k*lastinfo.xSize*lastinfo.ySize;
                            } break;
                        }
                        Sample * const SX = &S[Ooffset];
                        SX->weight = 1.0;
                        SX->index = Ioffset;
                        SX->valid = isfinite(x) && isfinite(y);
                    }
                }
            }
        }
    }
        break;

    case Bilinear: {
        assert(samp_per_pixel==4);
        for(size_t j=0; j<M.sizey(); j++) {
            for(size_t i=0; i<M.sizex(); i++) {

                if (lastinfo.colorMode == 0) {
                    const size_t Ooffset = 4*(i*AutoResize.xStride + j*AutoResize.yStride);
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

                else {
                    for(size_t k=0; k<3; k++) {
                        size_t Ooffset;
                        size_t Ioffset[4];

                        double x=M.x(i,j), y=M.y(i,j),
                            fx=floor(x), fy=floor(y),
                            cx=ceil(x), cy=ceil(y),
                            xfact = cx==fx ? 0.0 : (x-fx)/(cx-fx),
                            yfact = cy==fy ? 0.0 : (y-fy)/(cy-fy);
                        bool valid = isfinite(x) && isfinite(y);

                        switch(lastinfo.colorMode) {
                            case 2: {
                                Ooffset = 4*(i*AutoResize.xStride + j*AutoResize.yStride + k);
                                Ioffset[0]  = fx*lastinfo.xStride + fy*lastinfo.yStride + k;
                                Ioffset[1]  = cx*lastinfo.xStride + fy*lastinfo.yStride + k;
                                Ioffset[2]  = fx*lastinfo.xStride + cy*lastinfo.yStride + k;
                                Ioffset[3]  = cx*lastinfo.xStride + cy*lastinfo.yStride + k;
                            } break;
                            case 3: {
                                Ooffset = 4*(i*AutoResize.xStride + j*AutoResize.yStride + k*AutoResize.xSize);
                                Ioffset[0]  = fx*lastinfo.xStride + fy*lastinfo.yStride + k*lastinfo.xSize;
                                Ioffset[1]  = cx*lastinfo.xStride + fy*lastinfo.yStride + k*lastinfo.xSize;
                                Ioffset[2]  = fx*lastinfo.xStride + cy*lastinfo.yStride + k*lastinfo.xSize;
                                Ioffset[3]  = cx*lastinfo.xStride + cy*lastinfo.yStride + k*lastinfo.xSize;
                            } break;
                            case 4: {
                                Ooffset = 4*(i*AutoResize.xStride + j*AutoResize.yStride + k*AutoResize.xSize*AutoResize.ySize);
                                Ioffset[0]  = fx*lastinfo.xStride + fy*lastinfo.yStride + k*lastinfo.xSize*lastinfo.ySize;
                                Ioffset[1]  = cx*lastinfo.xStride + fy*lastinfo.yStride + k*lastinfo.xSize*lastinfo.ySize;
                                Ioffset[2]  = fx*lastinfo.xStride + cy*lastinfo.yStride + k*lastinfo.xSize*lastinfo.ySize;
                                Ioffset[3]  = cx*lastinfo.xStride + cy*lastinfo.yStride + k*lastinfo.xSize*lastinfo.ySize;
                            } break;
                        }
                        
                        Sample * const SX = &S[Ooffset];
                        SX[0].weight = (1.0-xfact)*(1.0-yfact);
                        SX[0].index  = Ioffset[0];
                        SX[0].valid  = valid;
                        SX[1].weight = xfact*(1.0-yfact);
                        SX[1].index  = Ioffset[1];
                        SX[1].valid  = valid;
                        SX[2].weight = (1.0-xfact)*yfact;
                        SX[2].index  = Ioffset[2];
                        SX[2].valid  = valid;
                        SX[3].weight = xfact*yfact;
                        SX[3].index  = Ioffset[3];
                        SX[3].valid  = valid;
                    }
                }
            }
        }
    }
        break;
    }

    lastmap.swap(M);
}

void NDPluginWarp::fill_mapping(Mapping &M)
{
    double factor[2] = {0.0, 0.0};
    int center[2] = {0, 0};

    getDoubleParam(NDWarpFactorX, &factor[0]);
    getDoubleParam(NDWarpFactorY, &factor[1]);

    if (lastautoresize == 0) {
        getIntegerParam(NDWarpCenterX, &center[0]);
        getIntegerParam(NDWarpCenterY, &center[1]);
    } else {
        center[0] = AutoResize.xSize/2;
        center[1] = AutoResize.ySize/2;
    }

//    printf("# fill_mapping. center: [%d, %d] angle: %f F: [%f, %f]\n",
//           center[0], center[1], angle, factor[0], factor[1]);

    const double sina=sin(angle*PI_180), cosa=cos(angle*PI_180);

    // iterate through output image coordinates.
    // fill in each with input image coordinate
    for(epicsUInt16 y=0; y<M.sizey(); y++) {
        for(epicsUInt16 x=0; x<M.sizex(); x++) {
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

            M.x(x,y) = xc+center[0] - (AutoResize.xSize-lastinfo.xSize)/2;
            M.y(x,y) = yc+center[1] - (AutoResize.ySize-lastinfo.ySize)/2;
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
