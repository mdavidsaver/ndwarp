/*
 * This software is Copyright by the Board of Trustees of Michigan
 * State University (c) Copyright 2016.
 *
 * Author: Michael Davidsaver <mdavidsaver@gmail.com>
 */
#include <sstream>

#include <epicsUnitTest.h>
#include <testMain.h>
#include <dbDefs.h>

#include <asynPortClient.h>

#include "NDPluginWarp.h"

namespace {

template<typename LHS, typename RHS>
void testEqualx(const char *nlhs, const char *nrhs, const LHS& lhs, const RHS& rhs)
{
    bool result = lhs==rhs;
    std::ostringstream strm;
    strm<<nlhs<<" ("<<lhs<<") == "<<nrhs<<" ("<<rhs<<")";
    testOk(result, "%s", strm.str().c_str());
}
#define testEqual(LHS, RHS) testEqualx(#LHS, #RHS, LHS, RHS)

struct ArrayCapture : public asynGenericPointerClient
{
    ArrayCapture(const char *port)
        :asynGenericPointerClient(port, 0, NDArrayDataString)
    {
        if(registerInterruptUser(&cb)!=asynSuccess)
            throw std::runtime_error("ArrayCapture failed to register interrupt");
    }
    virtual ~ArrayCapture() {}
    static void cb(void *userPvt, asynUser *pasynUser, void *pointer)
    {
        testDiag("%s: capture %p", __PRETTY_FUNCTION__, pointer);
        ArrayCapture *self=(ArrayCapture*)userPvt;
        NDArray *arr=(NDArray*)pointer;
        self->last.reset(arr, ndarray_ptr::borrow());
    }
    ndarray_ptr last;
};

template<typename T>
void showArray(NDArray* A)
{
    const T* const base = (const T*)A->pData;
    NDArrayInfo_t info;
    A->getInfo(&info);
    if(info.colorMode!=NDColorModeMono || info.bytesPerElement!=sizeof(T)) {
        testDiag("Can't print array, %u %u", (unsigned)info.colorMode, (unsigned)info.bytesPerElement);
        return;
    }
    testDiag("(%u, %u)[", (unsigned)info.xSize, (unsigned)info.ySize);

    for(size_t i=0; i<info.xSize; i++) {
        printf("# [");
        for(size_t j=0; j<info.ySize; j++) {
            size_t offset = i*info.xStride + j*info.yStride;
            printf("%u, ", base[offset]);
        }
        printf("],\n");
    }
    printf("# ]\n");
}


template<typename T>
void arrcmp(const T* lhs, size_t lhs_count, const T* rhs, size_t rhs_count)
{
    bool result = lhs_count==rhs_count;
    size_t N = std::min(lhs_count, rhs_count);

    for(size_t i=0; i<N; i++)
        result &= lhs[i]==rhs[i];

    testOk(result, "arrays are equal");
    if(lhs_count!=rhs_count)
        testDiag("Length mis-match %u != %u", (unsigned)lhs_count, (unsigned)rhs_count);

    if(result) return;

    for(size_t i=0; i<N; i++) {
        if(lhs[i]==rhs[i]) continue;
        std::ostringstream strm;
        strm<<lhs[i]<<" != "<<rhs[i];
        testDiag("[%u] %s", (unsigned)i, strm.str().c_str());
    }
}

void testRotate()
{
    static const std::string name(__FUNCTION__);
    static const epicsUInt16 input[9] = {
        1, 2, 3,
        4, 5, 6,
        7, 8, 9
    };
    static const epicsUInt16 expect90[9] = {
        3, 6, 9,
        2, 5, 8,
        1, 4, 7
    };
    static const epicsUInt16 expect45[9] = {
        0, 3, 0,
        1, 5, 9,
        0, 7, 0
    };

    testDiag("In %s", __FUNCTION__);
    NDPluginWarp *P = new NDPluginWarp((name+"warp").c_str(), 2, 1, "", 0, 10, 0, 0, 0);
    ArrayCapture L(P->portName);

    {
        asynInt32Client CX(P->portName, 0, NDWarpCenterXString),
                        CY(P->portName, 0, NDWarpCenterYString),
                        CB(P->portName, 0, NDArrayCallbacksString);

        CX.write(1);
        CY.write(1);
        CB.write(1);
    }

    size_t dim[2] = {3, 3};

    NDArrayPool *pool = new NDArrayPool(P, 0);
    // don't bother to delete 'pool', it doesn't cleanup after itself anyway (no dtor)

    ndarray_ptr inp(pool->alloc(2, dim, NDUInt16, 0, NULL));
    memcpy(inp->pData, input, sizeof(input));

    testDiag("Input array");
    showArray<epicsUInt16>(inp.get());

    testDiag("no-op transform");
    {
        aPDLock G(*P);
        P->processCallbacks(inp.get());
    }

    {
        double val;
        P->getDoubleParam(P->NDWarpRunTime, &val);
        testDiag("Run in %g sec.", val);
    }

    testOk(!!L.last.get(), "Result %p", L.last.get());
    if(!L.last.get()) {
        testSkip(5, "No output");
    } else {
        NDArray *A = L.last.get();
        testDiag("Output array");
        showArray<epicsUInt16>(A);

        NDArrayInfo_t info;
        A->getInfo(&info);
        testEqual(A->ndims, 2);
        testEqual(A->dims[0].size, 3u);
        testEqual(A->dims[1].size, 3u);
        testEqual(A->dataType, NDUInt16);
        arrcmp(input, NELEMENTS(input), (epicsUInt16*)A->pData, info.nElements);
    }

    testDiag("Rotate 90 deg. transform");
    {
        asynFloat64Client M(P->portName, 0, NDWarpAngleString);
        M.write(90.0);
    }

    {
        aPDLock G(*P);
        P->processCallbacks(inp.get());
    }

    testOk(!!L.last.get(), "Result %p", L.last.get());
    if(!L.last.get()) {
        testSkip(5, "No output");
    } else {
        NDArray *A = L.last.get();
        testDiag("Output array");
        showArray<epicsUInt16>(A);

        NDArrayInfo_t info;
        A->getInfo(&info);
        testEqual(A->ndims, 2);
        testEqual(A->dims[0].size, 3u);
        testEqual(A->dims[1].size, 3u);
        testEqual(A->dataType, NDUInt16);
        arrcmp(expect90, NELEMENTS(expect90), (epicsUInt16*)A->pData, info.nElements);
    }

    testDiag("Rotate 45 deg. transform");
    {
        asynFloat64Client M(P->portName, 0, NDWarpAngleString);
        M.write(45.0);
    }

    {
        aPDLock G(*P);
        P->processCallbacks(inp.get());
    }

    testOk(!!L.last.get(), "Result %p", L.last.get());
    if(!L.last.get()) {
        testSkip(5, "No output");
    } else {
        NDArray *A = L.last.get();
        testDiag("Output array");
        showArray<epicsUInt16>(A);

        NDArrayInfo_t info;
        A->getInfo(&info);
        testEqual(A->ndims, 2);
        testEqual(A->dims[0].size, 3u);
        testEqual(A->dims[1].size, 3u);
        testEqual(A->dataType, NDUInt16);
        arrcmp(expect45, NELEMENTS(expect45), (epicsUInt16*)A->pData, info.nElements);
    }
}

} // namespace

MAIN(testwarp)
{
    testPlan(18);
    testRotate();
    return testDone();
}
