#ifndef NDUTIL_H
#define NDUTIL_H

#include <stdexcept>

#include <epicsGuard.h>

#include "NDPluginDriver.h"

struct ndarray_ptr
{
    typedef NDArray value_type;

    struct borrow {};

    ndarray_ptr() :ptr(NULL) {}
    ndarray_ptr(const ndarray_ptr& o) :ptr(o.ptr) { p_inc(ptr); }
    ndarray_ptr(NDArray *p) : ptr(p) {
        if(!p) throw std::runtime_error("NDArray alloc fails");
    }
    ndarray_ptr(NDArray *p, borrow) : ptr(p) {
        if(!p) throw std::runtime_error("NDArray alloc fails");
        p_inc(ptr);
    }

    ~ndarray_ptr() {
        p_dec(ptr);
    }

    ndarray_ptr& operator=(const ndarray_ptr& o) {
        if(this!=&o) {
            ptr = o.ptr;
            p_inc(ptr);
        }
        return *this;
    }

    void swap(ndarray_ptr& o) {
        std::swap(ptr, o.ptr);
    }

    void reset() {
        p_dec(ptr);
        ptr = 0;
    }

    void reset(NDArray *p) {
        if(!p) throw std::runtime_error("NDArray alloc fails");
        try {
            p_dec(ptr);
            ptr = p;
        } catch(...) {
            // hmmm... what to do here?
            // error was with old array, which should no longer be accessed.
            // assume release() prints something and continue...
            ptr = p;
        }
    }

    void reset(NDArray *p, borrow) {
        if(!p) throw std::runtime_error("NDArray alloc fails");
        p_inc(p);
        try {
            p_dec(ptr);
            ptr = p;
        } catch(...) {
            // hmmm... what to do here?
            // error was with old array, which should no longer be accessed.
            // assume release() prints something and continue...
            ptr = p;
        }
    }

    NDArray* release() {
        NDArray *ret = ptr;
        ptr = 0;
        return ret;
    }

    struct Inplace {
        ndarray_ptr& owner;
        NDArray *ptr;
        Inplace(ndarray_ptr& o) :owner(o), ptr(0) {}
        ~Inplace() {
            owner.reset(ptr);
        }
        operator NDArray**() { return &ptr; }
    };

    // use with calls which "return" using a double pointer
    // The Inplace temp object collects the new pointer
    // and installs it when the temp. object is destroyed
    inline Inplace inplace() { return Inplace(*this); }

    NDArray& operator*() const { return *ptr; }
    NDArray* operator->() const { return ptr; }

    NDArray* get() const { return ptr; }

private:
    NDArray *ptr;

    static void p_inc(NDArray *p) {
        if(p && p->reserve()!=ND_SUCCESS)
            throw std::logic_error("ND ref counter inc error");
    }
    static void p_dec(NDArray *p) {
        if(p && p->release()!=ND_SUCCESS)
            throw std::logic_error("ND ref counter dec error");
    }
};

struct aPDUnlock {
    asynPortDriver& self;
    aPDUnlock(asynPortDriver& s) : self(s) {
        self.unlock();
    }
    ~aPDUnlock() {
        self.lock();
    }
};

typedef epicsGuard<asynPortDriver> aPDLock;

// allow uninitialized array of same type and shape as 'proto'
inline
NDArray* cloneArray(NDArrayPool *pool, NDArray *proto)
{
    size_t dims[ND_ARRAY_MAX_DIMS];
    for(int d=0; d<proto->ndims; d++) {
        dims[d] = proto->dims[d].size;
    }
    return pool->alloc(proto->ndims, dims, proto->dataType, proto->dataSize, NULL);
}

#endif // NDUTIL_H
