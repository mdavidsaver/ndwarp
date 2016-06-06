#ifndef NDPLUGINWARP_H
#define NDPLUGINWARP_H

#include <memory>
#include <vector>

#include <epicsTypes.h>

#include "NDPluginDriver.h"

#include "ndutil.h"

/* Param definitions */
#define NDWarpRunTimeString    "WARP_RUN_TIME"
#define NDWarpAxisString       "WARP_AXIS"
#define NDWarpFactorString     "WARP_FACTOR"
#define NDWarpCenterXString    "WARP_CENTER_X"
#define NDWarpCenterYString    "WARP_CENTER_Y"
#define NDWarpModeString       "WARP_INTERP_MODE"

/** Generic coordinate transformation
 *
 * Holds a mapping from pixel in output image to a weighted sum
 * of some number of input pixels.
 *
 * The number of input pixels per output pixel depends on the interpolation mode.
 * Nearest neighbor has 1.  Bilinear has 4.
 */
class epicsShareClass NDPluginWarp : public NDPluginDriver {
public:
    NDPluginWarp(const char *portName, int queueSize, int blockingCallbacks,
                 const char *NDArrayPort, int NDArrayAddr,
                 int maxBuffers, size_t maxMemory,
                 int priority, int stackSize);
    virtual ~NDPluginWarp();

    virtual void processCallbacks(NDArray *pArray);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);


    struct Mapping {
        Mapping() :yStride(0) {}
        void resize(const NDArrayInfo_t& info) {
            map.resize(2*info.xSize*info.ySize);
#ifndef NDEBUG
            std::fill(map.begin(), map.end(), 0);
#endif
            yStride = info.xSize;
        }
        double& x(size_t ix, size_t iy) {
            return map[2*(ix+iy*yStride)];
        }
        double& y(size_t ix, size_t iy) {
            return map[2*(ix+iy*yStride)+1];
        }

        size_t sizex() const { return yStride; }
        size_t sizey() const { return map.size()/yStride/2; }

        size_t yStride;
        std::vector<double> map;
    };

    struct Sample {
        double weight;
        size_t index;
        Sample() :weight(1.0), index(0) {}
    };

    void recalculate_transform(const NDArrayInfo &info);
    void fill_mapping(Mapping &M);

    unsigned samp_per_pixel;
    typedef std::vector<Sample> mapping_t;
    mapping_t mapping; // size() is samp_per_pixel*listinfo.nElements

    NDArrayInfo lastinfo;
    enum mode_t {
        Nearest, Bilinear,
    } cur_mode;

    int NDWarpRunTime;
#define FIRST_NDPLUGIN_WARP_PARAM NDWarpRunTime
    int NDWarpMode;
    int NDWarpAxis;
    int NDWarpFactor;
    int NDWarpCenterX;
    int NDWarpCenterY;

#define LAST_NDPLUGIN_WARP_PARAM NDWarpCenterY
#define NUM_NDPLUGIN_WARP_PARAMS ((int)(&LAST_NDPLUGIN_WARP_PARAM - &FIRST_NDPLUGIN_WARP_PARAM + 1))

    // the following may only be accessed from processCallbacks()
};

bool operator==(const NDArrayInfo&, const NDArrayInfo&);
inline bool operator!=(const NDArrayInfo& lhs, const NDArrayInfo& rhs)
{ return !(lhs==rhs); }

#endif // NDPLUGINWARP_H
