

#ifndef CAM_CAM_WRAPPER_H
#define CAM_CAM_WRAPPER_H

// packages
#include <opencv2/core/core.hpp>

/**
 * @brief A virtual class for wrapper of camera and video files
 */
class WrapperHead
{
public:
    virtual ~WrapperHead() = default;
    ;
    virtual bool init() = 0;
    virtual bool read(cv::Mat &src) = 0;
    virtual bool setBrightness(int brightness) = 0;
    virtual int getFps() = 0;
    virtual cv::Size getSize() = 0;
    virtual bool close() = 0;
};

// submodules
#include <video/video_wrapper.hpp>
#include <hikcam/hikcam_wrapper.hpp>
#endif // CAM_CAM_WRAPPER_H
