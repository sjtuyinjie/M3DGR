// This file is modified from lkpyramid.hpp of openCV
#pragma once

// c++
#include <iostream>
#include <math.h>

// opencv
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#define CV_CPU_HAS_SUPPORT_SSE2 1
#define USING_OPENCV_TBB 1
#include <opencv2/core/hal/intrin.hpp>
#include <numeric>
#include <future>

enum
{
    cv_OPTFLOW_USE_INITIAL_FLOW = 4,
    cv_OPTFLOW_LK_GET_MIN_EIGENVALS = 8,
    cv_OPTFLOW_FARNEBACK_GAUSSIAN = 256
};

typedef short deriv_type;

inline int opencvBuildOpticalFlowPyramid(cv::InputArray img, cv::OutputArrayOfArrays pyramid, cv::Size winSize, int maxLevel, bool withDerivatives = true,
    int pyrBorder = cv::BORDER_REFLECT_101, int derivBorder = cv::BORDER_CONSTANT, bool tryReuseInputImage = true);

inline void calcSharrDeriv(const cv::Mat &src, cv::Mat &dst);

void calculateOpticalFlow(cv::InputArray prevImg_, cv::InputArray nextImg_, cv::InputArray prevPts_, cv::InputOutputArray nextPts_, cv::OutputArray status_, 
    cv::OutputArray err_, cv::Size winSize = cv::Size(21, 21), int maxLevel = 3,
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
    int flags = 0, double minEigThreshold = 1e-4);

inline void calculateLKOpticalFlow(const cv::Range &range, const cv::Mat *prevImg, const cv::Mat *prevDeriv, const cv::Mat *nextImg, const cv::Point2f *prevPts, 
    cv::Point2f *nextPts, uchar *status, float *err, cv::Size winSize, cv::TermCriteria criteria, int level, int maxLevel, int flags, float minEigThreshold);

struct opencvLKTrackerInvoker : cv::ParallelLoopBody
{
    opencvLKTrackerInvoker(const cv::Mat *prevImg_, const cv::Mat *prevDeriv_, const cv::Mat *nextImg_, const cv::Point2f *prevPts_, cv::Point2f *nextPts_, 
        uchar *status_, float *err_, cv::Size winSize_, cv::TermCriteria criteria_, int level_, int maxLevel_, int flags_, float minEigThreshold_);

    void operator()(const cv::Range &range) const;

    bool calculate( cv::Range range) const;

    const cv::Mat *prevImg;
    const cv::Mat *nextImg;
    const cv::Mat *prevDeriv;
    const cv::Point2f *prevPts;
    cv::Point2f *nextPts;
    uchar *status;
    float *err;
    cv::Size winSize;
    cv::TermCriteria criteria;
    int level;
    int maxLevel;
    int flags;
    float minEigThreshold;
};

class LKOpticalFlowKernel
{
private:
    std::vector<cv::Mat> prev_img_pyr, curr_img_pyr;
    std::vector<cv::Mat> prev_img_deriv_I, prev_img_deriv_I_buff;
    std::vector<cv::Mat> curr_img_deriv_I, curr_img_deriv_I_buff;

    cv::Size lk_win_size;
    int maxLevel;
    cv::TermCriteria terminate_criteria;

    int flags;
    double minEigThreshold;

public:
    cv::Size getWinSize() const { return lk_win_size; }

    void setWinSize(cv::Size winSize_) { lk_win_size = winSize_; }

    int getMaxLevel() const { return maxLevel; }

    void setMaxLevel(int maxLevel_) { maxLevel = maxLevel_; }

    cv::TermCriteria getTermCriteria() const { return terminate_criteria; }

    void setTermCriteria(cv::TermCriteria &crit_) { terminate_criteria = crit_; }

    int getFlags() const { return flags; }

    void setFlags(int flags_) { flags = flags_; }

    double getMinEigThreshold() const { return minEigThreshold; }

    void setMinEigThreshold(double minEigThreshold_) { minEigThreshold = minEigThreshold_; }

    LKOpticalFlowKernel(cv::Size winSize_ = cv::Size(21, 21), int maxLevel_ = 3,
        cv::TermCriteria criteria_ = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
        int flags_ = 0, double minEigThreshold_ = 1e-4) 
        : lk_win_size(winSize_), maxLevel(maxLevel_), terminate_criteria(criteria_), flags(flags_), minEigThreshold(minEigThreshold_)
    {
        setTerminationCriteria(terminate_criteria);
    }

    /**
    @brief Calculates a sparse optical flow.
    @param prevImg First input image.
    @param nextImg Second input image of the same cv::Size and the same type as prevImg.
    @param prevPts Vector of 2D points for which the flow needs to be found.
    @param nextPts Output vector of 2D points containing the calculated new positions of input features in the second image.
    @param status Output status vector. Each element of the vector is set to 1 if the
                  flow for the corresponding features has been found. Otherwise, it is set to 0.
    @param err Optional output vector that contains error response for each point (inverse confidence).
    **/
    void calc(cv::InputArray prevImg_, cv::InputArray nextImg_, cv::InputArray prevPts_, cv::InputOutputArray nextPts_, 
        cv::OutputArray status_, cv::OutputArray err_ = cv::noArray());                        

    void allocateImgDerivMemory(std::vector<cv::Mat> &img_pyr, std::vector<cv::Mat> &img_pyr_deriv_I, std::vector<cv::Mat> &img_pyr_deriv_I_buff);

    void calcImageDerivSharr(std::vector<cv::Mat> &img_pyr, std::vector<cv::Mat> &img_pyr_deriv_I, std::vector<cv::Mat> &img_pyr_deriv_I_buff);

    void setTerminationCriteria(cv::TermCriteria &crit);

    void swapImageBuffer();

    int trackImage(const cv::Mat & curr_img, const std::vector<cv::Point2f> & last_tracked_pts, std::vector<cv::Point2f> & curr_tracked_pts, 
        std::vector<uchar> & status, int opm_method = 3 ); // opm_method: [0] openCV parallel_body [1] openCV parallel for [2] Thread pool
};