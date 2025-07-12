/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include <float.h>
#include <stdio.h>
#include "lkpyramid.h"
#include <omp.h>

#define CV_DESCALE(x, n) (((x) + (1 << ((n)-1))) >> (n))

using namespace cv;
using std::cout;
using std::endl;

typedef float acctype;
typedef float itemtype;

inline void calcSharrDeriv(const cv::Mat &src, cv::Mat &dst)
{
    // printf_line;
    int rows = src.rows;
    int cols = src.cols;
    int cn = src.channels();
    int colsn = cols * cn;
    int depth = src.depth();

    CV_Assert(depth == CV_8U);

    if (dst.rows != rows || dst.cols != cols)
    {
        dst.create(rows, cols, CV_MAKETYPE(DataType<deriv_type>::depth, colsn * 2));
    }

    int x, y;
    int delta = (int)alignSize((cols + 2) * cn, 16);

    AutoBuffer<deriv_type> tempBuf(delta * 2 + 64);
    deriv_type *trow0 = alignPtr(tempBuf + cn, 16);
    deriv_type *trow1 = alignPtr(trow0 + delta, 16);

    v_int16x8 c3 = v_setall_s16(3);
    v_int16x8 c10 = v_setall_s16(10);
    bool haveSIMD = checkHardwareSupport(CV_CPU_SSE2) || checkHardwareSupport(CV_CPU_NEON);

    for (y = 0; y < rows; y++)
    {
        const uchar *srow0 = src.ptr<uchar>(y > 0 ? y - 1 : rows > 1 ? 1 : 0);
        const uchar *srow1 = src.ptr<uchar>(y);
        const uchar *srow2 = src.ptr<uchar>(y < rows - 1 ? y + 1 : rows > 1 ? rows - 2 : 0);
        deriv_type *drow = dst.ptr<deriv_type>(y);

        x = 0;

        if (haveSIMD)
        {
            for (; x <= colsn - 8; x += 8)
            {
                v_int16x8 s0 = v_reinterpret_as_s16(v_load_expand(srow0 + x));
                v_int16x8 s1 = v_reinterpret_as_s16(v_load_expand(srow1 + x));
                v_int16x8 s2 = v_reinterpret_as_s16(v_load_expand(srow2 + x));

                v_int16x8 t1 = s2 - s0;
                v_int16x8 t0 = (s0 + s2) * c3 + s1 * c10;

                v_store(trow0 + x, t0);
                v_store(trow1 + x, t1);
            }
        }

        for (; x < colsn; x++)
        {
            int t0 = (srow0[x] + srow2[x]) * 3 + srow1[x] * 10;
            int t1 = srow2[x] - srow0[x];
            trow0[x] = (deriv_type)t0;
            trow1[x] = (deriv_type)t1;
        }

        int x0 = (cols > 1 ? 1 : 0) * cn, x1 = (cols > 1 ? cols - 2 : 0) * cn;

        for (int k = 0; k < cn; k++)
        {
            trow0[-cn + k] = trow0[x0 + k];
            trow0[colsn + k] = trow0[x1 + k];
            trow1[-cn + k] = trow1[x0 + k];
            trow1[colsn + k] = trow1[x1 + k];
        }

        x = 0;

        if (haveSIMD)
        {
            for (; x <= colsn - 8; x += 8)
            {
                v_int16x8 s0 = v_load(trow0 + x - cn);
                v_int16x8 s1 = v_load(trow0 + x + cn);
                v_int16x8 s2 = v_load(trow1 + x - cn);
                v_int16x8 s3 = v_load(trow1 + x);
                v_int16x8 s4 = v_load(trow1 + x + cn);

                v_int16x8 t0 = s1 - s0;
                v_int16x8 t1 = ((s2 + s4) * c3) + (s3 * c10);

                v_store_interleave((drow + x * 2), t0, t1);
            }
        }

        for (; x < colsn; x++)
        {
            deriv_type t0 = (deriv_type)(trow0[x + cn] - trow0[x - cn]);
            deriv_type t1 = (deriv_type)((trow1[x + cn] + trow1[x - cn]) * 3 + trow1[x] * 10);
            drow[x * 2] = t0;
            drow[x * 2 + 1] = t1;
        }
    }
}

opencvLKTrackerInvoker::opencvLKTrackerInvoker(const Mat *prevImg_, const Mat *prevDeriv_, const Mat *nextImg_, const Point2f *prevPts_, 
    Point2f *nextPts_, uchar *status_, float *err_, Size winSize_, TermCriteria criteria_, int level_, int maxLevel_, int flags_, float minEigThreshold_)
{
    prevImg = prevImg_;
    prevDeriv = prevDeriv_;
    nextImg = nextImg_;
    prevPts = prevPts_;
    nextPts = nextPts_;
    status = status_;
    err = err_;
    winSize = winSize_;
    criteria = criteria_;
    level = level_;
    maxLevel = maxLevel_;
    flags = flags_;
    minEigThreshold = minEigThreshold_;
}

inline void calculateLKOpticalFlow(const cv::Range &range, const Mat *prevImg, const Mat *prevDeriv, const Mat *nextImg, const Point2f *prevPts, 
    Point2f *nextPts, uchar *status, float *err, Size winSize, TermCriteria criteria, int level, int maxLevel, int flags, float minEigThreshold)
{
    Point2f halfWin((winSize.width - 1) * 0.5f, (winSize.height - 1) * 0.5f);

    const Mat &I = *prevImg;
    const Mat &J = *nextImg;
    const Mat &derivI = *prevDeriv;

    int j;
    int cn = I.channels();
    int cn2 = cn * 2;

    cv::AutoBuffer<deriv_type> buf(winSize.area() * (cn + cn2));
    int derivDepth = DataType<deriv_type>::depth;

    Mat IWinBuf(winSize, CV_MAKETYPE(derivDepth, cn), (deriv_type *)buf);
    Mat derivIWinBuf(winSize, CV_MAKETYPE(derivDepth, cn2), (deriv_type *)buf + winSize.area() * cn);

    for (int ptidx = range.start; ptidx < range.end; ptidx++)
    {
        Point2f prevPt = prevPts[ptidx] * (float)(1. / (1 << level));
        Point2f nextPt;

        if (level == maxLevel)
        {
            if (flags & OPTFLOW_USE_INITIAL_FLOW)
                nextPt = nextPts[ptidx] * (float)(1. / (1 << level));
            else
                nextPt = prevPt;
        }
        else
            nextPt = nextPts[ptidx] * 2.f;

        nextPts[ptidx] = nextPt;

        Point2i iprevPt, inextPt;
        prevPt -= halfWin;
        iprevPt.x = cvFloor(prevPt.x);
        iprevPt.y = cvFloor(prevPt.y);

        if (iprevPt.x < -winSize.width || iprevPt.x >= derivI.cols ||
            iprevPt.y < -winSize.height || iprevPt.y >= derivI.rows)
        {
            if (level == 0)
            {
                if (status)
                    status[ptidx] = false;
                if (err)
                    err[ptidx] = 0;
            }
            continue;
        }

        float a = prevPt.x - iprevPt.x;
        float b = prevPt.y - iprevPt.y;

        const int W_BITS = 14, W_BITS1 = 14;
        const float FLT_SCALE = 1.f / (1 << 20);

        int iw00 = cvRound((1.f - a) * (1.f - b) * (1 << W_BITS));
        int iw01 = cvRound(a * (1.f - b) * (1 << W_BITS));
        int iw10 = cvRound((1.f - a) * b * (1 << W_BITS));
        int iw11 = (1 << W_BITS) - iw00 - iw01 - iw10;

        int dstep = (int)(derivI.step / derivI.elemSize1());
        int stepI = (int)(I.step / I.elemSize1());
        int stepJ = (int)(J.step / J.elemSize1());

        acctype iA11 = 0, iA12 = 0, iA22 = 0;
        float A11, A12, A22;

        __m128i qw0 = _mm_set1_epi32(iw00 + (iw01 << 16));
        __m128i qw1 = _mm_set1_epi32(iw10 + (iw11 << 16));
        __m128i z = _mm_setzero_si128();
        __m128i qdelta_d = _mm_set1_epi32(1 << (W_BITS1 - 1));
        __m128i qdelta = _mm_set1_epi32(1 << (W_BITS1 - 5 - 1));
        __m128 qA11 = _mm_setzero_ps();
        __m128 qA12 = _mm_setzero_ps();
        __m128 qA22 = _mm_setzero_ps();

        // extract the patch from the first image, compute covariation matrix of derivatives
        int x, y;
        for (y = 0; y < winSize.height; y++)
        {
            const uchar *src = I.ptr() + (y + iprevPt.y) * stepI + iprevPt.x * cn;
            const deriv_type *dsrc = derivI.ptr<deriv_type>() + (y + iprevPt.y) * dstep + iprevPt.x * cn2;

            deriv_type *Iptr = IWinBuf.ptr<deriv_type>(y);
            deriv_type *dIptr = derivIWinBuf.ptr<deriv_type>(y);

            x = 0;

            for (; x <= winSize.width * cn - 4; x += 4, dsrc += 4 * 2, dIptr += 4 * 2)
            {
                __m128i v00, v01, v10, v11, t0, t1;

                v00 = _mm_unpacklo_epi8(_mm_cvtsi32_si128(*(const int *)(src + x)), z);
                v01 = _mm_unpacklo_epi8(_mm_cvtsi32_si128(*(const int *)(src + x + cn)), z);
                v10 = _mm_unpacklo_epi8(_mm_cvtsi32_si128(*(const int *)(src + x + stepI)), z);
                v11 = _mm_unpacklo_epi8(_mm_cvtsi32_si128(*(const int *)(src + x + stepI + cn)), z);

                t0 = _mm_add_epi32(_mm_madd_epi16(_mm_unpacklo_epi16(v00, v01), qw0), _mm_madd_epi16(_mm_unpacklo_epi16(v10, v11), qw1));
                t0 = _mm_srai_epi32(_mm_add_epi32(t0, qdelta), W_BITS1 - 5);
                _mm_storel_epi64((__m128i *)(Iptr + x), _mm_packs_epi32(t0, t0));

                v00 = _mm_loadu_si128((const __m128i *)(dsrc));
                v01 = _mm_loadu_si128((const __m128i *)(dsrc + cn2));
                v10 = _mm_loadu_si128((const __m128i *)(dsrc + dstep));
                v11 = _mm_loadu_si128((const __m128i *)(dsrc + dstep + cn2));

                t0 = _mm_add_epi32(_mm_madd_epi16(_mm_unpacklo_epi16(v00, v01), qw0), _mm_madd_epi16(_mm_unpacklo_epi16(v10, v11), qw1));
                t1 = _mm_add_epi32(_mm_madd_epi16(_mm_unpackhi_epi16(v00, v01), qw0), _mm_madd_epi16(_mm_unpackhi_epi16(v10, v11), qw1));

                t0 = _mm_srai_epi32(_mm_add_epi32(t0, qdelta_d), W_BITS1);
                t1 = _mm_srai_epi32(_mm_add_epi32(t1, qdelta_d), W_BITS1);
                v00 = _mm_packs_epi32(t0, t1); // Ix0 Iy0 Ix1 Iy1 ...

                _mm_storeu_si128((__m128i *)dIptr, v00);
                t0 = _mm_srai_epi32(v00, 16);                     // Iy0 Iy1 Iy2 Iy3
                t1 = _mm_srai_epi32(_mm_slli_epi32(v00, 16), 16); // Ix0 Ix1 Ix2 Ix3

                __m128 fy = _mm_cvtepi32_ps(t0);
                __m128 fx = _mm_cvtepi32_ps(t1);

                qA22 = _mm_add_ps(qA22, _mm_mul_ps(fy, fy));
                qA12 = _mm_add_ps(qA12, _mm_mul_ps(fx, fy));
                qA11 = _mm_add_ps(qA11, _mm_mul_ps(fx, fx));
            }

            for (; x < winSize.width * cn; x++, dsrc += 2, dIptr += 2)
            {
                int ival = CV_DESCALE(src[x] * iw00 + src[x + cn] * iw01 + src[x + stepI] * iw10 + src[x + stepI + cn] * iw11, W_BITS1 - 5);

                int ixval = CV_DESCALE(dsrc[0] * iw00 + dsrc[cn2] * iw01 + dsrc[dstep] * iw10 + dsrc[dstep + cn2] * iw11, W_BITS1);

                int iyval = CV_DESCALE(dsrc[1] * iw00 + dsrc[cn2 + 1] * iw01 + dsrc[dstep + 1] * iw10 + dsrc[dstep + cn2 + 1] * iw11, W_BITS1);

                Iptr[x] = (short)ival;
                dIptr[0] = (short)ixval;
                dIptr[1] = (short)iyval;

                iA11 += (itemtype)(ixval * ixval);
                iA12 += (itemtype)(ixval * iyval);
                iA22 += (itemtype)(iyval * iyval);
            }
        }

        float CV_DECL_ALIGNED(16) A11buf[4], A12buf[4], A22buf[4];

        _mm_store_ps(A11buf, qA11);
        _mm_store_ps(A12buf, qA12);
        _mm_store_ps(A22buf, qA22);

        iA11 += A11buf[0] + A11buf[1] + A11buf[2] + A11buf[3];
        iA12 += A12buf[0] + A12buf[1] + A12buf[2] + A12buf[3];
        iA22 += A22buf[0] + A22buf[1] + A22buf[2] + A22buf[3];

        A11 = iA11 * FLT_SCALE;
        A12 = iA12 * FLT_SCALE;
        A22 = iA22 * FLT_SCALE;

        float D = A11 * A22 - A12 * A12;
        float minEig = (A22 + A11 - std::sqrt((A11 - A22) * (A11 - A22) + 4.f * A12 * A12)) / (2 * winSize.width * winSize.height);

        if (err && (flags & OPTFLOW_LK_GET_MIN_EIGENVALS) != 0)
            err[ptidx] = (float)minEig;

        if (minEig < minEigThreshold || D < FLT_EPSILON)
        {
            if (level == 0 && status)
                status[ptidx] = false;

            continue;
        }

        D = 1.f / D;

        nextPt -= halfWin;
        Point2f prevDelta;

        for (j = 0; j < criteria.maxCount; j++)
        {
            inextPt.x = cvFloor(nextPt.x);
            inextPt.y = cvFloor(nextPt.y);

            if (inextPt.x < -winSize.width || inextPt.x >= J.cols || inextPt.y < -winSize.height || inextPt.y >= J.rows)
            {
                if (level == 0 && status)
                    status[ptidx] = false;
                break;
            }

            a = nextPt.x - inextPt.x;
            b = nextPt.y - inextPt.y;

            iw00 = cvRound((1.f - a) * (1.f - b) * (1 << W_BITS));
            iw01 = cvRound(a * (1.f - b) * (1 << W_BITS));
            iw10 = cvRound((1.f - a) * b * (1 << W_BITS));
            iw11 = (1 << W_BITS) - iw00 - iw01 - iw10;

            acctype ib1 = 0, ib2 = 0;
            float b1, b2;

            qw0 = _mm_set1_epi32(iw00 + (iw01 << 16));
            qw1 = _mm_set1_epi32(iw10 + (iw11 << 16));
            __m128 qb0 = _mm_setzero_ps(), qb1 = _mm_setzero_ps();

            for (y = 0; y < winSize.height; y++)
            {
                const uchar *Jptr = J.ptr() + (y + inextPt.y) * stepJ + inextPt.x * cn;
                const deriv_type *Iptr = IWinBuf.ptr<deriv_type>(y);
                const deriv_type *dIptr = derivIWinBuf.ptr<deriv_type>(y);

                x = 0;

                for (; x <= winSize.width * cn - 8; x += 8, dIptr += 8 * 2)
                {
                    __m128i diff0 = _mm_loadu_si128((const __m128i *)(Iptr + x)), diff1;
                    __m128i v00 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(Jptr + x)), z);
                    __m128i v01 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(Jptr + x + cn)), z);
                    __m128i v10 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(Jptr + x + stepJ)), z);
                    __m128i v11 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(Jptr + x + stepJ + cn)), z);

                    __m128i t0 = _mm_add_epi32(_mm_madd_epi16(_mm_unpacklo_epi16(v00, v01), qw0), _mm_madd_epi16(_mm_unpacklo_epi16(v10, v11), qw1));
                    __m128i t1 = _mm_add_epi32(_mm_madd_epi16(_mm_unpackhi_epi16(v00, v01), qw0), _mm_madd_epi16(_mm_unpackhi_epi16(v10, v11), qw1));

                    t0 = _mm_srai_epi32(_mm_add_epi32(t0, qdelta), W_BITS1 - 5);
                    t1 = _mm_srai_epi32(_mm_add_epi32(t1, qdelta), W_BITS1 - 5);
                    diff0 = _mm_subs_epi16(_mm_packs_epi32(t0, t1), diff0);
                    diff1 = _mm_unpackhi_epi16(diff0, diff0);
                    diff0 = _mm_unpacklo_epi16(diff0, diff0);        // It0 It0 It1 It1 ...
                    v00 = _mm_loadu_si128((const __m128i *)(dIptr)); // Ix0 Iy0 Ix1 Iy1 ...
                    v01 = _mm_loadu_si128((const __m128i *)(dIptr + 8));
                    v10 = _mm_unpacklo_epi16(v00, v01);
                    v11 = _mm_unpackhi_epi16(v00, v01);
                    v00 = _mm_unpacklo_epi16(diff0, diff1);
                    v01 = _mm_unpackhi_epi16(diff0, diff1);
                    v00 = _mm_madd_epi16(v00, v10);
                    v11 = _mm_madd_epi16(v01, v11);
                    qb0 = _mm_add_ps(qb0, _mm_cvtepi32_ps(v00));
                    qb1 = _mm_add_ps(qb1, _mm_cvtepi32_ps(v11));
                }

                for (; x < winSize.width * cn; x++, dIptr += 2)
                {
                    int diff = CV_DESCALE(Jptr[x] * iw00 + Jptr[x + cn] * iw01 + Jptr[x + stepJ] * iw10 + Jptr[x + stepJ + cn] * iw11, W_BITS1 - 5) - Iptr[x];
                    ib1 += (itemtype)(diff * dIptr[0]);
                    ib2 += (itemtype)(diff * dIptr[1]);
                }
            }

#if CV_SSE2
            float CV_DECL_ALIGNED(16) bbuf[4];
            _mm_store_ps(bbuf, _mm_add_ps(qb0, qb1));

            ib1 += bbuf[0] + bbuf[2];
            ib2 += bbuf[1] + bbuf[3];
#endif

            b1 = ib1 * FLT_SCALE;
            b2 = ib2 * FLT_SCALE;

            Point2f delta((float)((A12 * b2 - A22 * b1) * D), (float)((A12 * b1 - A11 * b2) * D));

            nextPt += delta;
            nextPts[ptidx] = nextPt + halfWin;

            if (delta.ddot(delta) <= criteria.epsilon) break;

            if (j > 0 && std::abs(delta.x + prevDelta.x) < 0.01 && std::abs(delta.y + prevDelta.y) < 0.01)
            {
                nextPts[ptidx] -= delta * 0.5f;
                break;
            }

            prevDelta = delta;
        }

        CV_Assert(status != NULL);

        if (status[ptidx] && err && level == 0 && (flags & OPTFLOW_LK_GET_MIN_EIGENVALS) == 0)
        {
            Point2f nextPoint = nextPts[ptidx] - halfWin;
            Point inextPoint;

            inextPoint.x = cvFloor(nextPoint.x);
            inextPoint.y = cvFloor(nextPoint.y);

            if (inextPoint.x < -winSize.width || inextPoint.x >= J.cols || inextPoint.y < -winSize.height || inextPoint.y >= J.rows)
            {
                if (status)
                    status[ptidx] = false;

                continue;
            }

            float aa = nextPoint.x - inextPoint.x;
            float bb = nextPoint.y - inextPoint.y;

            iw00 = cvRound((1.f - aa) * (1.f - bb) * (1 << W_BITS));
            iw01 = cvRound(aa * (1.f - bb) * (1 << W_BITS));
            iw10 = cvRound((1.f - aa) * bb * (1 << W_BITS));
            iw11 = (1 << W_BITS) - iw00 - iw01 - iw10;

            float errval = 0.f;

            for (y = 0; y < winSize.height; y++)
            {
                const uchar *Jptr = J.ptr() + (y + inextPoint.y) * stepJ + inextPoint.x * cn;
                const deriv_type *Iptr = IWinBuf.ptr<deriv_type>(y);

                for (x = 0; x < winSize.width * cn; x++)
                {
                    int diff = CV_DESCALE(Jptr[x] * iw00 + Jptr[x + cn] * iw01 + Jptr[x + stepJ] * iw10 + Jptr[x + stepJ + cn] * iw11, W_BITS1 - 5) - Iptr[x];
                    errval += std::abs((float)diff);
                }
            }

            err[ptidx] = errval * 1.f / (32 * winSize.width * cn * winSize.height);
        }
    }
}

void opencvLKTrackerInvoker::operator()(const cv::Range &range) const
{
    calculateLKOpticalFlow(range, prevImg, prevDeriv, nextImg, prevPts, nextPts, status, err, winSize, criteria, level, maxLevel, flags, minEigThreshold);
}

bool opencvLKTrackerInvoker::calculate(cv::Range range) const
{
    calculateLKOpticalFlow(range, prevImg, prevDeriv, nextImg, prevPts, nextPts, status, err, winSize, criteria, level, maxLevel, flags, minEigThreshold);

    return true;
}

inline int opencvBuildOpticalFlowPyramid(InputArray img_, OutputArrayOfArrays pyramid, Size winSize, int maxLevel, bool withDerivatives,
                                          int pyrBorder, int derivBorder, bool tryReuseInputImage)
{
    Mat img = img_.getMat();

    CV_Assert(img.depth() == CV_8U && winSize.width > 2 && winSize.height > 2);

    int pyrstep = withDerivatives ? 2 : 1;

#if (CV_MAJOR_VERSION==4)
    pyramid.create(1, (maxLevel + 1) * pyrstep, 0, -1, true);
#else
    pyramid.create(1, (maxLevel + 1) * pyrstep, 0, -1, true, 0);
#endif

    int derivType = CV_MAKETYPE(DataType<deriv_type>::depth, img.channels() * 2);

    bool lvl0IsSet = false;

    if (tryReuseInputImage && img.isSubmatrix() && (pyrBorder & BORDER_ISOLATED) == 0)
    {
        Size wholeSize;
        Point ofs;
        img.locateROI(wholeSize, ofs);
        if (ofs.x >= winSize.width && ofs.y >= winSize.height && ofs.x + img.cols + winSize.width <= wholeSize.width && ofs.y + img.rows + winSize.height <= wholeSize.height)
        {
            pyramid.getMatRef(0) = img;
            lvl0IsSet = true;
        }
    }

    if (!lvl0IsSet)
    {
        Mat &temp = pyramid.getMatRef(0);

        if (!temp.empty())
            temp.adjustROI(winSize.height, winSize.height, winSize.width, winSize.width);

        if (temp.type() != img.type() || temp.cols != winSize.width * 2 + img.cols || temp.rows != winSize.height * 2 + img.rows)
        {
            temp.create(img.rows + winSize.height * 2, img.cols + winSize.width * 2, img.type());
        }

        if (pyrBorder == BORDER_TRANSPARENT)
            img.copyTo(temp(Rect(winSize.width, winSize.height, img.cols, img.rows)));
        else
            copyMakeBorder(img, temp, winSize.height, winSize.height, winSize.width, winSize.width, pyrBorder);

        temp.adjustROI(-winSize.height, -winSize.height, -winSize.width, -winSize.width);
    }

    Size sz = img.size();
    Mat prevLevel = pyramid.getMatRef(0);
    Mat thisLevel = prevLevel;

    for (int level = 0; level <= maxLevel; ++level)
    {
        if (level != 0)
        {
            Mat &temp = pyramid.getMatRef(level * pyrstep);

            if (!temp.empty())
            {
                temp.adjustROI(winSize.height, winSize.height, winSize.width, winSize.width);
            }

            if (temp.type() != img.type() || temp.cols != winSize.width * 2 + sz.width || temp.rows != winSize.height * 2 + sz.height)
            {
                temp.create(sz.height + winSize.height * 2, sz.width + winSize.width * 2, img.type());
            }

            thisLevel = temp(Rect(winSize.width, winSize.height, sz.width, sz.height));
            pyrDown(prevLevel, thisLevel, sz);

            if (pyrBorder != BORDER_TRANSPARENT)
                copyMakeBorder(thisLevel, temp, winSize.height, winSize.height, winSize.width, winSize.width, pyrBorder | BORDER_ISOLATED);

            temp.adjustROI(-winSize.height, -winSize.height, -winSize.width, -winSize.width);
        }

        if (withDerivatives)
        {
            Mat &deriv = pyramid.getMatRef(level * pyrstep + 1);

            if (!deriv.empty())
                deriv.adjustROI(winSize.height, winSize.height, winSize.width, winSize.width);

            if (deriv.type() != derivType || deriv.cols != winSize.width * 2 + sz.width || deriv.rows != winSize.height * 2 + sz.height)
                deriv.create(sz.height + winSize.height * 2, sz.width + winSize.width * 2, derivType);

            Mat derivI = deriv(Rect(winSize.width, winSize.height, sz.width, sz.height));
            calcSharrDeriv(thisLevel, derivI);

            if (derivBorder != BORDER_TRANSPARENT)
                copyMakeBorder(derivI, deriv, winSize.height, winSize.height, winSize.width, winSize.width, derivBorder | BORDER_ISOLATED);

            deriv.adjustROI(-winSize.height, -winSize.height, -winSize.width, -winSize.width);
        }

        sz = Size((sz.width + 1) / 2, (sz.height + 1) / 2);

        if (sz.width <= winSize.width || sz.height <= winSize.height)
        {
#if (CV_MAJOR_VERSION==4)
            pyramid.create(1, (level + 1) * pyrstep, 0, -1, true); //check this
#else 
            pyramid.create(1, (level + 1) * pyrstep, 0, -1, true, 0); //check this
#endif
            return level;
        }

        prevLevel = thisLevel;
    }

    return maxLevel;
}

void LKOpticalFlowKernel::allocateImgDerivMemory(std::vector<Mat> &img_pyr, std::vector<Mat> &img_pyr_deriv_I, std::vector<Mat> &img_pyr_deriv_I_buff)
{
    int derivDepth = cv::DataType<deriv_type>::depth;

    img_pyr_deriv_I.resize(img_pyr.size());
    img_pyr_deriv_I_buff.resize(img_pyr.size());

    for (int level = maxLevel; level >= 0; level--)
    {
        if (img_pyr_deriv_I_buff[level].cols == 0)
        {
            // dI/dx ~ Ix, dI/dy ~ Iy
            // Create the pyramid mat with add the padding.
            img_pyr_deriv_I_buff[level].create(img_pyr[level].rows + lk_win_size.height * 2, 
                img_pyr[level].cols + lk_win_size.width * 2, CV_MAKETYPE(derivDepth, img_pyr[level].channels() * 2));
        }
    }
}

void LKOpticalFlowKernel::calcImageDerivSharr(std::vector<cv::Mat> &img_pyr, std::vector<cv::Mat> &img_pyr_deriv_I, std::vector<cv::Mat> &img_pyr_deriv_I_buff)
{
    if (img_pyr_deriv_I_buff.size() == 0 || img_pyr_deriv_I_buff[0].size().width == 0 || img_pyr_deriv_I_buff[0].size().height == 0)
    {
        allocateImgDerivMemory(img_pyr, img_pyr_deriv_I, img_pyr_deriv_I_buff);
    }

    // Calculate Image derivative
    for (int level = maxLevel; level >= 0; level--)
    {
        cv::Size imgSize = img_pyr[level].size();

        cv::Mat derivI(imgSize.height + lk_win_size.height * 2, imgSize.width + lk_win_size.width * 2, 
            img_pyr_deriv_I_buff[level].type(), img_pyr_deriv_I_buff[level].ptr());

        img_pyr_deriv_I[level] = derivI(cv::Rect(lk_win_size.width, lk_win_size.height, imgSize.width, imgSize.height));

        calcSharrDeriv(img_pyr[level], img_pyr_deriv_I[level]);

        cv::copyMakeBorder(img_pyr_deriv_I[level], derivI, lk_win_size.height, lk_win_size.height, 
            lk_win_size.width, lk_win_size.width, cv::BORDER_CONSTANT | cv::BORDER_ISOLATED);
    }
}

void LKOpticalFlowKernel::setTerminationCriteria(cv::TermCriteria &crit)
{
    // Set ther creteria of termination.
    if ((terminate_criteria.type & TermCriteria::COUNT) == 0)
        terminate_criteria.maxCount = 30;
    else
        terminate_criteria.maxCount = std::min(std::max(terminate_criteria.maxCount, 0), 100);

    if ((terminate_criteria.type & TermCriteria::EPS) == 0)
        terminate_criteria.epsilon = 0.01;
    else
        terminate_criteria.epsilon = std::min(std::max(terminate_criteria.epsilon, 0.), 10.);
}

void LKOpticalFlowKernel::calc(InputArray prevImg_, InputArray nextImg_, InputArray prevPts_, InputOutputArray nextPts_, OutputArray status_, OutputArray err_)
{
    Mat prevPtsMat = prevPts_.getMat();
    const int derivDepth = DataType<deriv_type>::depth;

    CV_Assert(maxLevel >= 0 && lk_win_size.width > 2 && lk_win_size.height > 2);
    int level = 0, i, n_points;
    CV_Assert((n_points = prevPtsMat.checkVector(2, CV_32F, true)) >= 0);

    if (n_points == 0)
    {
        nextPts_.release();
        status_.release();
        err_.release();
        return;
    }

    if (!(flags & cv_OPTFLOW_USE_INITIAL_FLOW))
    {
        nextPts_.create(prevPtsMat.size(), prevPtsMat.type(), -1, true);
    }
    Mat nextPtsMat = nextPts_.getMat();
    CV_Assert(nextPtsMat.checkVector(2, CV_32F, true) == n_points);

    const Point2f *prevPts = prevPtsMat.ptr<Point2f>();
    Point2f *nextPts = nextPtsMat.ptr<Point2f>();

    status_.create((int)n_points, 1, CV_8U, -1, true);
    Mat statusMat = status_.getMat(), errMat;
    CV_Assert(statusMat.isContinuous());
    uchar *status = statusMat.ptr();
    float *err = 0;

    for (i = 0; i < n_points; i++)
    {
        status[i] = true;
    }

    if (err_.needed())
    {
        err_.create((int)n_points, 1, CV_32F, -1, true);
        errMat = err_.getMat();
        CV_Assert(errMat.isContinuous());
        err = errMat.ptr<float>();
    }

    maxLevel = opencvBuildOpticalFlowPyramid(prevImg_, prev_img_pyr, lk_win_size, maxLevel, false);
    maxLevel = opencvBuildOpticalFlowPyramid(nextImg_, curr_img_pyr, lk_win_size, maxLevel, false);

    calcImageDerivSharr(prev_img_pyr, prev_img_deriv_I, prev_img_deriv_I_buff);

    for (level = maxLevel; level >= 0; level--)
    {
        CV_Assert(prev_img_pyr[level].size() == curr_img_pyr[level].size());
        CV_Assert(prev_img_pyr[level].type() == curr_img_pyr[level].type());
        parallel_for_(Range(0, n_points), opencvLKTrackerInvoker(&prev_img_pyr[level], &prev_img_deriv_I[level], &curr_img_pyr[level], 
            prevPts, nextPts, status, err, lk_win_size, terminate_criteria, level, maxLevel, flags, (float)minEigThreshold));
    }
}

void LKOpticalFlowKernel::swapImageBuffer()
{
    // Swap image buffer, avoiding reallocate the memory.
    for (int level = maxLevel; level >= 0; level--)
    {
        std::swap(prev_img_pyr[level], curr_img_pyr[level]);
        std::swap(prev_img_deriv_I[level], curr_img_deriv_I[level]);
        std::swap(prev_img_deriv_I_buff[level], curr_img_deriv_I_buff[level]);
    }
}

int LKOpticalFlowKernel::trackImage(const cv::Mat &curr_img, const std::vector<cv::Point2f> &last_tracked_pts,
    std::vector<cv::Point2f> &curr_tracked_pts, std::vector<uchar> &status, int opm_method)
{
    maxLevel = opencvBuildOpticalFlowPyramid(curr_img, curr_img_pyr, lk_win_size, maxLevel, false);

    calcImageDerivSharr(curr_img_pyr, curr_img_deriv_I, curr_img_deriv_I_buff);

    if (prev_img_pyr.size() == 0 || (prev_img_pyr[0].cols == 0)) // The first img
    {
        prev_img_pyr.resize(curr_img_pyr.size());

        allocateImgDerivMemory(curr_img_pyr, prev_img_deriv_I, prev_img_deriv_I_buff);

        swapImageBuffer();

        curr_tracked_pts = last_tracked_pts;

        return 0;
    }

    curr_tracked_pts = last_tracked_pts;
    status.resize(last_tracked_pts.size());

    for (int i = 0; i < last_tracked_pts.size(); i++)
    {
        status[i] = 1;
    }

    cv::parallel_for_(
        Range(0, last_tracked_pts.size()), [&](const Range &range) {
            for (int level = maxLevel; level >= 0; level--) {
                calculateLKOpticalFlow(range, &prev_img_pyr[level], &prev_img_deriv_I[level], &curr_img_pyr[level], last_tracked_pts.data(),
                    curr_tracked_pts.data(), status.data(), 0, lk_win_size, terminate_criteria, level, maxLevel, flags, minEigThreshold);
            }
        }
    );

    swapImageBuffer();

    return std::accumulate(status.begin(), status.end(), 0);
}

void calculateOpticalFlow(InputArray prevImg_, InputArray nextImg_, InputArray prevPts_, InputOutputArray nextPts_,
    OutputArray status_, OutputArray err_, Size winSize, int maxLevel, TermCriteria criteria, int flags, double minEigThreshold)
{
    cv::Ptr<LKOpticalFlowKernel> optflow_kernel = cv::makePtr<LKOpticalFlowKernel>(winSize, maxLevel, criteria, flags, minEigThreshold);
    optflow_kernel->calc(prevImg_, nextImg_, prevPts_, nextPts_, status_, err_);
}
