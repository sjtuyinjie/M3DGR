/*
 * patch_score.h
 *
 *  Created on: Dec 5, 2013
 *      Author: cforster
 */

#ifndef VIKIT_PATCH_SCORE_H_
#define VIKIT_PATCH_SCORE_H_

#include <stdint.h>

#if __SSE2__
#include <tmmintrin.h>
#endif

namespace vk {
namespace patch_score {


#if __SSE2__
// Horizontal sum of uint16s stored in an XMM register
inline int SumXMM_16(__m128i &target)
{
  unsigned short int sums_store[8];
  _mm_storeu_si128((__m128i*)sums_store, target);
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3] +
    sums_store[4] + sums_store[5] + sums_store[6] + sums_store[7];
}
// Horizontal sum of uint32s stored in an XMM register
inline int SumXMM_32(__m128i &target)
{
  unsigned int sums_store[4];
  _mm_storeu_si128((__m128i*)sums_store, target);
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3];
}
#endif

/// Zero Mean Sum of Squared Differences Cost
template<int HALF_PATCH_SIZE>
class ZMSSD {
public:

  static const int patch_size_ = 2*HALF_PATCH_SIZE;
  static const int patch_area_ = patch_size_*patch_size_;
  static const int threshold_  = 2000*patch_area_;
  uint8_t* ref_patch_;
  int sumA_, sumAA_;

  ZMSSD(uint8_t* ref_patch) :
    ref_patch_(ref_patch)
  {
    uint32_t sumA_uint=0, sumAA_uint=0;
    for(int r = 0; r < patch_area_; r++)
    {
      uint8_t n = ref_patch_[r];
      sumA_uint += n;
      sumAA_uint += n*n;
    }
    sumA_ = sumA_uint;
    sumAA_ = sumAA_uint;
  }

  static int threshold() { return threshold_; }

  int computeScore(uint8_t* cur_patch) const
  {
    uint32_t sumB_uint = 0;
    uint32_t sumBB_uint = 0;
    uint32_t sumAB_uint = 0;
    for(int r = 0; r < patch_area_; r++)
    {
      const uint8_t cur_pixel = cur_patch[r];
      sumB_uint  += cur_pixel;
      sumBB_uint += cur_pixel*cur_pixel;
      sumAB_uint += cur_pixel * ref_patch_[r];
    }
    const int sumB = sumB_uint;
    const int sumBB = sumBB_uint;
    const int sumAB = sumAB_uint;
    return sumAA_ - 2*sumAB + sumBB - (sumA_*sumA_ - 2*sumA_*sumB + sumB*sumB)/patch_area_;
  }

  int computeScore(uint8_t* cur_patch, int stride) const
  {
    int sumB, sumBB, sumAB;
#if __SSE2__
    if(patch_size_ == 8)
    {
      // From PTAM-GPL, Copyright 2008 Isis Innovation Limited
      __m128i xImageAsEightBytes;
      __m128i xImageAsWords;
      __m128i xTemplateAsEightBytes;
      __m128i xTemplateAsWords;
      __m128i xZero;
      __m128i xImageSums;   // These sums are 8xuint16
      __m128i xImageSqSums; // These sums are 4xint32
      __m128i xCrossSums;   // These sums are 4xint32
      __m128i xProduct;

      xImageSums = _mm_setzero_si128();
      xImageSqSums = _mm_setzero_si128();
      xCrossSums = _mm_setzero_si128();
      xZero = _mm_setzero_si128();

      uint8_t* imagepointer = cur_patch;
      uint8_t* templatepointer = ref_patch_;
      long unsigned int cur_stride = stride;

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += cur_stride;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      sumB = SumXMM_16(xImageSums);
      sumAB = SumXMM_32(xCrossSums);
      sumBB = SumXMM_32(xImageSqSums);
    }
    else
#endif
    {
      uint32_t sumB_uint = 0;
      uint32_t sumBB_uint = 0;
      uint32_t sumAB_uint = 0;
      for(int y=0, r=0; y < patch_size_; ++y)
      {
        uint8_t* cur_patch_ptr = cur_patch + y*stride;
        for(int x=0; x < patch_size_; ++x, ++r)
        {
          const uint8_t cur_px = cur_patch_ptr[x];
          sumB_uint  += cur_px;
          sumBB_uint += cur_px * cur_px;
          sumAB_uint += cur_px * ref_patch_[r];
        }
      }
      sumB = sumB_uint;
      sumBB = sumBB_uint;
      sumAB = sumAB_uint;
    }
    return sumAA_ - 2*sumAB + sumBB - (sumA_*sumA_ - 2*sumA_*sumB + sumB*sumB)/patch_area_;
  }
};

} // namespace patch_score
} // namespace vk

#endif // VIKIT_PATCH_SCORE_H_
