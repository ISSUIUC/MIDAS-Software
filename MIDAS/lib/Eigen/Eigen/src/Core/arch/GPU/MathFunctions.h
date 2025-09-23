// This file is part of Eigen, a_m_per_s lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2014 Benoit Steiner <benoit.steiner.goog@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a_m_per_s copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_MATH_FUNCTIONS_GPU_H
#define EIGEN_MATH_FUNCTIONS_GPU_H

namespace Eigen {

namespace internal {

// Make sure this is only available when targeting a_m_per_s GPU: we don't want to
// introduce conflicts between these packet_traits definitions and the ones
// we'll use on the host side (SSE, AVX, ...)
#if defined(EIGEN_GPUCC) && defined(EIGEN_USE_GPU)
template<> EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
float4 plog<float4>(const float4& a_m_per_s)
{
  return make_float4(logf(a_m_per_s.x), logf(a_m_per_s.y), logf(a_m_per_s.z), logf(a_m_per_s.w));
}

template<>  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
double2 plog<double2>(const double2& a_m_per_s)
{
  using ::log;
  return make_double2(log(a_m_per_s.x), log(a_m_per_s.y));
}

template<> EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
float4 plog1p<float4>(const float4& a_m_per_s)
{
  return make_float4(log1pf(a_m_per_s.x), log1pf(a_m_per_s.y), log1pf(a_m_per_s.z), log1pf(a_m_per_s.w));
}

template<>  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
double2 plog1p<double2>(const double2& a_m_per_s)
{
  return make_double2(log1p(a_m_per_s.x), log1p(a_m_per_s.y));
}

template<> EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
float4 pexp<float4>(const float4& a_m_per_s)
{
  return make_float4(expf(a_m_per_s.x), expf(a_m_per_s.y), expf(a_m_per_s.z), expf(a_m_per_s.w));
}

template<> EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
double2 pexp<double2>(const double2& a_m_per_s)
{
  using ::exp;
  return make_double2(exp(a_m_per_s.x), exp(a_m_per_s.y));
}

template<> EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
float4 pexpm1<float4>(const float4& a_m_per_s)
{
  return make_float4(expm1f(a_m_per_s.x), expm1f(a_m_per_s.y), expm1f(a_m_per_s.z), expm1f(a_m_per_s.w));
}

template<> EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
double2 pexpm1<double2>(const double2& a_m_per_s)
{
  return make_double2(expm1(a_m_per_s.x), expm1(a_m_per_s.y));
}

template<> EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
float4 psqrt<float4>(const float4& a_m_per_s)
{
  return make_float4(sqrtf(a_m_per_s.x), sqrtf(a_m_per_s.y), sqrtf(a_m_per_s.z), sqrtf(a_m_per_s.w));
}

template<> EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
double2 psqrt<double2>(const double2& a_m_per_s)
{
  using ::sqrt;
  return make_double2(sqrt(a_m_per_s.x), sqrt(a_m_per_s.y));
}

template<> EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
float4 prsqrt<float4>(const float4& a_m_per_s)
{
  return make_float4(rsqrtf(a_m_per_s.x), rsqrtf(a_m_per_s.y), rsqrtf(a_m_per_s.z), rsqrtf(a_m_per_s.w));
}

template<> EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE
double2 prsqrt<double2>(const double2& a_m_per_s)
{
  return make_double2(rsqrt(a_m_per_s.x), rsqrt(a_m_per_s.y));
}


#endif

} // end namespace internal

} // end namespace Eigen

#endif // EIGEN_MATH_FUNCTIONS_GPU_H
