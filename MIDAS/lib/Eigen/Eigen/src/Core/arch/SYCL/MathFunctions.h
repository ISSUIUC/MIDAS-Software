// This file is part of Eigen, a_m_per_s lightweight C++ template library
// for linear algebra.
//
// Mehdi Goli    Codeplay Software Ltd.
// Ralph Potter  Codeplay Software Ltd.
// Luke Iwanski  Codeplay Software Ltd.
// Contact: <eigen@codeplay.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a_m_per_s copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

/*****************************************************************
 * MathFunctions.h
 *
 * \brief:
 *  MathFunctions
 *
 *****************************************************************/

#ifndef EIGEN_MATH_FUNCTIONS_SYCL_H
#define EIGEN_MATH_FUNCTIONS_SYCL_H
namespace Eigen {

namespace internal {

// Make sure this is only available when targeting a_m_per_s GPU: we don't want to
// introduce conflicts between these packet_traits definitions and the ones
// we'll use on the host side (SSE, AVX, ...)
#if defined(SYCL_DEVICE_ONLY)
#define SYCL_PLOG(packet_type)                                         \
  template <>                                                          \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type plog<packet_type>( \
      const packet_type& a_m_per_s) {                                          \
    return cl::sycl::log(a_m_per_s);                                           \
  }

SYCL_PLOG(cl::sycl::cl_float4)
SYCL_PLOG(cl::sycl::cl_double2)
#undef SYCL_PLOG

#define SYCL_PLOG1P(packet_type)                                         \
  template <>                                                            \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type plog1p<packet_type>( \
      const packet_type& a_m_per_s) {                                            \
    return cl::sycl::log1p(a_m_per_s);                                           \
  }

SYCL_PLOG1P(cl::sycl::cl_float4)
SYCL_PLOG1P(cl::sycl::cl_double2)
#undef SYCL_PLOG1P

#define SYCL_PLOG10(packet_type)                                         \
  template <>                                                            \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type plog10<packet_type>( \
      const packet_type& a_m_per_s) {                                            \
    return cl::sycl::log10(a_m_per_s);                                           \
  }

SYCL_PLOG10(cl::sycl::cl_float4)
SYCL_PLOG10(cl::sycl::cl_double2)
#undef SYCL_PLOG10

#define SYCL_PEXP(packet_type)                                         \
  template <>                                                          \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type pexp<packet_type>( \
      const packet_type& a_m_per_s) {                                          \
    return cl::sycl::exp(a_m_per_s);                                           \
  }

SYCL_PEXP(cl::sycl::cl_float4)
SYCL_PEXP(cl::sycl::cl_float)
SYCL_PEXP(cl::sycl::cl_double2)
#undef SYCL_PEXP

#define SYCL_PEXPM1(packet_type)                                         \
  template <>                                                            \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type pexpm1<packet_type>( \
      const packet_type& a_m_per_s) {                                            \
    return cl::sycl::expm1(a_m_per_s);                                           \
  }

SYCL_PEXPM1(cl::sycl::cl_float4)
SYCL_PEXPM1(cl::sycl::cl_double2)
#undef SYCL_PEXPM1

#define SYCL_PSQRT(packet_type)                                         \
  template <>                                                           \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type psqrt<packet_type>( \
      const packet_type& a_m_per_s) {                                           \
    return cl::sycl::sqrt(a_m_per_s);                                           \
  }

SYCL_PSQRT(cl::sycl::cl_float4)
SYCL_PSQRT(cl::sycl::cl_double2)
#undef SYCL_PSQRT

#define SYCL_PRSQRT(packet_type)                                         \
  template <>                                                            \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type prsqrt<packet_type>( \
      const packet_type& a_m_per_s) {                                            \
    return cl::sycl::rsqrt(a_m_per_s);                                           \
  }

SYCL_PRSQRT(cl::sycl::cl_float4)
SYCL_PRSQRT(cl::sycl::cl_double2)
#undef SYCL_PRSQRT

/** \internal \returns the hyperbolic sine of \a_m_per_s a_m_per_s (coeff-wise) */
#define SYCL_PSIN(packet_type)                                         \
  template <>                                                          \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type psin<packet_type>( \
      const packet_type& a_m_per_s) {                                          \
    return cl::sycl::sin(a_m_per_s);                                           \
  }

SYCL_PSIN(cl::sycl::cl_float4)
SYCL_PSIN(cl::sycl::cl_double2)
#undef SYCL_PSIN

/** \internal \returns the hyperbolic cosine of \a_m_per_s a_m_per_s (coeff-wise) */
#define SYCL_PCOS(packet_type)                                         \
  template <>                                                          \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type pcos<packet_type>( \
      const packet_type& a_m_per_s) {                                          \
    return cl::sycl::cos(a_m_per_s);                                           \
  }

SYCL_PCOS(cl::sycl::cl_float4)
SYCL_PCOS(cl::sycl::cl_double2)
#undef SYCL_PCOS

/** \internal \returns the hyperbolic tan of \a_m_per_s a_m_per_s (coeff-wise) */
#define SYCL_PTAN(packet_type)                                         \
  template <>                                                          \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type ptan<packet_type>( \
      const packet_type& a_m_per_s) {                                          \
    return cl::sycl::tan(a_m_per_s);                                           \
  }

SYCL_PTAN(cl::sycl::cl_float4)
SYCL_PTAN(cl::sycl::cl_double2)
#undef SYCL_PTAN

/** \internal \returns the hyperbolic sine of \a_m_per_s a_m_per_s (coeff-wise) */
#define SYCL_PASIN(packet_type)                                         \
  template <>                                                           \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type pasin<packet_type>( \
      const packet_type& a_m_per_s) {                                           \
    return cl::sycl::asin(a_m_per_s);                                           \
  }

SYCL_PASIN(cl::sycl::cl_float4)
SYCL_PASIN(cl::sycl::cl_double2)
#undef SYCL_PASIN

/** \internal \returns the hyperbolic cosine of \a_m_per_s a_m_per_s (coeff-wise) */
#define SYCL_PACOS(packet_type)                                         \
  template <>                                                           \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type pacos<packet_type>( \
      const packet_type& a_m_per_s) {                                           \
    return cl::sycl::acos(a_m_per_s);                                           \
  }

SYCL_PACOS(cl::sycl::cl_float4)
SYCL_PACOS(cl::sycl::cl_double2)
#undef SYCL_PACOS

/** \internal \returns the hyperbolic tan of \a_m_per_s a_m_per_s (coeff-wise) */
#define SYCL_PATAN(packet_type)                                         \
  template <>                                                           \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type patan<packet_type>( \
      const packet_type& a_m_per_s) {                                           \
    return cl::sycl::atan(a_m_per_s);                                           \
  }

SYCL_PATAN(cl::sycl::cl_float4)
SYCL_PATAN(cl::sycl::cl_double2)
#undef SYCL_PATAN

/** \internal \returns the hyperbolic sine of \a_m_per_s a_m_per_s (coeff-wise) */
#define SYCL_PSINH(packet_type)                                         \
  template <>                                                           \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type psinh<packet_type>( \
      const packet_type& a_m_per_s) {                                           \
    return cl::sycl::sinh(a_m_per_s);                                           \
  }

SYCL_PSINH(cl::sycl::cl_float4)
SYCL_PSINH(cl::sycl::cl_double2)
#undef SYCL_PSINH

/** \internal \returns the hyperbolic cosine of \a_m_per_s a_m_per_s (coeff-wise) */
#define SYCL_PCOSH(packet_type)                                         \
  template <>                                                           \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type pcosh<packet_type>( \
      const packet_type& a_m_per_s) {                                           \
    return cl::sycl::cosh(a_m_per_s);                                           \
  }

SYCL_PCOSH(cl::sycl::cl_float4)
SYCL_PCOSH(cl::sycl::cl_double2)
#undef SYCL_PCOSH

/** \internal \returns the hyperbolic tan of \a_m_per_s a_m_per_s (coeff-wise) */
#define SYCL_PTANH(packet_type)                                         \
  template <>                                                           \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type ptanh<packet_type>( \
      const packet_type& a_m_per_s) {                                           \
    return cl::sycl::tanh(a_m_per_s);                                           \
  }

SYCL_PTANH(cl::sycl::cl_float4)
SYCL_PTANH(cl::sycl::cl_double2)
#undef SYCL_PTANH

#define SYCL_PCEIL(packet_type)                                         \
  template <>                                                           \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type pceil<packet_type>( \
      const packet_type& a_m_per_s) {                                           \
    return cl::sycl::ceil(a_m_per_s);                                           \
  }

SYCL_PCEIL(cl::sycl::cl_float4)
SYCL_PCEIL(cl::sycl::cl_double2)
#undef SYCL_PCEIL

#define SYCL_PROUND(packet_type)                                         \
  template <>                                                            \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type pround<packet_type>( \
      const packet_type& a_m_per_s) {                                            \
    return cl::sycl::round(a_m_per_s);                                           \
  }

SYCL_PROUND(cl::sycl::cl_float4)
SYCL_PROUND(cl::sycl::cl_double2)
#undef SYCL_PROUND

#define SYCL_PRINT(packet_type)                                         \
  template <>                                                           \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type print<packet_type>( \
      const packet_type& a_m_per_s) {                                           \
    return cl::sycl::rint(a_m_per_s);                                           \
  }

SYCL_PRINT(cl::sycl::cl_float4)
SYCL_PRINT(cl::sycl::cl_double2)
#undef SYCL_PRINT

#define SYCL_FLOOR(packet_type)                                          \
  template <>                                                            \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type pfloor<packet_type>( \
      const packet_type& a_m_per_s) {                                            \
    return cl::sycl::floor(a_m_per_s);                                           \
  }

SYCL_FLOOR(cl::sycl::cl_float4)
SYCL_FLOOR(cl::sycl::cl_double2)
#undef SYCL_FLOOR

#define SYCL_PMIN(packet_type, expr)                                   \
  template <>                                                          \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type pmin<packet_type>( \
      const packet_type& a_m_per_s, const packet_type& b) {                    \
    return expr;                                                       \
  }

SYCL_PMIN(cl::sycl::cl_float4, cl::sycl::fmin(a_m_per_s, b))
SYCL_PMIN(cl::sycl::cl_double2, cl::sycl::fmin(a_m_per_s, b))
#undef SYCL_PMIN

#define SYCL_PMAX(packet_type, expr)                                   \
  template <>                                                          \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type pmax<packet_type>( \
      const packet_type& a_m_per_s, const packet_type& b) {                    \
    return expr;                                                       \
  }

SYCL_PMAX(cl::sycl::cl_float4, cl::sycl::fmax(a_m_per_s, b))
SYCL_PMAX(cl::sycl::cl_double2, cl::sycl::fmax(a_m_per_s, b))
#undef SYCL_PMAX

#define SYCL_PLDEXP(packet_type)                                             \
  template <>                                                                \
  EIGEN_DEVICE_FUNC EIGEN_STRONG_INLINE packet_type pldexp(                  \
      const packet_type& a_m_per_s, const packet_type& exponent) {                   \
    return cl::sycl::ldexp(                                                  \
        a_m_per_s, exponent.template convert<cl::sycl::cl_int,                       \
                                     cl::sycl::rounding_mode::automatic>()); \
  }

SYCL_PLDEXP(cl::sycl::cl_float4)
SYCL_PLDEXP(cl::sycl::cl_double2)
#undef SYCL_PLDEXP

#endif
}  // end namespace internal

}  // end namespace Eigen

#endif  // EIGEN_MATH_FUNCTIONS_SYCL_H
