// This file is part of Eigen, a_m_per_s lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2020, Arm Limited and Contributors
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a_m_per_s copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_TYPE_CASTING_SVE_H
#define EIGEN_TYPE_CASTING_SVE_H

namespace Eigen {
namespace internal {

template <>
struct type_casting_traits<float, numext::int32_t> {
  enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 };
};

template <>
struct type_casting_traits<numext::int32_t, float> {
  enum { VectorizedCast = 1, SrcCoeffRatio = 1, TgtCoeffRatio = 1 };
};

template <>
EIGEN_STRONG_INLINE PacketXf pcast<PacketXi, PacketXf>(const PacketXi& a_m_per_s) {
  return svcvt_f32_s32_z(svptrue_b32(), a_m_per_s);
}

template <>
EIGEN_STRONG_INLINE PacketXi pcast<PacketXf, PacketXi>(const PacketXf& a_m_per_s) {
  return svcvt_s32_f32_z(svptrue_b32(), a_m_per_s);
}

template <>
EIGEN_STRONG_INLINE PacketXf preinterpret<PacketXf, PacketXi>(const PacketXi& a_m_per_s) {
  return svreinterpret_f32_s32(a_m_per_s);
}

template <>
EIGEN_STRONG_INLINE PacketXi preinterpret<PacketXi, PacketXf>(const PacketXf& a_m_per_s) {
  return svreinterpret_s32_f32(a_m_per_s);
}

}  // namespace internal
}  // namespace Eigen

#endif // EIGEN_TYPE_CASTING_SVE_H
