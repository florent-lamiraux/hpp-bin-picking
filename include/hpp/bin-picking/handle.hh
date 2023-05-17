// Copyright (c) 2023, LAAS-CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <cmath>
#include <boost/format.hpp>
#include <hpp/bin-picking/fwd.hh>
#include <hpp/manipulation/handle.hh>

#ifndef HPP_BIN_PICKING_HANDLE_HH
#define  HPP_BIN_PICKING_HANDLE_HH

namespace hpp{
namespace bin_picking{
std::vector<HandlePtr_t> discretizeHandle(const HandlePtr_t& handle,
                                          size_type nbHandles)
{
  std::vector<HandlePtr_t> res(nbHandles, HandlePtr_t());
  std::vector<bool> mask(handle->mask());
  if ((!mask[0] || !mask[1] || !mask[2]) ||
      (mask[3] + mask[4] + mask[5] != 2)){
    throw std::logic_error("Mask should contain one and only one false value"
                           "along a rotation degree of freedom");
  }
  value_type c = cos(2*M_PI/(double)nbHandles),
    s = sin(2*M_PI/(double)nbHandles);
  matrix3_t R; R.setZero();
  matrix3_t R0; R0.setIdentity();
  size_type i0, i1, i2;
  if (!mask[3]) {// rotation around x axis
    i0=0; i1=1; i2=2;
  }
  else if (!mask[4]) {// rotation around y axis
    i0=1; i1=2; i2=0;
  }
  else {// rotation around z axis
    i0=2; i1=0; i2=1;
  }
  R(i0, i0) = 1.;
  R(i1, i1) = c; R(i1, i2) = -s;
  R(i2, i1) = s; R(i2, i2) = c;
  vector3_t zero; zero.setZero();
  SE3 pose(handle->localPosition());
  SE3 delta(R, zero);
  for (size_type i=0; i<nbHandles; ++i) {
    boost::format fmt = boost::format("%s_%03d") % handle->name()% i;
    res[i] = Handle::create(fmt.str(), pose, handle->robot(), handle->joint());
    res[i]->clearance(handle->clearance());
    pose = pose * delta;
  }
  return res;
}
} // namespace bin_picking
} // namespace hpp
#endif //  HPP_BIN_PICKING_HANDLE_HH
