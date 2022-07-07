/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "linear_interpolation.h"

#include <cmath>

#include "math_utils.h"


namespace planning {

float slerp(const float a0, const float t0, const float a1, const float t1,
             const float t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    return NormalizeAngle(a0);
  }
  const float a0_n = NormalizeAngle(a0);
  const float a1_n = NormalizeAngle(a1);
  float d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const float r = (t - t0) / (t1 - t0);
  const float a = a0_n + d * r;
  return NormalizeAngle(a);
}



template<int N>
std::array<float, N> linspace(const float start, const float end)
{
  std::array<float, N> res;
  const float step = (end - start) / (N - 1);

  for(int16_t i = 0; i < N; i++) {
    res[i] = start + step * i;
  }

  return res;
}


std::vector<float> linspace(const float start, const float end, const int16_t count)
{
  std::vector<float> res(count, 0);
  float step = (end - start) / (count - 1);

  for(int16_t i = 0; i < count; i++) {
    res[i] = start + step * i;
  }

  return res;
}

}  // namespace planning