// Copyright (c) 2024 Tohoku Univ. Space Robotics Lab.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SYSTEM_DESCRIPTION__SYSTEM_DESCRIPTION_HPP_
#define SYSTEM_DESCRIPTION__SYSTEM_DESCRIPTION_HPP_

#include <math.h>


constexpr int JOINT_NUM = 1;

constexpr float Kpp = 32; // position controller P gain
constexpr float Kpv = 105; // velocity controller P gain
constexpr float Tiv = 0.03; // velocity controller I gain
constexpr float Tc = 0.005; // anti wind-up gain
constexpr float Tf = 1.0/(10*2*M_PI); // filtering time constant for refernce pre-filter (wf=1/Tf)
constexpr float Tdf = 1.0/(50*2*M_PI); // LP filter of derivative action, for realizability
constexpr float N = 250.0;  // reduction ratio



#endif  // SYSTEM_DESCRIPTION__SYSTEM_DESCRIPTION_HPP_
