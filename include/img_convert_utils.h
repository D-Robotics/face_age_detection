// Copyright (c) 2024，D-Robotics.
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

#ifndef IMG_CONVERT_UTILS_H
#define IMG_CONVERT_UTILS_H

#include <arm_neon.h>
#include "opencv2/opencv.hpp"

namespace utils
{
void nv12_to_bgr24_neon(uint8_t *nv12, uint8_t *bgr24, int width, int height);
void bgr24_to_nv12_neon(uint8_t *bgr24, uint8_t *nv12, int width, int height);
void bgr_to_nv12_mat(const cv::Mat &bgr, cv::Mat &nv12);
} // namespace utils

#endif