// Copyright 2021 DeepMind Technologies Limited
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

#ifndef MUJOCO_SAMPLE_ARRAY_SAFETY_H_
#define MUJOCO_SAMPLE_ARRAY_SAFETY_H_

#include <algorithm>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstring>

// 提供了 sizeof() 运算符和标准库函数的安全替代方案,用于处理原始字符数组中的
// 以空字符结尾的(C风格)字符串。
// 这些函数使用编译时数组大小来限制读写操作在数组边界内。当数组大小无法在编译时
// 确定时(例如当数组退化为指针时),它们会触发编译错误。
// 它们不执行运行时边界检查。

namespace mujoco {
namespace sample_util {

// returns sizeof(arr)
// use instead of sizeof() to avoid unintended array-to-pointer decay
template <typename T, int N>
static constexpr std::size_t sizeof_arr(const T(&arr)[N]) {
  return sizeof(arr);
}

// like std::strcmp but it will not read beyond the bound of either lhs or rhs
template <std::size_t N1, std::size_t N2>
static inline int strcmp_arr(const char (&lhs)[N1], const char (&rhs)[N2]) {
  return std::strncmp(lhs, rhs, std::min(N1, N2));
}

// like std::strlen but it will not read beyond the bound of str
// if str is not null-terminated, returns sizeof(str)
template <std::size_t N>
static inline std::size_t strlen_arr(const char (&str)[N]) {
  for (std::size_t i = 0; i < N; ++i) {
    if (str[i] == '\0') {
      return i;
    }
  }
  return N;
}

// like std::sprintf but will not write beyond the bound of dest
// dest is guaranteed to be null-terminated
template <std::size_t N>
static inline int sprintf_arr(char (&dest)[N], const char* format, ...) {
  std::va_list vargs;
  va_start(vargs, format);
  int retval = std::vsnprintf(dest, N, format, vargs);
  va_end(vargs);
  return retval;
}

// like std::strcat but will not write beyond the bound of dest
// dest is guaranteed to be null-terminated
template <std::size_t N>
static inline char* strcat_arr(char (&dest)[N], const char* src) {
  const std::size_t dest_len = strlen_arr(dest);
  const std::size_t dest_size = sizeof_arr(dest);
  for (std::size_t i = dest_len; i < dest_size; ++i) {
    dest[i] = src[i - dest_len];
    if (!dest[i]) {
      break;
    }
  }
  dest[dest_size - 1] = '\0';
  return dest;
}

// like std::strcpy but won't write beyond the bound of dest
// dest is guaranteed to be null-terminated
template <std::size_t N>
static inline char* strcpy_arr(char (&dest)[N], const char* src) {
  {
    std::size_t i = 0;
    for (; src[i] && i < N - 1; ++i) {
      dest[i] = src[i];
    }
    dest[i] = '\0';
  }
  return &dest[0];
}

}  // namespace sample_util
}  // namespace mujoco

#endif  // MUJOCO_SAMPLE_ARRAY_SAFETY_H_
