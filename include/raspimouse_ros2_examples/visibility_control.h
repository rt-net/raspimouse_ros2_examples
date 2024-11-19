// Copyright 2020 RT Corporation
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

#ifndef RASPIMOUSE_ROS2_EXAMPLES__VISIBILITY_CONTROL_H_
#define RASPIMOUSE_ROS2_EXAMPLES__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define RASPIMOUSE_ROS2_EXAMPLES_EXPORT __attribute__((dllexport))
#define RASPIMOUSE_ROS2_EXAMPLES_IMPORT __attribute__((dllimport))
#else
#define RASPIMOUSE_ROS2_EXAMPLES_EXPORT __declspec(dllexport)
#define RASPIMOUSE_ROS2_EXAMPLES_IMPORT __declspec(dllimport)
#endif
#ifdef RASPIMOUSE_ROS2_EXAMPLES_BUILDING_DLL
#define RASPIMOUSE_ROS2_EXAMPLES_PUBLIC RASPIMOUSE_ROS2_EXAMPLES_EXPORT
#else
#define RASPIMOUSE_ROS2_EXAMPLES_PUBLIC RASPIMOUSE_ROS2_EXAMPLES_IMPORT
#endif
#define RASPIMOUSE_ROS2_EXAMPLES_PUBLIC_TYPE RASPIMOUSE_ROS2_EXAMPLES_PUBLIC
#define RASPIMOUSE_ROS2_EXAMPLES_LOCAL
#else
#define RASPIMOUSE_ROS2_EXAMPLES_EXPORT __attribute__((visibility("default")))
#define RASPIMOUSE_ROS2_EXAMPLES_IMPORT
#if __GNUC__ >= 4
#define RASPIMOUSE_ROS2_EXAMPLES_PUBLIC __attribute__((visibility("default")))
#define RASPIMOUSE_ROS2_EXAMPLES_LOCAL __attribute__((visibility("hidden")))
#else
#define RASPIMOUSE_ROS2_EXAMPLES_PUBLIC
#define RASPIMOUSE_ROS2_EXAMPLES_LOCAL
#endif
#define RASPIMOUSE_ROS2_EXAMPLES_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // RASPIMOUSE_ROS2_EXAMPLES__VISIBILITY_CONTROL_H_
