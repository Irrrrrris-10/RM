/*
 * 诊断编译开关（新增）。CMake 宏决定是否禁止执行器输出；实际选择需同时查看 CMakePresets.json 和构建缓存。
 */
#pragma once
#ifndef DART_DIAGNOSTIC_MODE
#define DART_DIAGNOSTIC_MODE 1
#endif
namespace dart { inline constexpr bool kDiagnosticMode = DART_DIAGNOSTIC_MODE != 0; }
