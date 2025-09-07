#pragma once
#include <algorithm>

namespace smmc {
template<typename T>
inline T clamp(T v, T lo, T hi){ return std::max(lo, std::min(hi, v)); }
} // namespace smmc
