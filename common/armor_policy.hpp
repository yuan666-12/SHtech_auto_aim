#pragma once
#include <cstdlib>
#include <cstring>
namespace armor_policy {
// Provincial mode keeps enemy-color filtering; digit classification is diagnostic only.
inline bool ignore_armor_id()
{
    const char* value = std::getenv("AUTOAIM_IGNORE_ARMOR_ID");
    return value == nullptr || std::strcmp(value, "0") != 0;
}
// Nonzero ordinary small-armor class; not a detected digit.
constexpr int generic_small_armor_id = 2;
inline int tracking_id(int detected_id, bool ignore_id)
{
    return ignore_id ? generic_small_armor_id : detected_id;
}
}
