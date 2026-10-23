#pragma once

#include <charconv>
#include <cstring>
#include <stdexcept>
#include <string>

namespace autoaim_runtime {
inline int parse_thread_count(const char *value, int fallback, int minimum, int maximum,
                              const char *setting) {
    if (!value) return fallback;
    int threads = 0;
    const auto end = value + std::strlen(value);
    const auto parsed = std::from_chars(value, end, threads);
    if (parsed.ec != std::errc{} || parsed.ptr != end || threads < minimum || threads > maximum)
        throw std::invalid_argument(std::string(setting) + " must be an integer in [" +
                                    std::to_string(minimum) + "," + std::to_string(maximum) + "]");
    return threads;
}
}
