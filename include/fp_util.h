/**
 * Copyright (c) 2025 Samarium150
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FP_UTIL_HPP_
#define FP_UTIL_HPP_

#include <cmath>
#include <limits>
#include <sstream>

template <std::floating_point T>
constexpr T TOLERANCE = static_cast<T>(1e-6);

template <std::floating_point T>
constexpr T EPSILON = std::numeric_limits<T>::epsilon();

template <std::floating_point T>
bool FPEquals(T a, T b, T rel_epsilon = TOLERANCE<T>, T abs_epsilon = EPSILON<T>) {
    if (std::isinf(a) || std::isinf(b)) {
        return a == b;
    }
    if (std::isnan(a) || std::isnan(b)) {
        return false;
    }
    T diff = std::abs(a - b);
    if (diff <= abs_epsilon) {
        return true;
    }
    T max_val = std::max(std::abs(a), std::abs(b));
    return diff <= rel_epsilon * max_val;
}

template <std::floating_point T>
bool FPGreater(T a, T b, T rel_epsilon = TOLERANCE<T>, T abs_epsilon = EPSILON<T>) {
    if (std::isnan(a) || std::isnan(b)) {
        return false;
    }
    if (std::isinf(a) || std::isinf(b)) {
        return a > b;
    }
    const T max_val = std::max(std::abs(a), std::abs(b));
    const T threshold = std::max(abs_epsilon, rel_epsilon * max_val);
    return a - b > threshold;
}

template <std::floating_point T>
bool FPLess(T a, T b, T rel_epsilon = TOLERANCE<T>, T abs_epsilon = EPSILON<T>) {
    if (std::isnan(a) || std::isnan(b)) {
        return false;
    }
    if (std::isinf(a) || std::isinf(b)) {
        return a < b;
    }
    const T max_val = std::max(std::abs(a), std::abs(b));
    const T threshold = std::max(abs_epsilon, rel_epsilon * max_val);
    return b - a > threshold;
}

template <std::floating_point T>
bool FPGreaterOrEqual(T a, T b, T rel_epsilon = TOLERANCE<T>, T abs_epsilon = EPSILON<T>) {
    return FPGreater(a, b, rel_epsilon, abs_epsilon) || FPEquals(a, b, rel_epsilon, abs_epsilon);
}

template <std::floating_point T>
bool FPLessOrEqual(T a, T b, T rel_epsilon = TOLERANCE<T>, T abs_epsilon = EPSILON<T>) {
    return FPLess(a, b, rel_epsilon, abs_epsilon) || FPEquals(a, b, rel_epsilon, abs_epsilon);
}

template <std::floating_point T>
std::partial_ordering FPCompare(T a, T b, T rel_epsilon = TOLERANCE<T>,
                                T abs_epsilon = EPSILON<T>) {
    if (std::isnan(a) || std::isnan(b)) {
        return std::partial_ordering::unordered;
    }
    const T diff = a - b;
    const T max_val = std::max(std::abs(a), std::abs(b));
    const T threshold = std::max(abs_epsilon, rel_epsilon * max_val);
    if (diff > threshold) {
        return std::partial_ordering::greater;
    }
    if (diff < -threshold) {
        return std::partial_ordering::less;
    }
    return std::partial_ordering::equivalent;
}

template <std::floating_point T>
size_t CountDecimalPlaces(const T value) {
    std::stringstream ss;
    ss << std::fixed << value;
    std::string str = ss.str();
    const size_t decimal_pos = str.find('.');
    if (decimal_pos == std::string::npos) {
        return 0;
    }
    const size_t last_non_zero = str.find_last_not_of('0');
    if (last_non_zero == decimal_pos) {
        return 0;
    }
    return last_non_zero - decimal_pos;
}

template <typename T, std::unsigned_integral U>
double Round(const T f, const U n) {
    const double factor = std::pow(10, n);
    return std::round(f * factor) / factor;
}

#endif  // FP_UTIL_HPP_
