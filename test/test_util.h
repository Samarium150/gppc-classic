#ifndef TEST_UTIL_H_
#define TEST_UTIL_H_

#include <gtest/gtest.h>

#include <limits>

#include "grid.h"
#include "scenario_loader.h"

struct alignas(64) Map {
    std::string name;
    std::vector<bool> data;
    int width;
    int height;
};

extern std::vector<Map> kMaps;

struct alignas(64) TestCase {
    size_t map_id;
    size_t case_id;
    gppc::Point start;
    gppc::Point goal;
    double expected_length;

    explicit TestCase(const size_t map_id, const size_t case_id,
                      const gppc::lib::Experiment& experiment) noexcept
        : map_id(map_id),
          case_id(case_id),
          start(experiment.GetStartX(), experiment.GetStartY()),
          goal(experiment.GetGoalX(), experiment.GetGoalY()),
          expected_length(experiment.GetDistance()) {}
};

std::vector<TestCase> LoadTestCases(const std::string& directory) noexcept;

std::string GenerateTestName(const testing::TestParamInfo<TestCase>& info) noexcept;

double GetPathLength(const std::vector<gppc::Point>& path) noexcept;

template <std::floating_point T>
bool Equals(T a, T b, T rel_epsilon = 1e-6, T abs_epsilon = std::numeric_limits<T>::epsilon()) {
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

#endif  // TEST_UTIL_H_
