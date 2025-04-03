#include <gtest/gtest.h>

#include <chrono>

#include "jps.h"
#include "test_util.h"

class TestJPSOctile : public testing::TestWithParam<TestCase> {  // NOLINT
protected:
    static void SetUpTestSuite() {
        const auto num_maps = kMaps.size();
        algorithms = new std::vector<gppc::algorithm::JPS>();
        algorithms->reserve(num_maps);
        std::ranges::for_each(kMaps, [](const auto& map) {
            const auto& [name, data, width, height] = map;
            algorithms->emplace_back(data, width, height);
        });
    }

    static void TearDownTestSuite() { delete algorithms; }

    static std::vector<gppc::algorithm::JPS>* algorithms;
};

std::vector<gppc::algorithm::JPS>* TestJPSOctile::algorithms = nullptr;

TEST_P(TestJPSOctile, TestInstance) {
    const auto& param = GetParam();
    auto& algo = algorithms->at(param.map_id);
    std::cout << param.case_id << ": " << param.start << " -> " << param.goal << std::endl;
    const auto start = std::chrono::high_resolution_clock::now();
    ASSERT_TRUE(algo(param.start, param.goal) || param.expected_length == 0);
    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time Elapsed: " << duration << std::endl;
    const auto length = GetPathLength(algo.GetPath());
    ASSERT_TRUE(Equals(length, param.expected_length, 1e-5));
    std::cout << "Path length: " << length << std::endl;
}

INSTANTIATE_TEST_SUITE_P(HOG2, TestJPSOctile, testing::ValuesIn(LoadTestCases("data")),
                         GenerateTestName);
