#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>

#include "jps.h"
#include "test_util.h"

static size_t init = 0;

class TestJPSOctile : public testing::TestWithParam<TestCase> {  // NOLINT
protected:
    static void SetUpTestSuite() {
        if (!algorithms) {
            algorithms = new std::vector<gppc::algorithm::JPS>();
            algorithms->reserve(kMaps.size());
        }
        const auto& [name, data, width, height] = kMaps[init++];
        algorithms->emplace_back(data, width, height);
    }

    static void TearDownTestSuite() {
        algorithms->at(init - 1) = {};
        if (init == kMaps.size()) {
            delete algorithms;
            algorithms = nullptr;
        }
    }

    static std::vector<gppc::algorithm::JPS>* algorithms;
};

std::vector<gppc::algorithm::JPS>* TestJPSOctile::algorithms = nullptr;

TEST_P(TestJPSOctile, TestInstance) {
    const auto& param = GetParam();
    std::cout << std::string(param) << std::endl;
    auto& algorithm = (*algorithms)[param.map_id];
    const auto start = std::chrono::high_resolution_clock::now();
    ASSERT_TRUE(algorithm(param.start, param.goal) || param.expected_length == 0);
    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Time Elapsed: " << duration << "ms" << std::endl;
    const auto length = GetPathLength(algorithm.GetPath());
    ASSERT_TRUE(Equals(length, param.expected_length, 1e-5));
    std::cout << "Path length: " << length << std::endl;
}

INSTANTIATE_TEST_SUITE_P(IH_2P01, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_2p_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_2P02, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_2p_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_2P03, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_2p_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_2P04, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_2p_04.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_4P01, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_4p_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_4P02, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_4p_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_4P03, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_4p_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_6P01, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_6p_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_6P02, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_6p_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_6P03, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_6p_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_CHA01, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_cha_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_CHA02, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_cha_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_CHA03, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_cha_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_CHA04, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_cha_04.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(
    IH_END, TestJPSOctile,
    testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_endmaps.map")),  // NOLINT
    GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_POL01, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_pol_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_POL02, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_pol_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_POL03, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_pol_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_POL04, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_pol_04.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_POL05, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_pol_05.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_POL06, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_pol_06.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS01, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS02, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS03, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS04, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_04.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS05, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_05.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS06, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_06.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS07, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_07.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX01, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX02, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX03, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX04, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_04.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX05, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_05.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX06, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_06.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX07, TestJPSOctile,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_07.map")),
                         GenerateTestName);
