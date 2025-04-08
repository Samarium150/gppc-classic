#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>

#include "embedding.h"
#include "jps.h"
#include "test_util.h"

static size_t init = 0;
constexpr size_t MAX_EMBEDDING_DIM = 5;

class TestJPSEmbeddings : public testing::TestWithParam<TestCase> {
protected:
    static void SetUpTestSuite() {
        if (!algorithms) {
            const auto num_maps = kMaps.size();
            algorithms = new std::vector<gppc::algorithm::JPS>();
            algorithms->reserve(num_maps);
            embeddings = new std::vector<std::vector<gppc::algorithm::GridEmbedding>>();
            embeddings->reserve(num_maps);
        }
        const auto& [name, data, width, height] = kMaps[init++];
        std::cout << "Build embeddings for Map " << name << std::endl;
        const auto grid = std::make_shared<gppc::Grid>(data, width, height);
        embeddings->emplace_back();
        auto& emb = embeddings->back();
        emb.emplace_back(grid, MAX_EMBEDDING_DIM, gppc::algorithm::GridEmbedding::Metric::kL1);
        emb.emplace_back(grid, MAX_EMBEDDING_DIM, gppc::algorithm::GridEmbedding::Metric::kL1);
        const auto start = std::chrono::high_resolution_clock::now();
        size_t n = 0;
        for (size_t i = 0; i < MAX_EMBEDDING_DIM; ++i) {
            std::cout << "Add dimension " << n++ << std::endl;
            emb[0].AddDimension(gppc::algorithm::GridEmbedding::DimensionType::kDifferential,
                                gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
        }
        for (size_t i = 0; i < MAX_EMBEDDING_DIM - 1; ++i) {
            std::cout << "Add dimension " << n++ << std::endl;
            emb[1].AddDimension(gppc::algorithm::GridEmbedding::DimensionType::kFastMap,
                                gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
        }
        std::cout << "Add dimension " << n++ << std::endl;
        emb[1].AddDimension(gppc::algorithm::GridEmbedding::DimensionType::kDifferential,
                            gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
        const auto end = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
        std::cout << "Time Elapsed: " << duration << "s" << std::endl;
        algorithms->emplace_back(grid);
        algorithms->back().SetHeuristic([&emb, grid](const auto& a, const auto& b) {
            return std::max(grid->HCost(a, b), std::max(emb[0].HCost(a, b), emb[1].HCost(a, b)));
        });
    }

    static void TearDownTestSuite() {
        embeddings->at(init - 1).clear();
        algorithms->at(init - 1) = {};
        if (init == kMaps.size()) {
            delete embeddings;
            embeddings = nullptr;
            delete algorithms;
            algorithms = nullptr;
        }
    }

    static std::vector<gppc::algorithm::JPS>* algorithms;
    static std::vector<std::vector<gppc::algorithm::GridEmbedding>>* embeddings;
};

std::vector<gppc::algorithm::JPS>* TestJPSEmbeddings::algorithms = nullptr;
std::vector<std::vector<gppc::algorithm::GridEmbedding>>* TestJPSEmbeddings::embeddings = nullptr;

TEST_P(TestJPSEmbeddings, TestInstance) {
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

INSTANTIATE_TEST_SUITE_P(IH_2P01, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_2p_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_2P02, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_2p_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_2P03, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_2p_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_2P04, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_2p_04.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_4P01, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_4p_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_4P02, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_4p_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_4P03, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_4p_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_6P01, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_6p_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_6P02, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_6p_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_6P03, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_mp_6p_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_CHA01, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_cha_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_CHA02, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_cha_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_CHA03, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_cha_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_CHA04, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_cha_04.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(
    IH_END, TestJPSEmbeddings,
    testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_endmaps.map")),  // NOLINT
    GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_POL01, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_pol_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_POL02, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_pol_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_POL03, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_pol_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_POL04, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_pol_04.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_POL05, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_pol_05.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_POL06, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_pol_06.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS01, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS02, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS03, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS04, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_04.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS05, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_05.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS06, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_06.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_RUS07, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_rus_07.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX01, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_01.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX02, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_02.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX03, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_03.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX04, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_04.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX05, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_05.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX06, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_06.map")),
                         GenerateTestName);
INSTANTIATE_TEST_SUITE_P(IH_SAX07, TestJPSEmbeddings,
                         testing::ValuesIn(LoadTestCases("iron-harvest/scene_sp_sax_07.map")),
                         GenerateTestName);
