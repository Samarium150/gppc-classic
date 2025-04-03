#include <gtest/gtest.h>

#include <chrono>

#include "embedding.h"
#include "jps.h"
#include "test_util.h"

constexpr size_t MAX_EMBEDDING_DIM = 10;

class TestJPSEmbeddings : public testing::TestWithParam<TestCase> {
protected:
    static void SetUpTestSuite() {
        const auto num_maps = kMaps.size();
        algorithms = new std::vector<gppc::algorithm::AStar>();
        algorithms->reserve(num_maps);
        embeddings = new std::vector<std::vector<gppc::algorithm::GridEmbedding>>();
        embeddings->reserve(num_maps);
        std::ranges::for_each(kMaps, [](const auto& map) {
            const auto& [name, data, width, height] = map;
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
                emb[0].AddDimension(
                    gppc::algorithm::GridEmbedding::DimensionType::kDifferential,
                    gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
            }
            for (size_t i = 0; i < MAX_EMBEDDING_DIM - 1; ++i) {
                std::cout << "Add dimension " << n++ << std::endl;
                emb[1].AddDimension(
                    gppc::algorithm::GridEmbedding::DimensionType::kFastMap,
                    gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
            }
            std::cout << "Add dimension " << n++ << std::endl;
            emb[1].AddDimension(gppc::algorithm::GridEmbedding::DimensionType::kDifferential,
                                gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
            const auto end = std::chrono::high_resolution_clock::now();
            const auto duration =
                std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            std::cout << "Time Elapsed: " << duration << std::endl;
            algorithms->emplace_back(grid);
            algorithms->back().SetHeuristic([&emb](const auto& a, const auto& b) {
                return std::max(emb[0].HCost(a, b), emb[1].HCost(a, b));
            });
        });
    }

    static void TearDownTestSuite() {
        delete algorithms;
        delete embeddings;
    }

    static std::vector<gppc::algorithm::AStar>* algorithms;
    static std::vector<std::vector<gppc::algorithm::GridEmbedding>>* embeddings;
};

std::vector<gppc::algorithm::AStar>* TestJPSEmbeddings::algorithms = nullptr;
std::vector<std::vector<gppc::algorithm::GridEmbedding>>* TestJPSEmbeddings::embeddings = nullptr;

TEST_P(TestJPSEmbeddings, TestInstance) {
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

INSTANTIATE_TEST_SUITE_P(IH, TestJPSEmbeddings, testing::ValuesIn(LoadTestCases("iron-harvest")),
                         GenerateTestName);
