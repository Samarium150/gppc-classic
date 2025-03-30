/**
 * Copyright (c) 2023 Grid-based Path Planning Competition and Contributors
 * <https://gppc.search-conference.org/>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "entry.h"

#include "a_star.h"
#include "embedding.h"

struct SearchData {
    std::shared_ptr<gppc::algorithm::AStar> search = nullptr;
    std::array<std::shared_ptr<gppc::algorithm::GridEmbedding>, 2> embeddings = {};
};

constexpr uint8_t kMaxDim = 5;

void PreprocessMap(const std::vector<bool> & /*bits*/, const int /*width*/, const int /*height*/,
                   const std::string & /*filename*/) {}

void *PrepareForSearch(const std::vector<bool> &bits, const int width, const int height,
                       const std::string & /*filename*/) {
    const auto grid = std::make_shared<gppc::Grid>(bits, width, height);
    auto dh5 = std::make_shared<gppc::algorithm::GridEmbedding>(
        grid, kMaxDim, gppc::algorithm::GridEmbedding::Metric::kL1);
    for (size_t i = 0; i < kMaxDim; ++i) {
        dh5->AddDimension(gppc::algorithm::GridEmbedding::DimensionType::kDifferential,
                          gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
    }
    auto fm4dh = std::make_shared<gppc::algorithm::GridEmbedding>(
        grid, kMaxDim, gppc::algorithm::GridEmbedding::Metric::kL1);
    for (size_t i = 0; i < kMaxDim - 1; ++i) {
        fm4dh->AddDimension(gppc::algorithm::GridEmbedding::DimensionType::kFastMap,
                            gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
    }
    fm4dh->AddDimension(gppc::algorithm::GridEmbedding::DimensionType::kDifferential,
                        gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
    return new SearchData{.search = std::make_shared<gppc::algorithm::AStar>(bits, width, height),
                          .embeddings = std::array{std::move(dh5), std::move(fm4dh)}};
}

// NOLINTNEXTLINE
bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path) {
    const auto search_data = static_cast<SearchData *>(data);
    const auto search = search_data->search;
    const auto &embeddings = search_data->embeddings;
    search->SetHeuristic([&embeddings](const auto &a, const auto &b) {
        double h = 0;
        for (const auto &embedding : embeddings) {
            h = std::max(h, embedding->HCost(a, b));
        }
        return h;
    });
    path.clear();
    if (const bool exists = (*search)({s.x, s.y}, {g.x, g.y}); !exists) {
        return true;
    }
    for (auto [x, y] : search->GetPath()) {
        path.emplace_back(x, y);
    }
    return true;
}

std::string GetName() { return "A*"; }
