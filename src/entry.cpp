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

#include <fstream>

#include "embedding.h"
#include "jps.h"
//
// struct SearchData {
//     std::shared_ptr<gppc::algorithm::JPS> search = nullptr;
//     std::vector<std::shared_ptr<gppc::algorithm::GridEmbedding>> embeddings = {};
// };

constexpr uint8_t kMaxDim = 5;

void PreprocessMap(const std::vector<bool> &bits, int width, int height,
                   const std::string &filename) {
    const auto grid = std::make_shared<gppc::Grid>(bits, width, height);
    auto dh5 =
        gppc::algorithm::GridEmbedding(grid, kMaxDim, gppc::algorithm::GridEmbedding::Metric::kL1);
    for (size_t i = 0; i < kMaxDim; ++i) {
        dh5.AddDimension(gppc::algorithm::GridEmbedding::DimensionType::kDifferential,
                         gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
    }
    auto fm4dh =
        gppc::algorithm::GridEmbedding(grid, kMaxDim, gppc::algorithm::GridEmbedding::Metric::kL1);
    for (size_t i = 0; i < kMaxDim - 1; ++i) {
        fm4dh.AddDimension(gppc::algorithm::GridEmbedding::DimensionType::kFastMap,
                           gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
    }
    fm4dh.AddDimension(gppc::algorithm::GridEmbedding::DimensionType::kDifferential,
                       gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs.is_open()) {
        return;
    }
    ofs << dh5;
    ofs << fm4dh;
    ofs.close();
}

void *PrepareForSearch(const std::vector<bool> &bits, const int width, const int height,
                       const std::string &filename) {
    const auto grid = std::make_shared<gppc::Grid>(bits, width, height);
    const auto search = new gppc::algorithm::JPS(grid);
    search->SetJumpLimit(16);
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs.is_open()) {
        return search;
    }
    auto dh5 = std::make_shared<gppc::algorithm::GridEmbedding>(grid);
    ifs >> *dh5;
    auto fm4dh = std::make_shared<gppc::algorithm::GridEmbedding>(grid);
    ifs >> *fm4dh;
    ifs.close();
    search->SetHeuristic([dh5, fm4dh](const auto &a, const auto &b) {
        return std::max(dh5->HCost(a, b), fm4dh->HCost(a, b));
    });
    return search;
}

// NOLINTNEXTLINE
bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path) {
    path.clear();
    const auto search = static_cast<gppc::algorithm::JPS *>(data);
    if (const bool exists = (*search)({s.x, s.y}, {g.x, g.y}); !exists) {
        return true;
    }
    for (const auto &[x, y] : search->GetPath()) {
        path.emplace_back(x, y);
    }
    return true;
}

std::string GetName() { return "JPS+(P)"; }
