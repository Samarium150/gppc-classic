/**
 * Copyright (c) 2025 Samarium
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

#include "embedding.h"

#include <random>

namespace gppc::algorithm {

ResidualGrid::ResidualGrid(const Grid& grid,
                           std::function<double(const Point& a, const Point& b)> heuristic) noexcept
    : Grid(grid), heuristic_(std::move(heuristic)) {}

double ResidualGrid::HCost(const Point& /*a*/, const Point& /*b*/) const noexcept { return 0.0; }

double ResidualGrid::GCost(const Point& a, const Point& b) const noexcept {
    return std::max(Grid::GCost(a, b) - heuristic_(a, b), 0.0);
}

GridEmbedding::GridEmbedding(const std::shared_ptr<Grid>& grid, const size_t max_dimensions,
                             const Metric metric) noexcept
    : grid_(grid),
      residual_grid_(std::make_shared<ResidualGrid>(
          *grid, [this](const Point& a, const Point& b) { return this->HCost(a, b); })),
      max_dimensions_(max_dimensions),
      metric_(metric),
      embedding_(grid_->Size() * max_dimensions_, -1) {
    s1.StopAfterGoal(false).SetHeuristic([](const Point&, const Point&) { return 0.0; });
    s2.StopAfterGoal(false).SetHeuristic([](const Point&, const Point&) { return 0.0; });
    GetConnectedComponents();
}

void GridEmbedding::GetConnectedComponents() noexcept {
    num_connected_components_ = 0;
    connected_components_.resize(grid_->Size(), std::numeric_limits<uint8_t>::max());
    for (size_t i = 0; i < grid_->Size(); ++i) {
        if (!grid_->Get(i) || connected_components_[i] != std::numeric_limits<uint8_t>::max()) {
            continue;
        }
        const auto point = grid_->Unpack(i);
        s1(grid_, point, point);
        for (auto& node : s1.GetNodes()) {
            if (node.id != std::numeric_limits<size_t>::max()) {
                connected_components_[node.id] = num_connected_components_;
            }
        }
        ++num_connected_components_;
    }
    pivots_.resize(num_connected_components_);
}

bool GridEmbedding::AddDimension(const DimensionType type,
                                 const PivotPlacement placement) noexcept {
    if (current_dimensions_ == max_dimensions_) {
        return false;
    }
    for (auto component = 0; component < num_connected_components_; ++component) {
        SelectPivots(placement, component);
        Embed(type, component);
    }
    ++current_dimensions_;
    return true;
}

double GridEmbedding::HCost(const Point& a, const Point& b) const noexcept {
    double h = 0.0;
    const auto a_id = grid_->Pack(a);
    const auto b_id = grid_->Pack(b);
    for (size_t i = 0; i < current_dimensions_; ++i) {
        switch (metric_) {
            case Metric::kL1: {
                h += std::fabs(embedding_[a_id * max_dimensions_ + i] -
                               embedding_[b_id * max_dimensions_ + i]);
                break;
            }
            case Metric::kLInf: {
                h = std::max(h, std::fabs(embedding_[a_id * max_dimensions_ + i] -
                                          embedding_[b_id * max_dimensions_ + i]));
                break;
            }
        }
    }
    return h;
}

void GridEmbedding::SelectPivots(const PivotPlacement placement, const uint8_t component) noexcept {
    switch (metric_) {
        case Metric::kL1: {
            switch (placement) {
                case PivotPlacement::kFarthest: {
                    const auto rand = GetRandomState(component);
                    const auto p1 = GetFarthestState(rand, s2, residual_grid_);
                    const auto p2 = GetFarthestState(p1, s1, residual_grid_);
                    GetFarthestState(p2, s2, residual_grid_);
                    pivots_[component].push_back(p1);
                    pivots_[component].push_back(p2);
                    break;
                }
                case PivotPlacement::kHeuristicError: {
                    const auto rand = GetRandomState(component);
                    auto p1 = GetFarthestState(rand, s2, residual_grid_);
                    double max_he = 0;
                    for (size_t id = 0; id < grid_->Size(); ++id) {
                        if (connected_components_[id] != component) {
                            continue;
                        }
                        auto candidate = grid_->Unpack(id);
                        const double g = s2.GetNodes()[id].g;
                        if (const double he = 3 * g - 2 * grid_->HCost(rand, candidate);
                            he > max_he) {
                            max_he = he;
                            p1 = candidate;
                        }
                    }
                    auto p2 = GetFarthestState(p1, s1, residual_grid_);
                    max_he = 0;
                    for (size_t id = 0; id < grid_->Size(); ++id) {
                        if (connected_components_[id] != component) {
                            continue;
                        }
                        auto candidate = grid_->Unpack(id);
                        const double g = s1.GetNodes()[id].g;
                        if (const auto he = 3 * g - 2 * grid_->HCost(p1, candidate); he > max_he) {
                            max_he = he;
                            p2 = candidate;
                        }
                    }
                    GetFarthestState(p2, s2, residual_grid_);
                    pivots_[component].push_back(p1);
                    pivots_[component].push_back(p2);
                    break;
                }
            }
            break;
        }
        case Metric::kLInf: {
            switch (placement) {
                case PivotPlacement::kFarthest: {
                    Point pivot;
                    if (pivots_[component].empty()) {
                        pivot = GetFarthestState(GetRandomState(component), s1, grid_);
                    } else {
                        pivot = GetFarthestState(pivots_[component], s1, grid_);
                    }
                    pivots_[component].push_back(pivot);
                    GetFarthestState(pivot, s2, grid_);
                    break;
                }
                case PivotPlacement::kHeuristicError: {
                    break;
                }
            }
            break;
        }
    }
}

void GridEmbedding::Embed(const DimensionType type, const uint8_t component) noexcept {
    for (size_t id = 0; id < grid_->Size(); ++id) {
        if (connected_components_[id] != component) {
            continue;
        }
        if (const auto dp1 = s1.GetNodes()[id].g; dp1 != std::numeric_limits<double>::max()) {
            const auto index = id * max_dimensions_ + current_dimensions_;
            switch (type) {
                case DimensionType::kDifferential: {
                    embedding_[index] = dp1;
                    break;
                }
                case DimensionType::kFastMap: {
                    const auto dp1p2 = s1.GetNodes()[grid_->Pack(pivots_[component].back())].g;
                    const auto dp2 = s2.GetNodes()[id].g;
                    embedding_[index] = (dp1 + dp1p2 - dp2) / 2;
                    break;
                }
            }
        }
    }
}

Point GridEmbedding::GetRandomState(const uint8_t component) const noexcept {
    while (true) {
        thread_local std::mt19937 gen{std::random_device()()};
        std::uniform_int_distribution<size_t> distribution(0, grid_->Size() - 1);
        if (const auto id = distribution(gen); connected_components_[id] == component) {
            return grid_->Unpack(id);
        }
    }
}

Point GridEmbedding::GetFarthestState(const Point& point, AStar& astar,
                                      const std::shared_ptr<Grid>& grid) noexcept {
    astar.Init(grid, point, point);
    Point farthest_point = point;
    while (!astar()) {
        if (!astar.EmptyOpen()) {
            farthest_point = grid->Unpack(astar.PeekOpen().id);
        }
    }
    return farthest_point;
}

Point GridEmbedding::GetFarthestState(const std::vector<Point>& points, AStar& astar,
                                      const std::shared_ptr<Grid>& grid) noexcept {
    astar.Init(grid, points[0], points[0]);
    for (size_t i = 1; i < points.size(); ++i) {
        astar.AddStart(points[i]);
    }
    Point farthest_point = points[0];
    while (!astar()) {
        if (!astar.EmptyOpen()) {
            farthest_point = grid->Unpack(astar.PeekOpen().id);
        }
    }
    return farthest_point;
}

}  // namespace gppc::algorithm
