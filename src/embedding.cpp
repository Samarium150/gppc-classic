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

#include <algorithm>
#include <fstream>
#include <random>

namespace gppc::algorithm {

ResidualGrid::ResidualGrid(const Grid& grid) noexcept : Grid(grid) {}

ResidualGrid::ResidualGrid(const Grid& grid,
                           std::function<double(const Point&, const Point&)> heuristic) noexcept
    : Grid(grid), heuristic_(std::move(heuristic)) {}

void ResidualGrid::SetHeuristic(
    std::function<double(const Point&, const Point&)> heuristic) noexcept {
    heuristic_ = std::move(heuristic);
}

double ResidualGrid::HCost(const Point&, const Point&) const noexcept { return 0.0; }

double ResidualGrid::GCost(const Point& a, const Point& b) const noexcept {
    return std::max(Grid::GCost(a, b) - heuristic_(a, b), 0.0);
}

GridEmbedding::GridEmbedding(const std::shared_ptr<Grid>& grid) noexcept : grid_(grid) {}

GridEmbedding::GridEmbedding(const std::shared_ptr<Grid>& grid, const size_t max_dimensions,
                             const Metric metric) noexcept
    : grid_(grid),
      residual_grid_(std::make_shared<ResidualGrid>(*grid)),
      max_dimensions_(max_dimensions),
      metric_(metric),
      embedding_(grid_->Size() * max_dimensions_, 0) {
    s1.StopAfterGoal(false).SetHeuristic([](const Point&, const Point&) { return 0.0; });
    s2.StopAfterGoal(false).SetHeuristic([](const Point&, const Point&) { return 0.0; });
    FindLargestComponent();
}

bool GridEmbedding::AddDimension(const DimensionType type,
                                 const PivotPlacement placement) noexcept {
    if (current_dimensions_ == max_dimensions_) {
        return false;
    }
    residual_grid_->SetHeuristic([this](const Point& a, const Point& b) { return HCost(a, b); });
    SelectPivots(placement);
    Embed(type);
    ++current_dimensions_;
    return true;
}

double GridEmbedding::HCost(const Point& a, const Point& b) const noexcept {
    if (!grid_) {
        return 0;
    }
    double h = 0;
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

std::ofstream& operator<<(std::ofstream& ofs, const GridEmbedding& embedding) noexcept {
    ofs.write(reinterpret_cast<const char*>(&embedding.max_dimensions_),
              sizeof(embedding.max_dimensions_))
        .write(reinterpret_cast<const char*>(&embedding.current_dimensions_),
               sizeof(embedding.current_dimensions_))
        .write(reinterpret_cast<const char*>(&embedding.metric_), sizeof(embedding.metric_));
    const auto embedding_size = embedding.embedding_.size();
    ofs.write(reinterpret_cast<const char*>(&embedding_size), sizeof(embedding_size))
        .write(reinterpret_cast<const char*>(embedding.embedding_.data()),
               static_cast<std::streamsize>(embedding.embedding_.size() * sizeof(double)));
    return ofs;
}

std::ifstream& operator>>(std::ifstream& ifs, GridEmbedding& embedding) noexcept {
    ifs.read(reinterpret_cast<char*>(&embedding.max_dimensions_), sizeof(embedding.max_dimensions_))
        .read(reinterpret_cast<char*>(&embedding.current_dimensions_),
              sizeof(embedding.current_dimensions_))
        .read(reinterpret_cast<char*>(&embedding.metric_), sizeof(embedding.metric_));
    size_t embedding_size;
    ifs.read(reinterpret_cast<char*>(&embedding_size), sizeof(embedding_size));
    embedding.embedding_.resize(embedding_size, -1);
    ifs.read(reinterpret_cast<char*>(embedding.embedding_.data()),
             static_cast<std::streamsize>(embedding_size * sizeof(double)));
    return ifs;
}

bool GridEmbedding::operator==(const GridEmbedding& other) const noexcept {
    return max_dimensions_ == other.max_dimensions_ &&
           current_dimensions_ == other.current_dimensions_ && metric_ == other.metric_ &&
           embedding_ == other.embedding_;
}

void GridEmbedding::FloodFill(const Point& point, std::vector<size_t>& component) noexcept {
    std::vector visited(grid_->Size(), false);
    queue_.push(point);
    const auto start_id = grid_->Pack(point);
    visited[start_id] = true;
    component.push_back(start_id);

    while (!queue_.empty()) {
        const auto [x, y] = queue_.front();
        queue_.pop();
        for (const auto& direction : kDirections) {
            const auto [dx, dy] = Grid::GetOffset(direction);
            const auto xx = x + dx;
            const auto yy = y + dy;
            const auto successor = Point{xx, yy};
            if (!grid_->Get(successor)) {
                continue;
            }
            if (dx != 0 && dy != 0 && (!grid_->Get(xx, y) || !grid_->Get(x, yy))) {
                continue;
            }
            const auto successor_id = grid_->Pack(xx, yy);
            if (visited[successor_id]) {
                continue;
            }
            visited[successor_id] = true;
            component.push_back(successor_id);
            queue_.push(successor);
        }
    }
}

void GridEmbedding::FindLargestComponent() noexcept {
    if (!grid_) {
        return;
    }
    std::vector<std::vector<size_t>> components;
    std::vector visited(grid_->Size(), false);
    for (size_t id = 0; id < grid_->Size(); ++id) {
        if (!grid_->Get(id) || visited[id]) {
            continue;
        }
        std::vector<size_t> component;
        FloodFill(grid_->Unpack(id), component);
        for (const auto& node_id : component) {
            visited[node_id] = true;
        }
        components.push_back(std::move(component));
    }
    if (!components.empty()) {
        const auto largest_it = std::ranges::max_element(
            components, [](const auto& a, const auto& b) { return a.size() < b.size(); });
        largest_component_indices_ = std::move(*largest_it);
        largest_component_indices_.shrink_to_fit();
    }
}

Point GridEmbedding::GetRandomState() const noexcept {
    thread_local std::mt19937 gen{std::random_device()()};
    std::uniform_int_distribution<size_t> distribution(0, largest_component_indices_.size() - 1);
    return grid_->Unpack(largest_component_indices_[distribution(gen)]);
}

Point GridEmbedding::GetFarthestState(const Point& point, AStar& search,
                                      const std::shared_ptr<Grid>& grid) noexcept {
    search.Init(grid, point, point);
    Point farthest_point = point;
    while (!search()) {
        if (!search.EmptyOpen()) {
            farthest_point = grid->Unpack(search.PeekOpen().id);
        }
    }
    return farthest_point;
}

Point GridEmbedding::GetFarthestState(const std::vector<Point>& points, AStar& search,
                                      const std::shared_ptr<Grid>& grid) noexcept {
    search.Init(grid, points[0], points[0]);
    for (size_t i = 1; i < points.size(); ++i) {
        search.AddStart(points[i]);
    }
    Point farthest_point = points[0];
    while (!search()) {
        if (!search.EmptyOpen()) {
            farthest_point = grid->Unpack(search.PeekOpen().id);
        }
    }
    return farthest_point;
}

void GridEmbedding::SelectPivots(const PivotPlacement placement) noexcept {
    switch (metric_) {
        case Metric::kL1: {
            switch (placement) {
                case PivotPlacement::kFarthest: {
                    const auto rand = GetRandomState();
                    const auto p1 = GetFarthestState(rand, s2, residual_grid_);
                    const auto p2 = GetFarthestState(p1, s1, residual_grid_);
                    GetFarthestState(p2, s2, residual_grid_);
                    pivots_.push_back(p1);
                    pivots_.push_back(p2);
                    break;
                }
                case PivotPlacement::kHeuristicError: {
                    const auto rand = GetRandomState();
                    auto p1 = GetFarthestState(rand, s2, residual_grid_);
                    double max_he = 0;
                    for (const auto& id : largest_component_indices_) {
                        auto candidate = grid_->Unpack(id);
                        const double g = s2.GetNode(id).g;
                        if (const double he = 3 * g - 2 * grid_->HCost(rand, candidate);
                            he > max_he) {
                            max_he = he;
                            p1 = candidate;
                        }
                    }
                    auto p2 = GetFarthestState(p1, s1, residual_grid_);
                    max_he = 0;
                    for (const auto& id : largest_component_indices_) {
                        auto candidate = grid_->Unpack(id);
                        const double g = s1.GetNode(id).g;
                        if (const auto he = 3 * g - 2 * grid_->HCost(p1, candidate); he > max_he) {
                            max_he = he;
                            p2 = candidate;
                        }
                    }
                    GetFarthestState(p2, s2, residual_grid_);
                    pivots_.push_back(p1);
                    pivots_.push_back(p2);
                    break;
                }
            }
            break;
        }
        case Metric::kLInf: {
            switch (placement) {
                case PivotPlacement::kFarthest: {
                    const Point pivot = pivots_.empty()
                                            ? GetFarthestState(GetRandomState(), s1, grid_)
                                            : GetFarthestState(pivots_, s1, grid_);
                    pivots_.push_back(pivot);
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

void GridEmbedding::Embed(const DimensionType type) noexcept {
    for (const auto& id : largest_component_indices_) {
        if (const auto dp1 = s1.GetNode(id).g; dp1 != std::numeric_limits<double>::max()) {
            const auto index = id * max_dimensions_ + current_dimensions_;
            switch (type) {
                case DimensionType::kDifferential: {
                    embedding_[index] = dp1;
                    break;
                }
                case DimensionType::kFastMap: {
                    const auto dp1p2 = s1.GetNode(grid_->Pack(pivots_.back())).g;
                    const auto dp2 = s2.GetNode(id).g;
                    embedding_[index] = (dp1 + dp1p2 - dp2) / 2;
                    break;
                }
            }
        }
    }
}

}  // namespace gppc::algorithm
