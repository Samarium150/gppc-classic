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

#ifndef GPPC_GRID_EMBEDDING_H_
#define GPPC_GRID_EMBEDDING_H_

#include "a_star.h"

namespace gppc::algorithm {

class ResidualGrid final : public Grid {
public:
    explicit ResidualGrid(const Grid& grid,
                          std::function<double(const Point& a, const Point& b)> heuristic) noexcept;

    [[nodiscard]] double HCost(const Point& a, const Point& b) const noexcept override;

    [[nodiscard]] double GCost(const Point& a, const Point& b) const noexcept override;

private:
    std::function<double(const Point& a, const Point& b)> heuristic_;
};

class GridEmbedding {
public:
    enum class Metric { kL1, kLInf };

    enum class DimensionType { kDifferential, kFastMap };

    enum class PivotPlacement { kFarthest, kHeuristicError };

    GridEmbedding(const std::shared_ptr<Grid>& grid, size_t max_dimensions, Metric metric) noexcept;

    bool AddDimension(DimensionType type, PivotPlacement placement) noexcept;

    [[nodiscard]] double HCost(const Point& a, const Point& b) const noexcept;

private:
    void GetConnectedComponents() noexcept;
    [[nodiscard]] Point GetRandomState(uint8_t component) const noexcept;
    static Point GetFarthestState(const Point& point, AStar& astar,
                                  const std::shared_ptr<Grid>& grid) noexcept;
    static Point GetFarthestState(const std::vector<Point>& points, AStar& astar,
                                  const std::shared_ptr<Grid>& grid) noexcept;
    void SelectPivots(PivotPlacement placement, uint8_t component) noexcept;
    void Embed(DimensionType type, uint8_t component) noexcept;
    std::shared_ptr<Grid> grid_;
    std::shared_ptr<ResidualGrid> residual_grid_;
    size_t max_dimensions_;
    Metric metric_;
    AStar s1{};
    AStar s2{};
    size_t current_dimensions_{};
    std::vector<double> embedding_{};
    uint8_t num_connected_components_{};
    std::vector<uint8_t> connected_components_{};
    std::vector<std::vector<Point>> pivots_{};
};

}  // namespace gppc::algorithm

#endif  // GPPC_GRID_EMBEDDING_H_
