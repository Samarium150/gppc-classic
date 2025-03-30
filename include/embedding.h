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

#include "jps.h"

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
    enum class Metric : uint8_t { kL1, kLInf };

    enum class DimensionType : uint8_t { kDifferential, kFastMap };

    enum class PivotPlacement : uint8_t { kFarthest, kHeuristicError };

    GridEmbedding() noexcept = default;

    GridEmbedding(const std::shared_ptr<Grid>& grid, size_t max_dimensions, Metric metric) noexcept;

    bool AddDimension(DimensionType type, PivotPlacement placement) noexcept;

    [[nodiscard]] double HCost(const Point& a, const Point& b) const noexcept;

    friend std::ofstream& operator<<(std::ofstream& ofs, const GridEmbedding& embedding) noexcept;

    friend std::ifstream& operator>>(std::ifstream& ifs, GridEmbedding& embedding) noexcept;

    bool operator==(const GridEmbedding& other) const noexcept;

private:
    void FloodFill(const Point& point) noexcept;

    void GetConnectedComponents() noexcept;

    [[nodiscard]] Point GetRandomState(uint8_t component) const noexcept;

    static Point GetFarthestState(const Point& point, JPS& search,
                                  const std::shared_ptr<Grid>& grid) noexcept;

    static Point GetFarthestState(const std::vector<Point>& points, JPS& search,
                                  const std::shared_ptr<Grid>& grid) noexcept;

    void SelectPivots(PivotPlacement placement, uint8_t component) noexcept;

    void Embed(DimensionType type, uint8_t component) noexcept;

    static constexpr auto MAX_NUM_COMPONENTS = std::numeric_limits<uint16_t>::max();
    std::shared_ptr<Grid> grid_ = nullptr;
    std::shared_ptr<ResidualGrid> residual_grid_ = nullptr;
    size_t max_dimensions_{};
    Metric metric_{};
    JPS s1{};
    JPS s2{};
    size_t current_dimensions_{};
    std::vector<double> embedding_{};
    std::queue<Point> queue_;
    uint16_t num_connected_components_{};
    std::vector<uint16_t> connected_components_{};
    std::vector<std::vector<size_t>> component_indices_{};
    std::vector<std::vector<Point>> pivots_{};
};

}  // namespace gppc::algorithm

#endif  // GPPC_GRID_EMBEDDING_H_
