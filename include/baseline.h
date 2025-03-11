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

#ifndef GPPC_BASELINE_SEARCH_H_
#define GPPC_BASELINE_SEARCH_H_

#include <cstdint>
#include <limits>
#include <memory_resource>
#include <vector>

namespace gppc::baseline {

constexpr uint32_t COST_0 = 1'000;
constexpr uint32_t COST_1 = 1'414;

using Point = std::pair<int, int>;
struct Node {
    static constexpr uint32_t INV = std::numeric_limits<uint32_t>::max();
    static constexpr uint32_t FLOOD_FILL = INV - 1;
    static constexpr uint32_t NO_PRED = INV - 2;
    uint32_t pred;
    uint32_t cost;
};

class Grid {
public:
    Grid(const std::vector<bool>& cells, int width, int height);

    size_t Size() const noexcept;
    uint32_t Pack(const Point& p) const noexcept;
    Point Unpack(uint32_t p) const noexcept;
    bool Get(uint32_t p) const noexcept;
    bool Get(const Point& p) const noexcept;

    uint32_t Width() const noexcept;
    uint32_t Height() const noexcept;
    const std::vector<bool>& Cells() const noexcept;
    std::vector<Node>& Nodes() const noexcept;

protected:
    uint32_t width_;
    uint32_t height_;
    const std::vector<bool> cells_;
    mutable std::vector<Node> nodes_;
};

void SetupGrid(Grid& grid);

class SpanningTreeSearch : public Grid {
public:
    SpanningTreeSearch(const std::vector<bool>& cells, int width, int height);

    const std::vector<Point>& GetPath() const noexcept;

    bool Search(const Point& start, const Point& goal);

private:
    std::array<std::vector<Point>, 2> path_parts_;
};

void FloodFill(Grid& grid, std::pmr::vector<Point>& out, uint32_t origin,
               std::pmr::memory_resource* res);

enum class Compass : uint32_t {
    N = 0b000'000'010,
    E = 0b000'100'000,
    S = 0b010'000'000,
    W = 0b000'001'000,
    NE = 0b000'000'100 | N | E,
    NW = 0b000'000'001 | N | W,
    SE = 0b100'000'000 | S | E,
    SW = 0b001'000'000 | S | W,
};

void Dijkstra(Grid& grid, uint32_t origin, std::pmr::memory_resource* res);

}  // namespace gppc::baseline

#endif  // GPPC_BASELINE_SEARCH_H_
