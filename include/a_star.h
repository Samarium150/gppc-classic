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

#ifndef GPPC_A_STAR_H_
#define GPPC_A_STAR_H_

#include "fp_util.h"
#include "grid.h"
#include "search.h"

namespace gppc::algorithm {

struct alignas(64) AstarNode {
    size_t id = std::numeric_limits<size_t>::max();
    size_t parent_id = std::numeric_limits<size_t>::max();
    double g = std::numeric_limits<double>::max();
    double h = std::numeric_limits<double>::max();
    double f = std::numeric_limits<double>::max();

    auto operator<=>(const AstarNode& other) const noexcept {
        if (const auto order = FPCompare(other.f, f); order != std::partial_ordering::equivalent) {
            return order;
        }
        return FPCompare(g, other.g);
    }
};

class AStar final : public HeuristicSearch<AstarNode> {
public:
    AStar() = default;

    AStar(const std::vector<bool>& map, size_t width, size_t height) noexcept;

    explicit AStar(const std::shared_ptr<Grid>& grid) noexcept;

    ~AStar() noexcept override = default;

    bool AddStart(const Point& point) noexcept;

    bool Init(const Point& start, const Point& goal) noexcept override;

    bool Init(const std::shared_ptr<Grid>& grid, const Point& start,
              const Point& goal) noexcept override;

    bool operator()() noexcept override;

    bool operator()(const Point& start, const Point& goal) noexcept override;

    bool operator()(const std::shared_ptr<Grid>& grid, const Point& start,
                    const Point& goal) noexcept override;

private:
    void GetSuccessors(int x, int y) noexcept;

    std::vector<Point> successors_{};
};

}  // namespace gppc::algorithm

#endif  // GPPC_A_STAR_H_
