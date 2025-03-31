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

#ifndef GPPC_JPS_H_
#define GPPC_JPS_H_

#include <stack>

#include "grid.h"
#include "open_closed_list.h"
#include "search.h"

namespace gppc::algorithm {

struct alignas(64) JPSNode {
    size_t id = std::numeric_limits<size_t>::max();
    std::pair<size_t, uint8_t> parent = {std::numeric_limits<size_t>::max(), kAll};
    double g = std::numeric_limits<double>::max();
    double f = std::numeric_limits<double>::max();

    bool operator>(const JPSNode& other) const noexcept { return f > other.f; }
};

class JPS final : public HeuristicSearch<JPSNode, std::greater<>> {
public:
    JPS() noexcept = default;

    JPS(const std::vector<bool>& map, size_t width, size_t height) noexcept;

    explicit JPS(const std::shared_ptr<Grid>& grid) noexcept;

    ~JPS() noexcept override = default;

    JPS& SetJumpLimit(unsigned limit) noexcept;

    bool Init(const Point& start, const Point& goal) noexcept override;

    bool Init(const std::shared_ptr<Grid>& grid, const Point& start,
              const Point& goal) noexcept override;

    bool AddStart(const Point& point) noexcept;

    bool operator()() noexcept override;

    bool operator()(const Point& start, const Point& goal) noexcept override;

    bool operator()(const std::shared_ptr<Grid>& grid, const Point& start,
                    const Point& goal) noexcept override;

private:
    struct alignas(64) Successor {
        int x;
        int y;
        uint8_t direction;
        double cost;
    };

    void PreProcess() noexcept;

    void SetJumpPoint(int x, int y) noexcept;

    [[nodiscard]] bool IsJumpPoint(int x, int y) const noexcept;

    void GetSuccessors(int x, int y, uint8_t parent_dir, double current_cost = 0) noexcept;

    void Interpolate(const std::vector<Point>& path) noexcept;

    int width_{};
    int height_{};
    unsigned jump_limit_ = std::numeric_limits<unsigned>::max();
    std::vector<bool> jump_points_{};
    std::vector<Successor> successors_{};
    std::stack<Successor> temp_successors_{};
};

}  // namespace gppc::algorithm

#endif  // GPPC_JPS_H_
