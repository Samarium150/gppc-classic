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

#include <limits>
#include <vector>

#include "grid.h"
#include "open_closed_list.h"

namespace gppc::algorithm {

class JPS {
public:
    JPS() noexcept = default;

    JPS(const std::vector<bool>& map, size_t width, size_t height) noexcept;

    void PreProcess() noexcept;

    [[nodiscard]] const std::vector<Point>& GetPath() const noexcept;

    JPS& SetJumpLimit(size_t limit) noexcept;

    JPS& SetHeuristic(std::function<double(const Point&, const Point&)> heuristic) noexcept;

    JPS& SetPhi(std::function<double(double, double)> phi) noexcept;

    void Reset() noexcept;

    bool Init(const Point& start, const Point& goal) noexcept;

    bool Init(const std::shared_ptr<Grid>& grid, const Point& start, const Point& goal) noexcept;

    bool operator()() noexcept;

    bool operator()(const Point& start, const Point& goal) noexcept;

    bool operator()(const std::shared_ptr<Grid>& grid, const Point& start,
                    const Point& goal) noexcept;

private:
    struct alignas(64) Node {
        size_t id = std::numeric_limits<size_t>::max();
        std::pair<size_t, uint8_t> parent = {std::numeric_limits<size_t>::max(), kAll};
        double g = std::numeric_limits<double>::max();
        double h = std::numeric_limits<double>::max();
        double f = std::numeric_limits<double>::max();

        bool operator>(const Node& other) const noexcept { return f > other.f; }
    };

    struct alignas(64) Successor {
        Point point;
        uint8_t direction;
        double cost;
    };

    void SetJumpPoint(int x, int y) noexcept;

    [[nodiscard]] bool IsJumpPoint(int x, int y) const noexcept;

    void GetSuccessors(int x, int y, uint8_t parent_dir, double current_cost = 0,
                       size_t jump_count = 0) noexcept;

    void Interpolate(const std::vector<Point>& path) noexcept;

    std::shared_ptr<Grid> grid_ = nullptr;
    std::vector<bool> jump_points_{};
    size_t node_expanded_{};
    std::vector<Point> path_{};
    size_t start_id_ = std::numeric_limits<size_t>::max();
    size_t goal_id_ = std::numeric_limits<size_t>::max();
    Point goal_{-1, -1};
    OpenClosedList<Node, std::greater<>> open_closed_list_{};
    std::vector<Successor> successors_{};
    size_t jump_limit_ = std::numeric_limits<size_t>::max();

    std::function<double(const Point&, const Point&)> heuristic_ =
        [this](const Point& a, const Point& b) { return grid_ ? grid_->HCost(a, b) : 0.0; };

    std::function<double(double, double)> phi_ = [](const double h, const double g) {
        return g + h;
    };
};

}  // namespace gppc::algorithm

#endif  // GPPC_JPS_H_
