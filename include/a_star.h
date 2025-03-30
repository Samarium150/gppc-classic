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

#include <limits>

#include "grid.h"
#include "open_closed_list.h"

namespace gppc::algorithm {

class AStar {
public:
    struct alignas(64) Node {
        size_t id = std::numeric_limits<size_t>::max();
        size_t parent_id = std::numeric_limits<size_t>::max();
        double g = std::numeric_limits<double>::max();
        double h = std::numeric_limits<double>::max();
        double f = std::numeric_limits<double>::max();

        bool operator>(const Node& other) const noexcept { return f > other.f; }
    };

    AStar() noexcept = default;

    AStar(const std::vector<bool>& map, size_t width, size_t height) noexcept;

    [[nodiscard]] const std::vector<Point>& GetPath() const noexcept;

    [[nodiscard]] size_t GetNodeExpanded() const noexcept;

    [[nodiscard]] const std::vector<Node>& GetNodes() const noexcept;

    [[nodiscard]] Node PeekOpen() const noexcept;

    [[nodiscard]] bool EmptyOpen() const noexcept;

    AStar& SetHeuristic(std::function<double(const Point& s1, const Point& s2)> heuristic) noexcept;

    AStar& SetPhi(std::function<double(double h, double g)> phi) noexcept;

    AStar& StopAfterGoal(bool stop) noexcept;

    void Reset() noexcept;

    bool Init(const Point& start, const Point& goal) noexcept;

    bool Init(const std::shared_ptr<Grid>& grid, const Point& start, const Point& goal) noexcept;

    bool AddStart(const Point& point) noexcept;

    bool operator()() noexcept;

    bool operator()(const Point& start, const Point& goal) noexcept;

    bool operator()(const std::shared_ptr<Grid>& grid, const Point& start,
                    const Point& goal) noexcept;

private:
    void GetSuccessors(int x, int y) noexcept;

    std::shared_ptr<Grid> grid_ = nullptr;
    size_t node_expanded_{};
    std::vector<Point> path_{};
    size_t start_id_ = std::numeric_limits<size_t>::max();
    size_t goal_id_ = std::numeric_limits<size_t>::max();
    Point goal_{-1, -1};
    bool stop_after_goal_ = true;
    OpenClosedList<Node, std::greater<>> open_closed_list_{};
    std::vector<Point> successors_{};

    std::function<double(const Point&, const Point&)> heuristic_ =
        [this](const Point& a, const Point& b) { return grid_ ? grid_->HCost(a, b) : 0.0; };

    std::function<double(double, double)> phi_ = [](const double h, const double g) {
        return g + h;
    };
};

}  // namespace gppc::algorithm

#endif  // GPPC_A_STAR_H_
