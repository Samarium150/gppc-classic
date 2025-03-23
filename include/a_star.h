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

namespace gppc::algorithm {

class AStar {
public:
    AStar(const std::vector<bool>& map, size_t width, size_t height);

    [[nodiscard]] const std::vector<Point>& GetPath() const noexcept;

    void SetHeuristic(std::function<double(const Point& s1, const Point& s2)> heuristic) noexcept;

    void SetPhi(std::function<double(double h, double g)> phi) noexcept;

    bool operator()(const Point& start, const Point& goal) noexcept;

private:
    struct Node {
        size_t id = std::numeric_limits<size_t>::max();
        size_t parent_id = std::numeric_limits<size_t>::max();
        double g = std::numeric_limits<double>::max();
        double h = std::numeric_limits<double>::max();
        double f = std::numeric_limits<double>::max();
    };
    struct CompareNode {
        bool operator()(const Node& a, const Node& b) const noexcept { return a.f > b.f; }
    };

    Grid grid_;
    size_t node_expanded_{};
    std::vector<Point> path_{};

    std::function<double(const Point& s1, const Point& s2)> heuristic_ =
        [](const Point&, const Point&) { return 0.0; };

    std::function<double(double h, double g)> phi_ = [](const double h, const double g) {
        return g + h;
    };
};

}  // namespace gppc::algorithm

#endif  // GPPC_A_STAR_H_
