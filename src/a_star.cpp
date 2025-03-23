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

#include "a_star.h"

#include <algorithm>
#include <queue>

namespace gppc::algorithm {

AStar::AStar(const std::vector<bool>& map, const size_t width, const size_t height)
    : grid_(map, width, height), heuristic_(Grid::HCost) {}

const std::vector<Point>& AStar::GetPath() const noexcept { return path_; }

void AStar::SetHeuristic(
    std::function<double(const Point& s1, const Point& s2)> heuristic) noexcept {
    heuristic_ = std::move(heuristic);
}

void AStar::SetPhi(std::function<double(double h, double g)> phi) noexcept {
    phi_ = std::move(phi);
}

bool AStar::operator()(const Point& start, const Point& goal) noexcept {
    path_.clear();
    node_expanded_ = 0;
    if (!grid_.Get(start) || !grid_.Get(goal)) {
        return false;
    }
    if (start == goal) {
        return true;
    }
    std::priority_queue<Node, std::vector<Node>, CompareNode> open;
    std::vector closed(grid_.Size(), false);
    std::vector<Node> nodes(grid_.Size());
    const auto start_id = grid_.Pack(start);
    const auto goal_id = grid_.Pack(goal);
    const auto h = Grid::HCost(start, goal);
    const auto f = phi_(h, 0.0);
    nodes[start_id] = {start_id, start_id, 0.0, h, f};
    open.push(nodes[start_id]);
    while (!open.empty()) {
        const auto current = open.top();
        open.pop();
        if (current.id == goal_id) {
            for (auto id = goal_id; id != start_id; id = nodes[id].parent_id) {
                path_.push_back(grid_.Unpack(id));
            }
            path_.push_back(start);
            std::ranges::reverse(path_);
            return true;
        }
        if (closed[current.id]) {
            continue;
        }
        closed[current.id] = true;
        ++node_expanded_;
        auto [x, y] = grid_.Unpack(current.id);
        for (const auto& direction : grid_.Directions()) {
            const auto [dx, dy] = Grid::GetOffset(direction);
            const auto xx = x + dx;
            const auto yy = y + dy;
            if (!grid_.Get(xx, yy)) {
                continue;
            }
            if (dx != 0 && dy != 0 && (!grid_.Get(xx, y) || !grid_.Get(x, yy))) {
                continue;
            }
            if (const auto successor_id = grid_.Pack(xx, yy); !closed[successor_id]) {
                if (const auto successor_g = current.g + Grid::GCost(direction);
                    successor_g < nodes[successor_id].g) {
                    const auto successor_h = Grid::HCost(Point{xx, yy}, goal);
                    const auto successor_f = phi_(successor_h, successor_g);
                    nodes[successor_id] = {successor_id, current.id, successor_g, successor_h,
                                           successor_f};
                    open.push(nodes[successor_id]);
                }
            }
        }
    }
    return false;
}

}  // namespace gppc::algorithm
