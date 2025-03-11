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

#include "baseline.h"

#include <array>
#include <cassert>
#include <queue>
#include <stack>

namespace gppc::baseline {

Grid::Grid(const std::vector<bool>& cells, const int width, const int height)
    : width_(static_cast<uint32_t>(width)), height_(static_cast<uint32_t>(height)), cells_(cells) {}

size_t Grid::Size() const noexcept { return cells_.size(); }

uint32_t Grid::Pack(const Point& p) const noexcept {
    assert(static_cast<uint32_t>(p.first) < width_ && static_cast<uint32_t>(p.second) < height_);
    return static_cast<uint32_t>(p.second) * width_ + static_cast<uint32_t>(p.first);
}

Point Grid::Unpack(const uint32_t p) const noexcept {
    assert(width_ != 0 && p < Size());
    return {static_cast<int>(p % width_), static_cast<int>(p / width_)};
}

bool Grid::Get(const uint32_t p) const noexcept { return p < Size() && cells_[p]; }

bool Grid::Get(const Point& p) const noexcept {
    return static_cast<uint32_t>(p.first) < width_ && static_cast<uint32_t>(p.second) < height_ &&
           cells_[Pack(p)];
}

uint32_t Grid::Width() const noexcept { return width_; }

uint32_t Grid::Height() const noexcept { return height_; }

const std::vector<bool>& Grid::Cells() const noexcept { return cells_; }

std::vector<Node>& Grid::Nodes() const noexcept { return nodes_; }

void SetupGrid(Grid& grid) {
    auto& nodes = grid.Nodes();
    nodes.assign(grid.Size(), {Node::INV, Node::INV});
    std::pmr::unsynchronized_pool_resource vector_res;
    std::pmr::vector<Point> cluster;
    struct Dist {
        bool operator()(const Point& q, const Point& p) const noexcept {
            return dist(q, center) < dist(p, center);
        }
        static int dist(const Point& q, const Point& p) noexcept {
            return std::abs(q.first - p.first) + std::abs(q.second - p.second);
        }
        Point center;
    };
    for (uint32_t i = 0; i < grid.Size(); ++i) {
        if (grid.Cells()[i] && nodes[i].pred == Node::INV) {
            // new cluster
            FloodFill(grid, cluster, i, &vector_res);
            assert(!cluster.empty());
            uint64_t sum_x = 0, sum_y = 0;
            for (const auto [x, y] : cluster) {
                sum_x += x;
                sum_y += y;
            }
            const Point cluster_center(static_cast<int>(sum_x / cluster.size()),
                                       static_cast<int>(sum_y / cluster.size()));
            const uint32_t cluster_id =
                grid.Pack(*std::ranges::min_element(cluster, Dist{cluster_center}));
            Dijkstra(grid, cluster_id, &vector_res);
            assert(std::ranges::all_of(cluster, [&grid](const Point& q) {
                return grid.Nodes().at(grid.Pack(q)).pred != Node::FLOOD_FILL;
            }));
        }
    }
}

SpanningTreeSearch::SpanningTreeSearch(const std::vector<bool>& cells, const int width,
                                       const int height)
    : Grid(cells, width, height) {
    SetupGrid(*this);
}

const std::vector<Point>& SpanningTreeSearch::GetPath() const noexcept { return path_parts_[0]; }

bool SpanningTreeSearch::Search(const Point& start, const Point& goal) {
    auto node_id = std::array<uint32_t, 2>{{Pack(start), Pack(goal)}};
    if (nodes_[node_id[0]].pred == Node::INV || nodes_[node_id[1]].pred == Node::INV) {
        return false;
    }
    if (node_id[0] == node_id[1]) {
        // zero path case
        path_parts_[0].assign(2, start);
        return true;
    }
    path_parts_[0].clear();
    path_parts_[1].clear();
    while (true) {
        int progress_id = 0;
        if (auto c0 = nodes_[node_id[0]].cost, c1 = nodes_[node_id[1]].cost; c0 == c1) {
            // same dist, check if same root
            if (node_id[0] == node_id[1]) {
                path_parts_[0].push_back(Unpack(node_id[progress_id]));
                break;  // found the least common ancestor
            }
            if (c0 == 0) return false;  // tree root's are different, no path
        } else if (c1 > c0) {
            progress_id = 1;  // node_id[1] is longer thus process it first
        }
        path_parts_[progress_id].push_back(Unpack(node_id[progress_id]));
        node_id[progress_id] = nodes_[node_id[progress_id]].pred;
    }
    // finalize the path
    path_parts_[0].insert(path_parts_[0].end(), path_parts_[1].rbegin(), path_parts_[1].rend());
    return true;
}

void FloodFill(Grid& grid, std::pmr::vector<Point>& out, const uint32_t origin,
               std::pmr::memory_resource* res) {
    auto& nodes = grid.Nodes();
    assert(origin < nodes.size() && nodes[origin].pred == Node::INV);
    out.clear();
    std::stack<Point, std::pmr::vector<Point>> stack(res);
    auto&& push_queue = [&grid, &stack](Point p, const int dx, const int dy) {
        p.first += dx;
        p.second += dy;
        if (grid.Get(p)) {
            if (auto& [pred, cost] = grid.Nodes()[grid.Pack(p)]; pred == Node::INV) {
                pred = Node::FLOOD_FILL;
                stack.push(p);
            }
        }
    };
    nodes[origin].pred = Node::FLOOD_FILL;
    stack.push(grid.Unpack(origin));
    while (!stack.empty()) {
        Point p = stack.top();
        stack.pop();
        out.push_back(p);
        push_queue(p, 1, 0);
        push_queue(p, -1, 0);
        push_queue(p, 0, 1);
        push_queue(p, 0, -1);
    }
}

void Dijkstra(Grid& grid, uint32_t origin, std::pmr::memory_resource* res) {
    auto& nodes = grid.Nodes();
    // first = dist, second = node-id
    using node_type = std::pair<uint32_t, uint32_t>;
    std::priority_queue<node_type, std::pmr::vector<node_type>, std::greater<>> queue(res);
    auto try_push = [&grid, &queue](const uint32_t node, const int dx, const int dy,
                                    uint32_t cost) {
        uint32_t new_node = static_cast<int>(node) + dy * grid.Width() + dx;
        if (auto& [new_pred, new_cost] = grid.Nodes()[new_node]; cost < new_cost) {
            new_pred = node;
            new_cost = cost;
            queue.emplace(cost, new_node);
        }
    };
    queue.emplace(0, origin);
    nodes[origin].cost = 0;
    nodes[origin].pred = Node::NO_PRED;
    while (!queue.empty()) {
        auto [cost, node] = queue.top();
        queue.pop();
        if (cost != nodes[node].cost) {
            continue;
        }
        const auto [x, y] = grid.Unpack(node);
        // push successors
        uint32_t mask = 0;
        for (int i = 0, dy = -1; dy < 2; ++dy)
            for (int dx = -1; dx < 2; ++dx) {
                mask |= static_cast<uint32_t>(grid.Get(Point(x + dx, y + dy))) << i++;
            }
        mask = ~mask;  // 1 = non-trav, 0 = trav

        // 012
        // 345
        // 678
        // N
        if ((mask & static_cast<uint32_t>(Compass::N)) == 0) {
            try_push(node, 0, -1, cost + COST_0);
        }
        // E
        if ((mask & static_cast<uint32_t>(Compass::E)) == 0) {
            try_push(node, 1, 0, cost + COST_0);
        }
        // S
        if ((mask & static_cast<uint32_t>(Compass::S)) == 0) {
            try_push(node, 0, 1, cost + COST_0);
        }
        // W
        if ((mask & static_cast<uint32_t>(Compass::W)) == 0) {
            try_push(node, -1, 0, cost + COST_0);
        }
        // NE
        if ((mask & static_cast<uint32_t>(Compass::NE)) == 0) {
            try_push(node, 1, -1, cost + COST_1);
        }
        // NW
        if ((mask & static_cast<uint32_t>(Compass::NW)) == 0) {
            try_push(node, -1, -1, cost + COST_1);
        }
        // SE
        if ((mask & static_cast<uint32_t>(Compass::SE)) == 0) {
            try_push(node, 1, 1, cost + COST_1);
        }
        // SW
        if ((mask & static_cast<uint32_t>(Compass::SW)) == 0) {
            try_push(node, -1, 1, cost + COST_1);
        }
    }
}
}  // namespace gppc::baseline
