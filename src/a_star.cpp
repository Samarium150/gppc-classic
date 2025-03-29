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

AStar::AStar(const std::vector<bool>& map, size_t width, size_t height) noexcept
    : grid_(std::make_shared<Grid>(map, width, height)) {}

const std::vector<Point>& AStar::GetPath() const noexcept { return path_; }

size_t AStar::GetNodeExpanded() const noexcept { return node_expanded_; }

const std::vector<AStar::Node>& AStar::GetNodes() const noexcept {
    return open_closed_list_.GetNodes();
}

AStar::Node AStar::PeekOpen() const noexcept { return open_closed_list_.PeekOpen(); }

bool AStar::EmptyOpen() const noexcept { return open_closed_list_.EmptyOpen(); }

AStar& AStar::SetHeuristic(
    std::function<double(const Point& s1, const Point& s2)> heuristic) noexcept {
    heuristic_ = std::move(heuristic);
    return *this;
}

AStar& AStar::SetPhi(std::function<double(double h, double g)> phi) noexcept {
    phi_ = std::move(phi);
    return *this;
}

AStar& AStar::StopAfterGoal(const bool stop) noexcept {
    stop_after_goal_ = stop;
    return *this;
}

void AStar::Reset() noexcept {
    path_.clear();
    node_expanded_ = 0;
    if (grid_ != nullptr) {
        open_closed_list_.Reset(grid_->Size());
    }
}

bool AStar::Init(const Point& start, const Point& goal) noexcept {
    if (grid_ == nullptr) {
        return false;
    }
    path_.clear();
    node_expanded_ = 0;
    open_closed_list_.Reset(grid_->Size());
    start_ = start;
    goal_ = goal;
    const auto start_id = grid_->Pack(start_);
    const auto h = heuristic_(start_, goal_);
    const auto f = phi_(h, 0.0);
    open_closed_list_.AddOpen(open_closed_list_.SetNode(start_id, {start_id, start_id, 0.0, h, f}));
    return true;
}

bool AStar::Init(const std::shared_ptr<Grid>& grid, const Point& start,
                 const Point& goal) noexcept {
    if (!grid->Get(start) || !grid->Get(goal)) {
        return false;
    }
    grid_ = grid;
    return Init(start, goal);
}

bool AStar::AddStart(const Point& point) noexcept {
    if (grid_ == nullptr || !grid_->Get(point)) {
        return false;
    }
    const auto start_id = grid_->Pack(point);
    const auto h = grid_->HCost(point, goal_);
    const auto f = phi_(h, 0.0);
    open_closed_list_.AddOpen(open_closed_list_.SetNode(start_id, {start_id, start_id, 0.0, h, f}));
    return true;
}

void AStar::GetSuccessors(const int x, const int y) noexcept {
    for (const auto& direction : kDirections) {
        const auto [dx, dy] = Grid::GetOffset(direction);
        const auto xx = x + dx;
        const auto yy = y + dy;
        if (!grid_->Get(xx, yy)) {
            continue;
        }
        if (dx != 0 && dy != 0 && (!grid_->Get(xx, y) || !grid_->Get(x, yy))) {
            continue;
        }
        successors_.emplace_back(xx, yy);
    }
}

bool AStar::operator()() noexcept {
    if (grid_ == nullptr) {
        return true;
    }
    if (open_closed_list_.EmptyOpen()) {
        return true;
    }
    const auto current = open_closed_list_.PopOpen();
    if (stop_after_goal_ && current.id == grid_->Pack(goal_)) {
        const auto start_id = grid_->Pack(start_);
        const auto goal_id = grid_->Pack(goal_);
        for (auto id = goal_id; id != start_id; id = open_closed_list_.GetNode(id).parent_id) {
            path_.push_back(grid_->Unpack(id));
        }
        path_.push_back(start_);
        std::ranges::reverse(path_);
        return true;
    }
    if (!open_closed_list_.Close(current.id)) {
        return false;
    }
    ++node_expanded_;
    const auto [x, y] = grid_->Unpack(current.id);
    successors_.clear();
    GetSuccessors(x, y);
    for (const auto& [xx, yy] : successors_) {
        if (const auto successor_id = grid_->Pack(xx, yy);
            !open_closed_list_.InClosed(successor_id)) {
            if (const auto successor_g = current.g + grid_->GCost({x, y}, {xx, yy});
                successor_g < open_closed_list_.GetNode(successor_id).g) {
                const auto successor_h = heuristic_({xx, yy}, goal_);
                const auto successor_f = phi_(successor_h, successor_g);
                open_closed_list_.AddOpen(open_closed_list_.SetNode(
                    successor_id,
                    {successor_id, current.id, successor_g, successor_h, successor_f}));
            }
        }
    }
    return false;
}

bool AStar::operator()(const Point& start, const Point& goal) noexcept {
    if (!Init(start, goal)) {
        return false;
    }
    while (!operator()()) {
    }
    return !path_.empty();
}

bool AStar::operator()(const std::shared_ptr<Grid>& grid, const Point& start,
                       const Point& goal) noexcept {
    if (!Init(grid, start, goal)) {
        return false;
    }
    while (!operator()()) {
    }
    return !path_.empty();
}

}  // namespace gppc::algorithm
