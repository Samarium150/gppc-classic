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

namespace gppc::algorithm {

AStar::AStar(const std::vector<bool>& map, const size_t width, const size_t height) noexcept
    : HeuristicSearch(map, width, height) {}

AStar::AStar(const std::shared_ptr<Grid>& grid) noexcept : HeuristicSearch(grid) {}

bool AStar::AddStart(const Point& point) noexcept {
    if (!grid_ || !grid_->Get(point)) {
        return false;
    }
    const auto start_id = grid_->Pack(point);
    const auto h = grid_->HCost(point, goal_);
    const auto f = phi_(h, 0.0);
    open_closed_list_.AddOpen(open_closed_list_.SetNode(start_id, {start_id, start_id, 0.0, h, f}));
    return true;
}

bool AStar::Init(const Point& start, const Point& goal) noexcept {
    if (!grid_) {
        return false;
    }
    path_.clear();
    node_expanded_ = 0;
    open_closed_list_.Reset();
    start_id_ = grid_->Pack(start);
    goal_id_ = grid_->Pack(goal);
    start_ = start;
    goal_ = goal;
    const auto h = heuristic_(start, goal);
    const auto f = phi_(h, 0.0);
    open_closed_list_.AddOpen(
        open_closed_list_.SetNode(start_id_, {start_id_, start_id_, 0.0, h, f}));
    return true;
}

bool AStar::Init(const std::shared_ptr<Grid>& grid, const Point& start,
                 const Point& goal) noexcept {
    return HeuristicSearch::Init(grid, start, goal);
}

bool AStar::operator()() noexcept {
    if (!grid_) {
        return true;
    }
    if (open_closed_list_.EmptyOpen()) {
        return true;
    }
    const auto current = open_closed_list_.PopOpen();
    if (stop_after_goal_ && current.id == goal_id_) {
        for (auto id = goal_id_; id != start_id_;
             id = open_closed_list_.GetNode(id).value().parent_id) {
            path_.push_back(grid_->Unpack(id));
        }
        path_.push_back(grid_->Unpack(start_id_));
        std::ranges::reverse(path_);
        return true;
    }
    if (!open_closed_list_.Close(current.id)) {
        return false;
    }
    ++node_expanded_;
    const auto current_point = grid_->Unpack(current.id);
    successors_.clear();
    GetSuccessors(current_point.x, current_point.y);
    for (const auto& successor : successors_) {
        if (const auto successor_id = grid_->Pack(successor);
            !open_closed_list_.Closed(successor_id)) {
            if (const auto successor_g = current.g + grid_->GCost(current_point, successor);
                successor_g < open_closed_list_.GetNode(successor_id).value_or(AstarNode{}).g) {
                const auto successor_h = heuristic_(successor, goal_);
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
    return HeuristicSearch::operator()(start, goal);
}

bool AStar::operator()(const std::shared_ptr<Grid>& grid, const Point& start,
                       const Point& goal) noexcept {
    return HeuristicSearch::operator()(grid, start, goal);
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

}  // namespace gppc::algorithm
