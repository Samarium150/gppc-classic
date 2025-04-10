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

#include "jps.h"

#include <algorithm>

namespace gppc::algorithm {

JPS::JPS(const std::vector<bool>& map, const size_t width, const size_t height) noexcept
    : HeuristicSearch(map, width, height),
      width_(static_cast<int>(width)),
      height_(static_cast<int>(height)) {
    PreProcess();
}

JPS::JPS(const std::shared_ptr<Grid>& grid) noexcept
    : HeuristicSearch(grid),
      width_(static_cast<int>(grid->Width())),
      height_(static_cast<int>(grid->Height())) {
    PreProcess();
}

JPS& JPS::SetJumpLimit(const unsigned limit) noexcept {
    jump_limit_ = limit;
    return *this;
}

bool JPS::AddStart(const Point& point) noexcept {
    if (!grid_ || !grid_->Get(point)) {
        return false;
    }
    const auto start_id = grid_->Pack(point);
    open_closed_list_.AddOpen(open_closed_list_.SetNode(
        start_id, {start_id, {start_id, kAll}, 0.0, phi_(heuristic_(point, goal_), 0.0)}));
    return true;
}

bool JPS::Init(const Point& start, const Point& goal) noexcept {
    if (!grid_) {
        return false;
    }
    path_.clear();
    node_expanded_ = 0;
    open_closed_list_.Reset();
    start_id_ = grid_->Pack(start);
    goal_id_ = grid_->Pack(goal);
    goal_ = goal;
    open_closed_list_.AddOpen(open_closed_list_.SetNode(
        start_id_, {start_id_, {start_id_, kAll}, 0.0, phi_(heuristic_(start, goal), 0.0)}));
    return true;
}

bool JPS::Init(const std::shared_ptr<Grid>& grid, const Point& start, const Point& goal) noexcept {
    if (!grid->Get(start) || !grid->Get(goal)) {
        return false;
    }
    grid_ = grid;
    PreProcess();
    return Init(start, goal);
}

bool JPS::operator()() noexcept {
    if (!grid_) {
        return true;
    }
    if (open_closed_list_.EmptyOpen()) {
        return true;
    }
    const auto current = open_closed_list_.PopOpen();
    if (current.id == goal_id_) {
        std::vector<Point> path;
        for (auto id = goal_id_; id != start_id_;
             id = open_closed_list_.GetNode(id).value().parent.first) {
            path.push_back(grid_->Unpack(id));
        }
        path.push_back(grid_->Unpack(start_id_));
        std::ranges::reverse(path);
        Interpolate(path);
        return true;
    }
    if (!open_closed_list_.Close(current.id)) {
        return false;
    }
    ++node_expanded_;
    const auto [x, y] = grid_->Unpack(current.id);
    successors_.clear();
    GetSuccessors(x, y, current.parent.second);
    for (const auto& [xx, yy, direction, cost] : successors_) {
        if (const auto successor_id = grid_->Pack(xx, yy);
            !open_closed_list_.Closed(successor_id)) {
            if (const auto successor_g = current.g + cost;
                successor_g < open_closed_list_.GetNode(successor_id).value_or(JPSNode{}).g) {
                open_closed_list_.AddOpen(open_closed_list_.SetNode(
                    successor_id, {successor_id,
                                   {current.id, direction},
                                   successor_g,
                                   phi_(heuristic_({xx, yy}, goal_), successor_g)}));
            }
        }
    }
    return false;
}

bool JPS::operator()(const Point& start, const Point& goal) noexcept {
    return HeuristicSearch::operator()(start, goal);
}

bool JPS::operator()(const std::shared_ptr<Grid>& grid, const Point& start,
                     const Point& goal) noexcept {
    return HeuristicSearch::operator()(grid, start, goal);
}

void JPS::PreProcess() noexcept {
    if (!grid_) {
        return;
    }
    jump_points_.resize(grid_->Size());
    for (auto y = 0; y < height_; ++y) {
        for (auto x = 0; x < width_; ++x) {
            if (!grid_->Get(x, y)) {
                if (grid_->Get(x - 1, y) && grid_->Get(x, y + 1)) {
                    SetJumpPoint(x - 1, y + 1);
                }
                if (grid_->Get(x - 1, y) && grid_->Get(x, y - 1)) {
                    SetJumpPoint(x - 1, y - 1);
                }
                if (grid_->Get(x + 1, y) && grid_->Get(x, y - 1)) {
                    SetJumpPoint(x + 1, y - 1);
                }
                if (grid_->Get(x + 1, y) && grid_->Get(x, y + 1)) {
                    SetJumpPoint(x + 1, y + 1);
                }
            }
        }
    }
}

void JPS::SetJumpPoint(const int x, const int y) noexcept {
    if (grid_) {
        jump_points_[grid_->Pack(x, y)] = true;
    }
}

bool JPS::IsJumpPoint(const int x, const int y) const noexcept {
    return jump_points_[grid_->Pack(x, y)];
}

void JPS::GetSuccessors(const int x, const int y, const uint8_t parent_dir,
                        const double current_cost) noexcept {
    temp_successors_.emplace(x, y, parent_dir, current_cost);
    while (!temp_successors_.empty()) {
        const auto [xx, yy, dir, cost] = temp_successors_.top();
        temp_successors_.pop();
        if (xx == goal_.x && yy == goal_.y) {
            successors_.emplace_back(xx, yy, dir, cost);
            auto stack = std::stack<Successor>();
            temp_successors_.swap(stack);
            return;
        }
        bool n = false;
        bool s = false;
        bool w = false;
        bool e = false;
        if (dir & kN) {
            if (yy >= 0 && grid_->Get(xx, yy - 1)) {
                uint8_t next_dir = 0;
                if (IsJumpPoint(xx, yy - 1)) {
                    if (xx >= 0 && !grid_->Get(xx - 1, yy) && grid_->Get(xx - 1, yy - 1)) {
                        next_dir |= kNW;
                    }
                    if (xx < width_ && !grid_->Get(xx + 1, yy) && grid_->Get(xx + 1, yy - 1)) {
                        next_dir |= kNE;
                    }
                }
                const auto edge_cost = grid_->GCost(kN);
                if (next_dir) {
                    successors_.emplace_back(xx, yy - 1, next_dir, cost + edge_cost);
                } else if (cost >= jump_limit_) {
                    successors_.emplace_back(xx, yy - 1, kN, cost + edge_cost);
                } else {
                    temp_successors_.emplace(xx, yy - 1, kN, cost + edge_cost);
                }
                n = true;
            }
        }
        if (dir & kS) {
            if (yy < height_ && grid_->Get(xx, yy + 1)) {
                uint8_t next_dir = 0;
                if (IsJumpPoint(xx, yy + 1)) {
                    if (xx >= 0 && !grid_->Get(xx - 1, yy) && grid_->Get(xx - 1, yy + 1)) {
                        next_dir |= kSW;
                    }
                    if (xx < width_ && !grid_->Get(xx + 1, yy) && grid_->Get(xx + 1, yy + 1)) {
                        next_dir |= kSE;
                    }
                }
                const auto edge_cost = grid_->GCost(kS);
                if (next_dir) {
                    successors_.emplace_back(xx, yy + 1, next_dir, cost + edge_cost);
                } else {
                    if (cost >= jump_limit_) {
                        successors_.emplace_back(xx, yy + 1, kS, cost + edge_cost);
                    } else {
                        temp_successors_.emplace(xx, yy + 1, kS, cost + edge_cost);
                    }
                }
                s = true;
            }
        }
        if (dir & kW) {
            if (xx >= 0 && grid_->Get(xx - 1, yy)) {
                uint8_t next_dir = 0;
                if (IsJumpPoint(xx - 1, yy)) {
                    if (yy >= 0 && !grid_->Get(xx, yy - 1) && grid_->Get(xx - 1, yy - 1)) {
                        next_dir |= kNW;
                    }
                    if (yy < height_ && !grid_->Get(xx, yy + 1) && grid_->Get(xx - 1, yy + 1)) {
                        next_dir |= kSW;
                    }
                }
                const auto edge_cost = grid_->GCost(kW);
                if (next_dir) {
                    successors_.emplace_back(xx - 1, yy, next_dir, cost + edge_cost);
                } else if (cost >= jump_limit_) {
                    successors_.emplace_back(xx - 1, yy, kW, cost + edge_cost);
                } else {
                    temp_successors_.emplace(xx - 1, yy, kW, cost + edge_cost);
                }
                e = true;
            }
        }
        if (dir & kE) {
            if (xx < width_ && grid_->Get(xx + 1, yy)) {
                uint8_t next_dir = 0;
                if (IsJumpPoint(xx + 1, yy)) {
                    if (yy >= 0 && !grid_->Get(xx, yy - 1) && grid_->Get(xx + 1, yy - 1)) {
                        next_dir |= kNE;
                    }
                    if (yy < height_ && !grid_->Get(xx, yy + 1) && grid_->Get(xx + 1, yy + 1)) {
                        next_dir |= kSE;
                    }
                }
                const auto edge_cost = grid_->GCost(kE);
                if (next_dir) {
                    successors_.emplace_back(xx + 1, yy, next_dir, cost + edge_cost);
                } else if (cost >= jump_limit_) {
                    successors_.emplace_back(xx + 1, yy, kE, cost + edge_cost);
                } else {
                    temp_successors_.emplace(xx + 1, yy, kE, cost + edge_cost);
                }
                w = true;
            }
        }
        if (dir & kNW) {
            if (xx >= 0 && yy >= 0 && grid_->Get(xx - 1, yy - 1) && n && e) {
                const auto edge_cost = grid_->GCost(kNW);
                if (cost >= jump_limit_) {
                    successors_.emplace_back(xx - 1, yy - 1, kNW, cost + edge_cost);
                } else {
                    temp_successors_.emplace(xx - 1, yy - 1, kNW, cost + edge_cost);
                }
            }
        }
        if (dir & kNE) {
            if (xx < width_ && yy >= 0 && grid_->Get(xx + 1, yy - 1) && n && w) {
                const auto edge_cost = grid_->GCost(kNE);
                if (cost >= jump_limit_) {
                    successors_.emplace_back(xx + 1, yy - 1, kNE, cost + edge_cost);
                } else {
                    temp_successors_.emplace(xx + 1, yy - 1, kNE, cost + edge_cost);
                }
            }
        }
        if (dir & kSW) {
            if (xx >= 0 && yy < height_ && grid_->Get(xx - 1, yy + 1) && s && e) {
                const auto edge_cost = grid_->GCost(kSW);
                if (cost >= jump_limit_) {
                    successors_.emplace_back(xx - 1, yy + 1, kSW, cost + edge_cost);
                } else {
                    temp_successors_.emplace(xx - 1, yy + 1, kSW, cost + edge_cost);
                }
            }
        }
        if (dir & kSE) {
            if (xx < width_ && yy < height_ && grid_->Get(xx + 1, yy + 1) && s && w) {
                const auto edge_cost = grid_->GCost(kSE);
                if (cost >= jump_limit_) {
                    successors_.emplace_back(xx + 1, yy + 1, kSE, cost + edge_cost);
                } else {
                    temp_successors_.emplace(xx + 1, yy + 1, kSE, cost + edge_cost);
                }
            }
        }
    }
}

void JPS::Interpolate(const std::vector<Point>& path) noexcept {
    if (path.empty()) {
        return;
    }
    for (size_t i = 0; i < path.size() - 1; ++i) {
        int dx = path[i + 1].x - path[i].x;
        int dy = path[i + 1].y - path[i].y;
        const int delta_x = dx > 0 ? 1 : -1;
        const int delta_y = dy > 0 ? 1 : -1;
        dx = std::abs(dx);
        dy = std::abs(dy);
        if (dx == 0 || dy == 0 || dx == dy) {
            path_.push_back(path[i]);
            path_.push_back(path[i + 1]);
            continue;
        }
        auto current = path[i];
        path_.push_back(current);
        const int steps = std::min(dx, dy);
        current.x += delta_x * steps;
        current.y += delta_y * steps;
        path_.push_back(current);
        if (dx > steps) {
            current.x += delta_x * (dx - steps);
            path_.push_back(current);
        } else if (dy > steps) {
            current.y += delta_y * (dy - steps);
            path_.push_back(current);
        }
    }
}

}  // namespace gppc::algorithm
