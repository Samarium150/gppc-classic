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

namespace gppc::algorithm {

JPS::JPS(const std::vector<bool>& map, size_t width, size_t height) noexcept
    : grid_(std::make_shared<Grid>(map, width, height)) {
    PreProcess();
}

void JPS::PreProcess() noexcept {
    if (grid_ == nullptr) {
        return;
    }
    jump_points_.resize(grid_->Size());
    for (auto y = 0; y < grid_->Height(); ++y) {
        for (auto x = 0; x < grid_->Width(); ++x) {
            if (!grid_->Get(x, y)) {
                if ((grid_->Get(x - 1, y)) && (grid_->Get(x, y + 1))) {
                    SetJumpPoint(x - 1, y + 1);
                }
                if ((grid_->Get(x - 1, y)) && (grid_->Get(x, y - 1))) {
                    SetJumpPoint(x - 1, y - 1);
                }
                if ((grid_->Get(x + 1, y)) && (grid_->Get(x, y - 1))) {
                    SetJumpPoint(x + 1, y - 1);
                }
                if ((grid_->Get(x + 1, y)) && (grid_->Get(x, y + 1))) {
                    SetJumpPoint(x + 1, y + 1);
                }
            }
        }
    }
}

const std::vector<Point>& JPS::GetPath() const noexcept { return path_; }

JPS& JPS::SetHeuristic(std::function<double(const Point&, const Point&)> heuristic) noexcept {
    heuristic_ = std::move(heuristic);
    return *this;
}

JPS& JPS::SetPhi(std::function<double(double, double)> phi) noexcept {
    phi_ = std::move(phi);
    return *this;
}

void JPS::Reset() noexcept {
    path_.clear();
    node_expanded_ = 0;
    if (grid_ != nullptr) {
        open_closed_list_.Reset(grid_->Size());
    }
}

bool JPS::Init(const Point& start, const Point& goal) noexcept {
    if (grid_ == nullptr) {
        return false;
    }
    path_.clear();
    node_expanded_ = 0;
    open_closed_list_.Reset(grid_->Size());
    start_id_ = grid_->Pack(start);
    goal_id_ = grid_->Pack(goal);
    goal_ = goal;
    const auto h = heuristic_(start, goal);
    const auto f = phi_(h, 0.0);
    open_closed_list_.AddOpen(
        open_closed_list_.SetNode(start_id_, {start_id_, {start_id_, kAll}, 0.0, h, f}));
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
    if (grid_ == nullptr) {
        return true;
    }
    if (open_closed_list_.EmptyOpen()) {
        return true;
    }
    const auto current = open_closed_list_.PopOpen();
    if (current.id == goal_id_) {
        std::vector<Point> path;
        for (auto id = goal_id_; id != start_id_; id = open_closed_list_.GetNode(id).parent.first) {
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
    for (const auto& [successor, direction, cost] : successors_) {
        if (const auto successor_id = grid_->Pack(successor);
            !open_closed_list_.InClosed(successor_id)) {
            if (const auto successor_g = current.g + cost;
                successor_g < open_closed_list_.GetNode(successor_id).g) {
                const auto successor_h = heuristic_(successor, goal_);
                const auto successor_f = phi_(successor_h, successor_g);
                open_closed_list_.AddOpen(
                    open_closed_list_.SetNode(successor_id, {successor_id,
                                                             {current.id, direction},
                                                             successor_g,
                                                             successor_h,
                                                             successor_f}));
            }
        }
    }
    return false;
}

bool JPS::operator()(const Point& start, const Point& goal) noexcept {
    if (!Init(start, goal)) {
        return false;
    }
    while (!operator()()) {
    }
    return !path_.empty();
}

bool JPS::operator()(const std::shared_ptr<Grid>& grid, const Point& start,
                     const Point& goal) noexcept {
    if (!Init(grid, start, goal)) {
        return false;
    }
    while (!operator()()) {
    }
    return !path_.empty();
}

void JPS::SetJumpPoint(const int x, const int y) noexcept {
    if (grid_ != nullptr) {
        jump_points_[grid_->Pack(x, y)] = true;
    }
}

bool JPS::IsJumpPoint(const int x, const int y) const noexcept {
    return jump_points_[grid_->Pack(x, y)];
}

JPS& JPS::SetJumpLimit(const size_t limit) noexcept {
    jump_limit_ = limit;
    return *this;
}

void JPS::GetSuccessors(const int x, const int y, const uint8_t parent_dir,  // NOLINT
                        const double current_cost, const size_t jump_count) noexcept {
    if (x == goal_.x && y == goal_.y) {
        successors_.emplace_back(Point{x, y}, parent_dir, current_cost);
        return;
    }
    bool n = false, s = false, w = false, e = false;
    if (parent_dir & kN) {
        if (y != 0 && grid_->Get(x, y - 1)) {
            uint8_t next_dir = 0;
            if (IsJumpPoint(x, y - 1)) {
                if (x != 0 && !grid_->Get(x - 1, y) && grid_->Get(x - 1, y - 1)) {
                    next_dir |= kNW;
                }
                if (x != grid_->Width() - 1 && !grid_->Get(x + 1, y) && grid_->Get(x + 1, y - 1)) {
                    next_dir |= kNE;
                }
            }
            const auto edge_cost = grid_->GCost(kN);
            if (next_dir) {
                successors_.emplace_back(Point{x, y - 1}, next_dir, current_cost + edge_cost);
            } else if (jump_count >= jump_limit_) {
                successors_.emplace_back(Point{x, y - 1}, kN, current_cost + edge_cost);
            } else {
                GetSuccessors(x, y - 1, kN, current_cost + edge_cost, jump_count + 1);
            }
            n = true;
        }
    }
    if (parent_dir & kS) {
        if (y != grid_->Height() - 1 && grid_->Get(x, y + 1)) {
            uint8_t next_dir = 0;
            if (IsJumpPoint(x, y + 1)) {
                if (x != 0 && !grid_->Get(x - 1, y) && grid_->Get(x - 1, y + 1)) {
                    next_dir |= kSW;
                }
                if (x != grid_->Width() - 1 && !grid_->Get(x + 1, y) && grid_->Get(x + 1, y + 1)) {
                    next_dir |= kSE;
                }
            }
            const auto edge_cost = grid_->GCost(kS);
            if (next_dir)
                successors_.emplace_back(Point{x, y + 1}, next_dir, current_cost + edge_cost);
            else {
                if (jump_count >= jump_limit_) {
                    successors_.emplace_back(Point{x, y + 1}, kS, current_cost + edge_cost);
                } else {
                    GetSuccessors(x, y + 1, kS, current_cost + edge_cost, jump_count + 1);
                }
            }
            s = true;
        }
    }
    if (parent_dir & kW) {
        if (x != 0 && grid_->Get(x - 1, y)) {
            uint8_t next_dir = 0;
            if (IsJumpPoint(x - 1, y)) {
                if (y != 0 && !grid_->Get(x, y - 1) && grid_->Get(x - 1, y - 1)) {
                    next_dir |= kNW;
                }
                if (y != grid_->Height() - 1 && !grid_->Get(x, y + 1) && grid_->Get(x - 1, y + 1)) {
                    next_dir |= kSW;
                }
            }
            const auto edge_cost = grid_->GCost(kW);
            if (next_dir) {
                successors_.emplace_back(Point{x - 1, y}, next_dir, current_cost + edge_cost);
            } else if (jump_count >= jump_limit_) {
                successors_.emplace_back(Point{x - 1, y}, kW, current_cost + edge_cost);
            } else {
                GetSuccessors(x - 1, y, kW, current_cost + edge_cost, jump_count + 1);
            }
            e = true;
        }
    }
    if (parent_dir & kE) {
        if (x != grid_->Width() - 1 && grid_->Get(x + 1, y)) {
            uint8_t next_dir = 0;
            if (IsJumpPoint(x + 1, y)) {
                if (y != 0 && !grid_->Get(x, y - 1) && grid_->Get(x + 1, y - 1)) {
                    next_dir |= kNE;
                }
                if (y != grid_->Height() - 1 && !grid_->Get(x, y + 1) && grid_->Get(x + 1, y + 1)) {
                    next_dir |= kSE;
                }
            }
            const auto edge_cost = grid_->GCost(kE);
            if (next_dir) {
                successors_.emplace_back(Point{x + 1, y}, next_dir, current_cost + edge_cost);
            } else if (jump_count >= jump_limit_) {
                successors_.emplace_back(Point{x + 1, y}, kE, current_cost + edge_cost);
            } else {
                GetSuccessors(x + 1, y, kE, current_cost + edge_cost, jump_count + 1);
            }
            w = true;
        }
    }
    if (parent_dir & kNW) {
        if (x != 0 && y != 0 && grid_->Get(x - 1, y - 1) && n && e) {
            const auto edge_cost = grid_->GCost(kNW);
            if (jump_count >= jump_limit_) {
                successors_.emplace_back(Point{x - 1, y - 1}, kNW, current_cost + edge_cost);
            } else {
                GetSuccessors(x - 1, y - 1, kNW, current_cost + edge_cost, jump_count + 1);
            }
        }
    }
    if (parent_dir & kNE) {
        if (x != grid_->Width() - 1 && y != 0 && grid_->Get(x + 1, y - 1) && n && w) {
            const auto edge_cost = grid_->GCost(kNE);
            if (jump_count >= jump_limit_) {
                successors_.emplace_back(Point{x + 1, y - 1}, (kNE), current_cost + edge_cost);
            } else {
                GetSuccessors(x + 1, y - 1, kNE, current_cost + edge_cost, jump_count + 1);
            }
        }
    }
    if (parent_dir & kSW) {
        if (x != 0 && y != grid_->Height() - 1 && grid_->Get(x - 1, y + 1) && s && e) {
            const auto edge_cost = grid_->GCost(kSW);
            if (jump_count >= jump_limit_) {
                successors_.emplace_back(Point{x - 1, y + 1}, kSW, current_cost + edge_cost);
            } else {
                GetSuccessors(x - 1, y + 1, kSW, current_cost + edge_cost, jump_count + 1);
            }
        }
    }
    if (parent_dir & kSE) {
        if (x != grid_->Width() - 1 && y != grid_->Height() - 1 && grid_->Get(x + 1, y + 1) && s &&
            w) {
            const auto edge_cost = grid_->GCost(kSE);
            if (jump_count >= jump_limit_) {
                successors_.emplace_back(Point{x + 1, y + 1}, kSE, current_cost + edge_cost);
            } else {
                GetSuccessors(x + 1, y + 1, kSE, current_cost + edge_cost, jump_count + 1);
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
