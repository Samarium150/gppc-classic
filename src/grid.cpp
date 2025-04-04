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

#include "grid.h"

#include <numbers>

namespace gppc {

Point Point::operator+(const Point& other) const noexcept { return {x + other.x, y + other.y}; }

Point Point::operator-(const Point& other) const noexcept { return {x - other.x, y - other.y}; }

bool Point::operator==(const Point& other) const noexcept { return x == other.x && y == other.y; }

bool Point::operator!=(const Point& other) const noexcept { return x != other.x || y != other.y; }

Point::operator std::string() const {
    return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

std::ostream& operator<<(std::ostream& os, const Point& p) {
    return os << std::string(p);
}

Grid::Grid(const std::vector<bool>& map, const size_t width, const size_t height) noexcept
    : map_(map), width_(width), height_(height) {}

size_t Grid::Width() const noexcept { return width_; }

size_t Grid::Height() const noexcept { return height_; }

size_t Grid::Size() const noexcept { return map_.get().size(); }

size_t Grid::Pack(const size_t x, const size_t y) const noexcept { return x + y * width_; }

size_t Grid::Pack(const Point& point) const noexcept {
    const auto [x, y] = point;
    return Pack(static_cast<size_t>(x), static_cast<size_t>(y));
}

Point Grid::Unpack(const size_t index) const noexcept {
    return {static_cast<int>(index % width_), static_cast<int>(index / width_)};
}

bool Grid::Get(const size_t index) const noexcept { return index < Size() && map_.get()[index]; }

bool Grid::Get(const int x, const int y) const noexcept {
    if (x < 0 || y < 0) {
        return false;
    }
    const auto ux = static_cast<size_t>(x);
    const auto uy = static_cast<size_t>(y);
    return ux < width_ && uy < height_ && Get(Pack(ux, uy));
}

bool Grid::Get(const Point& p) const noexcept {
    const auto [x, y] = p;
    return Get(x, y);
}

std::pair<int, int> Grid::GetOffset(const Direction direction) noexcept {
    switch (direction) {
        case kN:
            return {0, -1};
        case kS:
            return {0, 1};
        case kW:
            return {-1, 0};
        case kE:
            return {1, 0};
        case kNW:
            return {-1, -1};
        case kNE:
            return {1, -1};
        case kSW:
            return {-1, 1};
        case kSE:
            return {1, 1};
        default:
            return {0, 0};
    }
}

double Grid::HCost(const Point& a, const Point& b) const noexcept {
    const auto dx = std::abs(a.x - b.x);
    const auto dy = std::abs(a.y - b.y);
    return static_cast<double>(dx + dy) + (std::numbers::sqrt2 - 2.0) * std::min(dx, dy);
}

double Grid::GCost(const Direction direction) const noexcept {
    switch (direction) {
        case kN:
        case kS:
        case kW:
        case kE:
            return 1.0;
        case kNW:
        case kNE:
        case kSW:
        case kSE:
            return std::numbers::sqrt2;
        default:
            return 0.0;
    }
}

double Grid::GCost(const Point& a, const Point& b) const noexcept {
    if (a == b) {
        return 0.0;
    }
    if (a.x == b.x || a.y == b.y) {
        return 1.0;
    }
    return std::numbers::sqrt2;
}

}  // namespace gppc
