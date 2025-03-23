/*
Copyright (c) 2023 Grid-based Path Planning Competition and Contributors
<https://gppc.search-conference.org/>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "path_validator.h"

#include <cassert>

namespace gppc::lib {

PathValidator::PathValidator(const std::vector<bool>& map, const int width, const int height)
    : map_(map),
      width_(static_cast<std::size_t>(width)),
      height_(static_cast<std::size_t>(height)) {}

bool PathValidator::Get(const int x, const int y) const noexcept {
    assert(static_cast<size_t>(x) < width_ && static_cast<size_t>(y) < height_);
    return map_[y * width_ + x];
}

bool PathValidator::Get(const Point& point) const noexcept { return Get(point.x, point.y); }

bool PathValidator::IsValidPoint(const Point& point) const noexcept {
    return static_cast<std::size_t>(point.x) < width_ &&
           static_cast<std::size_t>(point.y) < height_ && Get(point);
}

bool PathValidator::IsValidEdge(const Point& start, const Point& end) const noexcept {
    const auto [x, y] = end - start;
    Point diff;
    bool is_cardinal;
    if (x == 0) {
        is_cardinal = true;
        if (y == 0) {  // u = v
            return true;
        }
        // horizontal line
        diff = Point{0, y > 0 ? 1 : -1};
    } else if (y == 0) {
        is_cardinal = true;
        // vertical line
        diff = Point{x > 0 ? 1 : -1, 0};
    } else {  // non-cardinal line
        is_cardinal = false;
        // non-ordinal
        if (std::abs(x) != std::abs(y)) {
            return false;
        }
        // ordinal line
        diff = Point{x > 0 ? 1 : -1, y > 0 ? 1 : -1};
    }
    // check cells are clear in the grid
    return (is_cardinal) ? IsValidCardinal(start, end, diff) : IsValidOrdinal(start, end, diff);
}

bool PathValidator::IsValidCardinal(const Point& start, const Point& end,
                                    const Point& diff) const noexcept {
    for (Point x = start; x != end; x = x + diff) {
        if (!Get(x)) {
            return false;
        }
    }
    return true;
}

bool PathValidator::IsValidOrdinal(const Point& start, const Point& end,
                                   const Point& diff) const noexcept {
    // check every 2x2 square along u-v
    for (Point x = start; x != end; x = x + diff) {
        if (!Get(x) || !Get(x.x + diff.x, x.y) || !Get(x.x, x.y + diff.y)) {
            return false;
        }
    }
    // separate from loop to prevent 2x2 squares past v getting checked
    if (!Get(end)) {
        return false;
    }
    return true;
}

}  // namespace gppc::lib
