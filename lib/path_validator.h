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

#ifndef LIB_GPPC_PATH_VALIDATOR_H_
#define LIB_GPPC_PATH_VALIDATOR_H_

#include <vector>

namespace gppc::lib {

struct Point {
    int x = 0;
    int y = 0;

    Point operator+(const Point& other) const noexcept { return {x + other.x, y + other.y}; }
    Point operator-(const Point& other) const noexcept { return {x - other.x, y - other.y}; }
    bool operator==(const Point& other) const noexcept { return x == other.x && y == other.y; }
    bool operator!=(const Point& other) const noexcept { return x != other.x || y != other.y; }
};

class PathValidator {
public:
    PathValidator(const std::vector<bool>& map, int width, int height);

    [[nodiscard]] bool Get(int x, int y) const noexcept;

    [[nodiscard]] bool Get(const Point& point) const noexcept;

    [[nodiscard]] bool IsValidPoint(const Point& point) const noexcept;

    [[nodiscard]] bool IsValidEdge(const Point& start, const Point& end) const noexcept;

private:
    [[nodiscard]] bool IsValidCardinal(const Point& start, const Point& end,
                                       const Point& diff) const noexcept;

    [[nodiscard]] bool IsValidOrdinal(const Point& start, const Point& end,
                                      const Point& diff) const noexcept;

    const std::vector<bool> map_;
    std::size_t width_;
    std::size_t height_;
};

template <typename Container>
int ValidatePath(const std::vector<bool>& map, const int width, const int height,
                 const Container& path) {
    const auto size = static_cast<std::size_t>(path.size());
    if (size == 0) {
        return -1;
    }
    if (size == 1) {
        return 0;
    }
    const PathValidator validator(map, width, height);
    // check each point in the path
    for (std::size_t i = 0; i < size; ++i) {
        if (const Point u{static_cast<int>(path[i].x), static_cast<int>(path[i].y)};
            !validator.IsValidPoint(u)) {
            return static_cast<int>(i);
        }
    }
    // check each segment
    for (std::size_t i = 0; i < size - 1; ++i) {
        const Point u{static_cast<int>(path[i].x), static_cast<int>(path[i].y)};
        if (const Point v{static_cast<int>(path[i + 1].x), static_cast<int>(path[i + 1].y)};
            !validator.IsValidEdge(u, v)) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

}  // namespace gppc::lib

#endif  // LIB_GPPC_PATH_VALIDATOR_H_
