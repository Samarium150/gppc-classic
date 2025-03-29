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

#ifndef GPPC_GRID_H_
#define GPPC_GRID_H_

#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>  // NOLINT
#include <vector>

namespace gppc {

struct Point {
    int x;
    int y;

    Point operator+(const Point& other) const noexcept;
    Point operator-(const Point& other) const noexcept;
    bool operator==(const Point& other) const noexcept;
    bool operator!=(const Point& other) const noexcept;
    friend std::ostream& operator<<(std::ostream& os, const Point& p);
};

enum Direction : uint8_t {
    kN = 0x8,
    kS = 0x4,
    kE = 0x2,
    kW = 0x1,
    kNW = kN | kW,
    kNE = kN | kE,
    kSE = kS | kE,
    kSW = kS | kW,
    kStay = 0,
    kAll = 0xFF
};

constexpr std::array kDirections = {kN, kS, kE, kW, kNW, kNE, kSE, kSW};

class Grid : public std::enable_shared_from_this<Grid> {
public:
    Grid(const std::vector<bool>& map, size_t width, size_t height) noexcept;

    Grid(const Grid& other) noexcept = default;

    Grid(Grid&& other) noexcept = default;

    Grid& operator=(const Grid& other) noexcept = default;

    Grid& operator=(Grid&& other) noexcept = default;

    virtual ~Grid() = default;

    [[nodiscard]] size_t Width() const noexcept;

    [[nodiscard]] size_t Height() const noexcept;

    [[nodiscard]] size_t Size() const noexcept;

    [[nodiscard]] size_t Pack(size_t x, size_t y) const noexcept;

    [[nodiscard]] size_t Pack(const Point& point) const noexcept;

    [[nodiscard]] Point Unpack(size_t index) const noexcept;

    [[nodiscard]] bool Get(size_t index) const noexcept;

    [[nodiscard]] bool Get(int x, int y) const noexcept;

    [[nodiscard]] bool Get(const Point& p) const noexcept;

    static std::pair<int, int> GetOffset(Direction direction) noexcept;

    [[nodiscard]] virtual double HCost(const Point& a, const Point& b) const noexcept;

    [[nodiscard]] virtual double GCost(Direction direction) const noexcept;

    [[nodiscard]] virtual double GCost(const Point& a, const Point& b) const noexcept;

private:
    std::reference_wrapper<const std::vector<bool>> map_;
    size_t width_;
    size_t height_;
};
}  // namespace gppc

#endif  // GPPC_GRID_H_
