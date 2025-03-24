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

#ifndef GPPC_OPEN_CLOSED_LIST_H_
#define GPPC_OPEN_CLOSED_LIST_H_

#include <queue>
#include <vector>

template <typename T, typename Compare>
class OpenClosedList {
public:
    explicit OpenClosedList(size_t size) noexcept {
        nodes_.resize(size);
        closed_.resize(size, false);
    }

    void Reset() noexcept {
        open_ = {};
        closed_.assign(closed_.size(), false);
        nodes_.assign(nodes_.size(), T{});
    }

    void AddOpen(const T node) noexcept { open_.push(node); }

    T PopOpen() noexcept {
        auto node = open_.top();
        open_.pop();
        return node;
    }

    [[nodiscard]] bool EmptyOpen() const noexcept { return open_.empty(); }

    [[nodiscard]] bool InClosed(const size_t index) const noexcept { return closed_[index]; }

    bool Close(const size_t index) noexcept {
        if (!closed_[index]) {
            return closed_[index] = true;
        }
        return false;
    }

    T& GetNode(const size_t index) noexcept { return nodes_[index]; }

    T& SetNode(const size_t index, const T node) noexcept {
        return nodes_[index] = std::move(node);
    }

    [[nodiscard]] std::vector<T>& GetNodes() noexcept { return nodes_; }

    [[nodiscard]] const std::vector<T>& GetNodes() const noexcept { return nodes_; }

private:
    std::priority_queue<T, std::vector<T>, Compare> open_{};
    std::vector<bool> closed_{};
    std::vector<T> nodes_{};
};

#endif  // GPPC_OPEN_CLOSED_LIST_H_
