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
#include <unordered_set>
#include <vector>

template <typename T, typename Compare>
class OpenClosedList {
public:
    OpenClosedList() noexcept = default;

    void Reset() noexcept {
        open_ = {};
        closed_.clear();
        nodes_.clear();
    }

    void AddOpen(const T node) noexcept { open_.push(node); }

    T PopOpen() noexcept {
        auto node = open_.top();
        open_.pop();
        return node;
    }

    [[nodiscard]] T PeekOpen() const noexcept { return open_.top(); }

    [[nodiscard]] bool EmptyOpen() const noexcept { return open_.empty(); }

    [[nodiscard]] bool Closed(const size_t index) const noexcept { return closed_.contains(index); }

    bool Close(const size_t index) noexcept {
        if (!closed_.contains(index)) {
            return closed_.emplace(index).second;
        }
        return false;
    }

    std::optional<T> GetNode(const size_t index) const noexcept {
        auto it = nodes_.find(index);
        return it == nodes_.end() ? std::nullopt : std::make_optional(it->second);
    }

    T& SetNode(const size_t index, const T node) noexcept {
        return nodes_[index] = std::move(node);
    }

    // [[nodiscard]] std::vector<T>& GetNodes() noexcept { return nodes_; }

    // [[nodiscard]] const std::vector<T>& GetNodes() const noexcept { return nodes_; }

private:
    std::priority_queue<T, std::vector<T>, Compare> open_{};
    std::unordered_set<size_t> closed_{};
    std::unordered_map<size_t, T> nodes_{};
};

#endif  // GPPC_OPEN_CLOSED_LIST_H_
