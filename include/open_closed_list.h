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
#include <unordered_map>
#include <unordered_set>

template <typename T>
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

    [[nodiscard]] bool Closed(const uint64_t hash) const noexcept { return closed_.contains(hash); }

    bool Close(const uint64_t hash) noexcept {
        if (!closed_.contains(hash)) {
            return closed_.emplace(hash).second;
        }
        return false;
    }

    std::optional<T> GetNode(const uint64_t hash) const noexcept {
        auto it = nodes_.find(hash);
        return it == nodes_.end() ? std::nullopt : std::make_optional(it->second);
    }

    T& SetNode(const uint64_t hash, const T node) noexcept {
        return nodes_[hash] = std::move(node);
    }

private:
    struct Hash {
        size_t operator()(const uint64_t& key) const noexcept { return key; }
    };
    std::priority_queue<T> open_{};
    std::unordered_set<uint64_t, Hash> closed_{};
    std::unordered_map<uint64_t, T, Hash> nodes_{};
};

#endif  // GPPC_OPEN_CLOSED_LIST_H_
