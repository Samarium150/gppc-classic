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

#include "entry.h"

#include "baseline.h"

using namespace gppc;

void PreprocessMap(const std::vector<bool> & /*bits*/, const int /*width*/, const int /*height*/,
                   const std::string & /*filename*/) {}

void *PrepareForSearch(const std::vector<bool> &bits, const int width, const int height,
                       const std::string & /*filename*/) {
    auto *STS = new baseline::SpanningTreeSearch(bits, width, height);
    return STS;
}

bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path) {
    auto *STS = static_cast<baseline::SpanningTreeSearch *>(data);
    path.clear();
    if (const bool exists = STS->Search({s.x, s.y}, {g.x, g.y}); !exists) {
        return true;
    }
    for (auto [x, y] : STS->GetPath()) {
        path.emplace_back(x, y);
    }
    return true;
}

std::string GetName() { return "example-SpanningTreeSearch-8N"; }
