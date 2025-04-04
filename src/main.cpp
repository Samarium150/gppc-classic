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

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>

#include "grid.h"
#include "jps.h"
#include "path_validator.h"
#include "scenario_loader.h"
#include "timer.h"

static bool LoadMap(const std::string& filename, std::vector<bool>& map, int& width, int& height) {
    std::ifstream ifs(filename);
    if (!ifs) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return false;
    }
    std::string token;
    ifs >> token;
    ifs >> token;
    ifs >> token;
    ifs >> height;
    ifs >> token;
    ifs >> width;
    ifs >> token;

    map.resize(height * width);
    std::string line;
    std::getline(ifs, line);
    for (int y = 0; y < height; ++y) {
        std::getline(ifs, line);
        int x = 0;
        for (const char c : line) {
            if (std::isspace(static_cast<unsigned char>(c))) {
                continue;
            }
            if (x < width) {
                map[y * width + x] = c == '.' || c == 'G' || c == 'S';
                ++x;
            }
        }
    }
    return true;
}

[[maybe_unused]] static void PrintMap(const std::vector<bool>& map, const int width,
                                      const int height,
                                      const std::optional<gppc::Point> start = std::nullopt,
                                      const std::optional<gppc::Point> goal = std::nullopt) {
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (gppc::Point(x, y) == start.value_or(gppc::Point{-1, -1})) {
                std::cout << 'S';
                continue;
            }
            if (gppc::Point(x, y) == goal.value_or(gppc::Point{-1, -1})) {
                std::cout << 'G';
                continue;
            }
            std::cout << (map[y * width + x] ? '.' : '#');
        }
        std::cout << std::endl;
    }
}

double EuclideanDist(const gppc::Point& a, const gppc::Point& b) {
    const auto dx = std::abs(a.x - b.x);
    const auto dy = std::abs(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

double GetPathLength(const std::vector<gppc::Point>& path) {
    double len = 0;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        len += EuclideanDist(path[i], path[i + 1]);
    }
    return len;
}

template <typename T, typename U>
static double Round(T f, const U n) {  // NOLINT
    return std::round(f * std::pow(10, n)) / std::pow(10, n);
}

void PrintPathOnMap(const std::vector<bool>& map, const int width, const int height,
                    const std::vector<gppc::Point>& path) {
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (gppc::Point(x, y) == path.front()) {
                std::cout << 'S';
                continue;
            }
            if (gppc::Point(x, y) == path.back()) {
                std::cout << 'G';
                continue;
            }
            if (std::ranges::find(path, gppc::Point(x, y)) != path.end()) {
                std::cout << 'x';
                continue;
            }
            std::cout << (map[y * width + x] ? '.' : '#');
        }
        std::cout << std::endl;
    }
}

size_t CountDecimalPlaces(const double value) {
    std::stringstream ss;
    ss << std::fixed << value;
    std::string str = ss.str();

    const size_t decimalPos = str.find('.');
    if (decimalPos == std::string::npos) {
        return 0;
    }

    // Remove trailing zeros
    const size_t lastNonZero = str.find_last_not_of('0');
    if (lastNonZero == decimalPos) {
        return 0;  // No significant decimal digits
    }

    return lastNonZero - decimalPos;
}

template <std::floating_point T>
bool equals(T a, T b, T rel_epsilon = 1e-6, T abs_epsilon = std::numeric_limits<T>::epsilon()) {
    if (std::isinf(a) || std::isinf(b)) {
        return a == b;
    }
    if (std::isnan(a) || std::isnan(b)) {
        return false;
    }
    T diff = std::abs(a - b);
    if (diff <= abs_epsilon) {
        return true;
    }
    T max_val = std::max(std::abs(a), std::abs(b));
    return diff <= rel_epsilon * max_val;
}

constexpr unsigned kMaxDim = 5;

int main() {
    std::vector<bool> map;
    int width, height;
    if (!LoadMap("data/AcrosstheCape.map", map, width, height)) {
        return 1;
    }
    // PrintMap(map, width, height);
    const gppc::lib::ScenarioLoader loader("data/AcrosstheCape.map.scen");
    const auto grid = std::make_shared<gppc::Grid>(map, width, height);
    // auto dh5 =
    //     gppc::algorithm::GridEmbedding(grid, kMaxDim,
    //     gppc::algorithm::GridEmbedding::Metric::kL1);
    // for (size_t i = 0; i < kMaxDim; ++i) {
    //     std::cout << "Build Dim " << i << std::endl;
    //     dh5.AddDimension(gppc::algorithm::GridEmbedding::DimensionType::kDifferential,
    //                      gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
    // }
    // auto fm4dh =
    //     gppc::algorithm::GridEmbedding(grid, kMaxDim,
    //     gppc::algorithm::GridEmbedding::Metric::kL1);
    // for (size_t i = 0; i < kMaxDim - 1; ++i) {
    //     std::cout << "Build Dim " << i << std::endl;
    //     fm4dh.AddDimension(gppc::algorithm::GridEmbedding::DimensionType::kFastMap,
    //                        gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
    // }
    // fm4dh.AddDimension(gppc::algorithm::GridEmbedding::DimensionType::kDifferential,
    //                    gppc::algorithm::GridEmbedding::PivotPlacement::kHeuristicError);
    //
    // auto embeddings = std::vector{dh5, fm4dh};

    // std::ofstream ofs("rmtst01.map.emb", std::ios::binary);
    // ofs << embedding;
    // ofs.close();
    //
    // auto written_embedding = std::make_shared<gppc::algorithm::GridEmbedding>();
    // std::ifstream ifs("rmtst01.map.emb", std::ios::binary);
    // ifs >> *written_embedding;
    // assert((*written_embedding) == (*embedding));

    auto algo = gppc::algorithm::JPS(grid);
    algo.SetJumpLimit(16);
    // algo.SetHeuristic([&embeddings](const auto& a, const auto& b) {
    //     double h = 0;
    //     for (const auto& embedding : embeddings) {
    //         h = std::max(h, embedding.HCost(a, b));
    //     }
    //     // std::cout << "H(" << a << ", " << b << ") = " << h << std::endl;
    //     return h;
    // });

    [[maybe_unused]] constexpr auto weight = 1;
    // algo.SetPhi([&weight](const double h, const double g) { return g + weight * h; });
    gppc::lib::Timer timer;
    gppc::lib::Timer::duration elapsed_time{};
    for (size_t n = 0; n < loader.GetNumExperiments(); ++n) {
        const auto& experiment = loader.GetExperiment(n);
        const auto start = gppc::Point(experiment.GetStartX(), experiment.GetStartY());
        const auto goal = gppc::Point(experiment.GetGoalX(), experiment.GetGoalY());
        const auto expected = experiment.GetDistance();
        const auto dp = CountDecimalPlaces(expected);
        timer.StartTimer();
        if (algo(start, goal)) {
            timer.EndTimer();
            const auto& path = algo.GetPath();
            // if (!path.empty()) {
            //     PrintPathOnMap(map, width, height, path);
            // } else {
            //     PrintMap(map, width, height, start, goal);
            // }
            if (const auto length = GetPathLength(path);
                !equals(Round(length, dp), expected, 1e-5) ||
                gppc::lib::ValidatePath(map, width, height, algo.GetPath()) != -1) {
                std::cerr << std::setprecision(10) << n << ": " << start << " -> " << goal << " | "
                          << length << "/" << expected << std::endl;
            } else {
                std::cout << std::setprecision(10) << n << ": " << start << " -> " << goal << " | "
                          << length << "/" << expected << std::endl;
            }
        } else if (expected == 0) {
            timer.EndTimer();
            std::cout << n << ": " << start << " -> " << goal << " | No path" << std::endl;
        } else {
            timer.EndTimer();
            std::cerr << n << ": " << start << " -> " << goal << " | No path / " << expected
                      << std::endl;
        }
        elapsed_time += timer.GetElapsedTime();
    }
    std::cout << "Time elapsed: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() 
              << "ms" << std::endl;
    return 0;
}
