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

#include <fstream>
#include <iostream>

#include "a_star.h"
#include "path_validator.h"
#include "scenario_loader.h"

static bool LoadMap(const std::string &filename, std::vector<bool> &map, int &width, int &height) {
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
                map[y * width + x] = (c == '.' || c == 'G' || c == 'S');
                ++x;
            }
        }
    }
    return true;
}

static void PrintMap(const std::vector<bool> &map, const int width, const int height) {
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::cout << (map[y * width + x] ? '.' : '#');
        }
        std::cout << std::endl;
    }
}

double EuclideanDist(const gppc::Point &a, const gppc::Point &b) {
    const auto dx = std::abs(a.x - b.x);
    const auto dy = std::abs(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

double GetPathLength(const std::vector<gppc::Point> &path) {
    double len = 0;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        len += EuclideanDist(path[i], path[i + 1]);
    }
    return len;
}

template <typename T>
static double Round(T f, const int n) {  // NOLINT
    return std::round(f * std::pow(10, n)) / std::pow(10, n);
}

void PrintPathOnMap(const std::vector<bool> &map, const int width, const int height,
                    const std::vector<gppc::lib::Point> &path) {
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (gppc::lib::Point(x, y) == path.front()) {
                std::cout << 'S';
                continue;
            }
            if (gppc::lib::Point(x, y) == path.back()) {
                std::cout << 'G';
                continue;
            }
            if (std::ranges::find(path, gppc::lib::Point(x, y)) != path.end()) {
                std::cout << 'x';
                continue;
            }
            std::cout << (map[y * width + x] ? '.' : '#');
        }
        std::cout << std::endl;
    }
}

int main() {
    std::vector<bool> map;
    int width, height;
    if (!LoadMap("data/rmtst01.map", map, width, height)) {
        return 1;
    }
    PrintMap(map, width, height);
    const gppc::lib::ScenarioLoader loader("data/rmtst01.map.scen");
    auto algo = gppc::algorithm::AStar(map, width, height);
    constexpr auto weight = 2;
    algo.SetPhi([&weight](const double h, const double g) { return g + weight * h; });
    for (int n = 0; n < loader.GetNumExperiments(); ++n) {
        const auto &experiment = loader.GetExperiment(n);
        const auto start = gppc::Point(experiment.GetStartX(), experiment.GetStartY());
        const auto goal = gppc::Point(experiment.GetGoalX(), experiment.GetGoalY());
        const auto expected = Round(weight * experiment.GetDistance(), 3);
        if (algo(start, goal)) {
            if (const auto length = GetPathLength(algo.GetPath());
                Round(length, 3) > expected ||
                gppc::lib::ValidatePath(map, width, height, algo.GetPath()) != -1) {
                std::cerr << std::setprecision(10) << n << ": " << start << " -> " << goal << " | "
                          << length << "/" << expected << std::endl;
            } else {
                std::cout << std::setprecision(10) << n << ": " << start << " -> " << goal << " | "
                          << length << "/" << expected << std::endl;
            }
        } else if (expected == 0) {
            std::cout << n << ": " << start << " -> " << goal << " | No path" << std::endl;
        } else {
            std::cerr << n << ": " << start << " -> " << goal << " | No path / " << expected
                      << std::endl;
        }
    }
    return 0;
}
