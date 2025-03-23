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

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <string>
#include <vector>

#include "entry.h"
#include "gppc.h"
#include "path_validator.h"
#include "scenario_loader.h"
#include "timer.h"

using namespace gppc;

namespace fs = std::filesystem;

namespace {
const std::string kIndexDir = "index_data";
constexpr double kPathFirstStepLength = 20.0;

std::string g_data;
std::string g_map;
std::string g_scene;
std::string g_flag;
std::vector<bool> g_map_data;
int g_width;
int g_height;
bool g_pre = false;
bool g_run = false;
bool g_check = false;
}  // namespace

void LoadMap(const std::string &filename, std::vector<bool> &map, int &width, int &height) {
    std::ifstream ifs(filename);
    if (!ifs) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
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
    for (int y = 0; y < height; y++) {
        std::getline(ifs, line);
        int x = 0;
        for (const char c : line) {
            if (std::isspace(static_cast<unsigned char>(c))) {
                continue;
            }
            if (x < width) {
                map[y * width + x] = (c == '.' || c == 'G' || c == 'S');
                x++;
            }
        }
    }
}

double EuclideanDist(const gppc::xyLoc &a, const gppc::xyLoc &b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

double GetPathLength(const std::vector<gppc::xyLoc> &path) {
    double len = 0;
    for (size_t i = 0; i + 1 < path.size(); i++) {
        len += EuclideanDist(path[i], path[i + 1]);
    }
    return len;
}

int ValidatePath(const std::vector<gppc::xyLoc> &the_path) {
    return lib::ValidatePath(g_map_data, g_width, g_height, the_path);
}

void RunExperiment(void *data) {
    lib::Timer t;
    lib::ScenarioLoader loader(g_scene);
    std::vector<gppc::xyLoc> path;

    const std::string result_file = "result.csv";
    std::ofstream ofs(result_file);
    const std::string header =
        "map,scen,experiment_id,path_size,path_length,ref_length,time_cost,20steps_cost,"
        "max_step_time";
    ofs << header << std::endl;

    for (int x = 0; x < loader.GetNumExperiments(); x++) {
        xyLoc s{static_cast<int16_t>(loader.GetExperiment(x).GetStartX()),
                static_cast<int16_t>(loader.GetExperiment(x).GetStartY())};
        xyLoc g{static_cast<int16_t>(loader.GetExperiment(x).GetGoalX()),
                static_cast<int16_t>(loader.GetExperiment(x).GetGoalY())};

        path.clear();
        using dur = lib::Timer::duration;
        dur max_step = dur::zero(), t_cost = dur::zero(), t_cost_first = dur::zero();
        bool done = false, done_first = false;
        do {
            t.StartTimer();
            done = GetPath(data, s, g, path);
            t.EndTimer();
            max_step = std::max(max_step, t.GetElapsedTime());
            t_cost += t.GetElapsedTime();
            if (!done_first) {
                t_cost_first += t.GetElapsedTime();
                done_first = GetPathLength(path) >= kPathFirstStepLength - 1e-6;
            }
        } while (!done);
        double path_len = GetPathLength(path);
        double ref_len = loader.GetExperiment(x).GetDistance();

        ofs << std::setprecision(9) << std::fixed;
        ofs << g_map << "," << g_scene << "," << x << "," << path.size() << "," << path_len << ","
            << ref_len << "," << t_cost.count() << "," << t_cost_first.count() << ","
            << max_step.count() << std::endl;

        if (g_check) {
            std::cout << s.x << " " << s.y << " " << g.x << " " << g.y;
            if (int valid = ValidatePath(path); valid < 0) {
                std::cout << " valid";
            } else {
                std::cout << " invalid-" << valid;
            }
            std::cout << " " << path.size();
            for (const auto &[x, y] : path) {
                std::cout << " " << x << " " << y;
            }
            std::cout << " " << std::setprecision(5) << path_len << std::endl;
        }
    }
}

void PrintHelpInfo(char **argv) {
    std::cout << "Invalid Arguments\nUsage " << argv[0] << " <flag> <map> <scenario>\n";
    std::cout << "Flags:\n";
    std::cout << "\t-full : Preprocess map and run scenario\n";
    std::cout << "\t-pre  : Preprocess map\n";
    std::cout << "\t-run  : Run scenario without preprocessing\n";
    std::cout << "\t-check: Run for validation\n";
}

bool Parse(const int argc, char **argv) {
    if (argc < 2) {
        return false;
    }
    g_flag = std::string(argv[1]);
    if (g_flag == "-full") {
        g_pre = g_run = true;
    } else if (g_flag == "-pre") {
        g_pre = true;
    } else if (g_flag == "-run") {
        g_run = true;
    } else if (g_flag == "-check") {
        g_run = g_check = true;
    } else {
        return false;
    }

    if (argc < 3) {
        return false;
    }
    g_map = std::string(argv[2]);

    if (g_run) {
        if (argc < 4) {
            return false;
        }
        g_scene = std::string(argv[3]);
    }
    return true;
}

std::string GetBasename(const std::string &path) {
    const fs::path p(path);
    return p.stem().string();
}

int main(const int argc, char **argv) {
    if (!Parse(argc, argv)) {
        PrintHelpInfo(argv);
        return 1;
    }

    LoadMap(g_map, g_map_data, g_width, g_height);
    g_data = kIndexDir + "/" + GetName() + "-" + GetBasename(g_map);

    if (g_pre) {
        PreprocessMap(g_map_data, g_width, g_height, g_data);
    }

    if (!g_run) {
        return 0;
    }

    void *reference = PrepareForSearch(g_map_data, g_width, g_height, g_data);

    RunExperiment(reference);
    return 0;
}
