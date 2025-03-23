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

#include "scenario_loader.h"

#include <fstream>
#include <iostream>

namespace gppc::lib {

Experiment::Experiment(const int start_x, const int start_y, const int goal_x, const int goal_y,
                       const int bucket, const double distance, std::string map)
    : start_x_(start_x),
      start_y_(start_y),
      goal_x_(goal_x),
      goal_y_(goal_y),
      scale_x_(kNoScaling),
      scale_y_(kNoScaling),
      bucket_(bucket),
      distance_(distance),
      map_(std::move(map)) {}

Experiment::Experiment(const int start_x, const int start_y, const int goal_x, const int goal_y,
                       const int scale_x, const int scale_y, const int bucket,
                       const double distance, std::string map)
    : start_x_(start_x),
      start_y_(start_y),
      goal_x_(goal_x),
      goal_y_(goal_y),
      scale_x_(scale_x),
      scale_y_(scale_y),
      bucket_(bucket),
      distance_(distance),
      map_(std::move(map)) {}

int Experiment::GetStartX() const { return start_x_; }

int Experiment::GetStartY() const { return start_y_; }

int Experiment::GetGoalX() const { return goal_x_; }

int Experiment::GetGoalY() const { return goal_y_; }

int Experiment::GetXScale() const { return scale_x_; }

int Experiment::GetYScale() const { return scale_y_; }

int Experiment::GetBucket() const { return bucket_; }

double Experiment::GetDistance() const { return distance_; }

std::string Experiment::GetMapName() const { return map_; }

/**
 * Loads the experiments from the scenario file.
 */
ScenarioLoader::ScenarioLoader(std::string filename) : filename_(std::move(filename)) {
    std::ifstream ifs(filename_);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open file: " << filename_ << std::endl;
        return;
    }
    std::string line;
    ifs >> line;
    float version = 0.0f;
    if (line != "version") {
        ifs.seekg(0, std::ios::beg);
    } else {
        ifs >> version;
    }

    int start_x, start_y, goal_x, goal_y;
    int bucket;
    double distance;
    std::string map;
    if (version == 0.0f) {
        while (ifs >> bucket >> map >> start_x >> start_y >> goal_x >> goal_y >> distance) {
            experiments_.emplace_back(start_x, start_y, goal_x, goal_y, bucket, distance, map);
        }
    } else if (version == 1.0f) {
        int scale_x = 0, scale_y = 0;
        while (ifs >> bucket >> map >> scale_x >> scale_y >> start_x >> start_y >> goal_x >>
               goal_y >> distance) {
            experiments_.emplace_back(start_x, start_y, goal_x, goal_y, scale_x, scale_y, bucket,
                                      distance, map);
        }
    } else {
        std::cerr << "Invalid version number: " << version << std::endl;
    }
}

void ScenarioLoader::Save(const std::string& filename) const {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    constexpr float version = 1.0;
    ofs << "version " << version << std::endl;
    for (const auto& experiment : experiments_) {
        ofs << experiment.bucket_ << "\t" << experiment.map_ << "\t" << experiment.scale_x_ << "\t";
        ofs << experiment.scale_y_ << "\t" << experiment.start_x_ << "\t" << experiment.start_y_
            << "\t";
        ofs << experiment.goal_x_ << "\t" << experiment.goal_y_ << "\t" << experiment.distance_
            << std::endl;
    }
    ofs.close();
}

std::size_t ScenarioLoader::GetNumExperiments() const { return experiments_.size(); }

std::string ScenarioLoader::GetScenarioName() const { return filename_; }

const Experiment& ScenarioLoader::GetExperiment(const std::size_t which) const {
    return experiments_[which];
}

void ScenarioLoader::AddExperiment(const Experiment& which) { experiments_.push_back(which); }

}  // namespace gppc::lib
