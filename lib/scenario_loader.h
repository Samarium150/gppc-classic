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

#ifndef LIB_GPPC_SCENARIO_LOADER_H_
#define LIB_GPPC_SCENARIO_LOADER_H_

#include <string>
#include <vector>

namespace gppc::lib {

static constexpr int kNoScaling = -1;

/**
 * The class which loads and stores scenarios from files.
 * Versions currently handled: 0.0 and 1.0 (includes scale).
 */
class ScenarioLoader;

/**
 * Experiments stored by the ScenarioLoader class.
 */
class Experiment {
public:
    Experiment(int start_x, int start_y, int goal_x, int goal_y, int bucket, double distance,
               std::string map);
    Experiment(int start_x, int start_y, int goal_x, int goal_y, int scale_x, int scale_y,
               int bucket, double distance, std::string map);
    [[nodiscard]] int GetStartX() const;
    [[nodiscard]] int GetStartY() const;
    [[nodiscard]] int GetGoalX() const;
    [[nodiscard]] int GetGoalY() const;
    [[nodiscard]] int GetXScale() const;
    [[nodiscard]] int GetYScale() const;
    [[nodiscard]] int GetBucket() const;
    [[nodiscard]] double GetDistance() const;
    [[nodiscard]] std::string GetMapName() const;

private:
    friend class ScenarioLoader;
    int start_x_, start_y_, goal_x_, goal_y_;
    int scale_x_, scale_y_;
    int bucket_;
    double distance_;
    std::string map_;
};

class ScenarioLoader {
public:
    ScenarioLoader() = default;
    explicit ScenarioLoader(std::string filename);
    void Save(const std::string &filename) const;
    [[nodiscard]] std::size_t GetNumExperiments() const;
    [[nodiscard]] std::string GetScenarioName() const;
    [[nodiscard]] const Experiment &GetExperiment(std::size_t which) const;
    [[nodiscard]] const std::vector<Experiment> &Experiments() const;
    void AddExperiment(const Experiment &which);

private:
    std::string filename_;
    std::vector<Experiment> experiments_;
};
}  // namespace gppc::lib
#endif  // LIB_GPPC_SCENARIO_LOADER_H_
