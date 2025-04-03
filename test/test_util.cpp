#include "test_util.h"

#include <filesystem>
#include <fstream>

static Map LoadMap(std::string name, std::ifstream& map_file) noexcept {
    Map map;
    map.name = std::move(name);
    std::string token;
    map_file >> token;
    map_file >> token;
    map_file >> token;
    map_file >> map.height;
    map_file >> token;
    map_file >> map.width;
    map_file >> token;
    map.data.resize(map.width * map.height);
    std::string line;
    std::getline(map_file, line);
    for (size_t y = 0; y < map.height; ++y) {
        std::getline(map_file, line);
        size_t x = 0;
        for (const char c : line) {
            if (std::isspace(static_cast<unsigned char>(c))) {
                continue;
            }
            if (x < map.width) {
                map.data[y * map.width + x] = c == '.' || c == 'G' || c == 'S';
                ++x;
            }
        }
    }
    return map;
}

std::vector<Map> kMaps = {};

std::vector<TestCase> LoadTestCases(const std::string& directory) noexcept {
    std::vector<TestCase> test_cases;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".map") {
            const auto& path = entry.path();
            gppc::lib::ScenarioLoader loader(path.string() + ".scen");  // NOLINT
            if (std::ifstream map_file(path); map_file.is_open()) {
                const auto map_id = kMaps.size();
                kMaps.push_back(LoadMap(path.stem(), map_file));
                for (size_t i = 0; i < loader.GetNumExperiments(); ++i) {
                    const auto& experiment = loader.GetExperiment(i);
                    test_cases.emplace_back(map_id, i, experiment);
                }
            }
        }
    }
    return test_cases;
}

std::string GenerateTestName(const testing::TestParamInfo<TestCase>& info) noexcept {
    return kMaps[info.param.map_id].name + "_case_" + std::to_string(info.param.case_id + 1);
}

static double EuclideanDist(const gppc::Point& a, const gppc::Point& b) noexcept {
    const auto dx = std::abs(a.x - b.x);
    const auto dy = std::abs(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

double GetPathLength(const std::vector<gppc::Point>& path) noexcept {
    double len = 0;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        len += EuclideanDist(path[i], path[i + 1]);
    }
    return len;
}
