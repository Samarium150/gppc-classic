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

std::vector<TestCase> LoadTestCases(const std::string& filename) noexcept {
    std::vector<TestCase> test_cases;
    std::ifstream map_file(filename);
    if (!map_file.is_open()) {
        std::cerr << "cannot open " << filename << std::endl;
        return test_cases;
    }
    kMaps.push_back(LoadMap(filename, map_file));
    const gppc::lib::ScenarioLoader loader(filename + ".scen");  // NOLINT
    for (size_t i = 0; i < loader.GetNumExperiments(); ++i) {
        const auto& experiment = loader.GetExperiment(i);
        test_cases.emplace_back(kMaps.size() - 1, i, experiment);
    }
    return test_cases;
}

std::string GenerateTestName(const testing::TestParamInfo<TestCase>& info) noexcept {
    return "case_" + std::to_string(info.param.case_id + 1);
}

void PrintMap(const std::vector<bool>& map, const int width, const int height,
              const std::optional<gppc::Point> start, const std::optional<gppc::Point> goal) {
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
