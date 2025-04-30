#ifndef GPPC_SEARCH_H_
#define GPPC_SEARCH_H_

#include <limits>

#include "grid.h"
#include "open_closed_list.h"

namespace gppc::algorithm {

template <typename T>
class HeuristicSearch {
public:
    HeuristicSearch() noexcept = default;

    HeuristicSearch(const std::vector<bool>& map, size_t width, size_t height) noexcept
        : grid_(std::make_shared<Grid>(map, width, height)) {}

    explicit HeuristicSearch(const std::shared_ptr<Grid>& grid) noexcept : grid_(grid) {}

    virtual ~HeuristicSearch() noexcept = default;

    [[nodiscard]] virtual const std::vector<Point>& GetPath() const noexcept { return path_; }

    [[nodiscard]] virtual size_t GetNodeExpanded() const noexcept { return node_expanded_; }

    [[nodiscard]] virtual T GetNode(const size_t index) const noexcept {
        return open_closed_list_.GetNode(index).value_or(T{});
    }

    [[nodiscard]] virtual T PeekOpen() const noexcept { return open_closed_list_.PeekOpen(); }

    [[nodiscard]] virtual bool EmptyOpen() const noexcept { return open_closed_list_.EmptyOpen(); }

    auto& SetHeuristic(std::function<double(const Point&, const Point&)> heuristic) noexcept {
        heuristic_ = std::move(heuristic);
        return *this;
    }

    auto& SetPhi(std::function<double(double, double)> phi) noexcept {
        phi_ = std::move(phi);
        return *this;
    }

    auto& StopAfterGoal(const bool stop) noexcept {
        stop_after_goal_ = stop;
        return *this;
    }

    virtual bool Init(const Point& start, const Point& goal) noexcept = 0;

    virtual bool Init(const std::shared_ptr<Grid>& grid, const Point& start,
                      const Point& goal) noexcept {
        if (!grid->Get(start) || !grid->Get(goal)) {
            return false;
        }
        grid_ = grid;
        return Init(start, goal);
    }

    virtual bool operator()() noexcept = 0;

    virtual bool operator()(const Point& start, const Point& goal) noexcept {
        if (!Init(start, goal)) {
            return false;
        }
        while (!operator()()) {
        }
        return !path_.empty();
    }

    virtual bool operator()(const std::shared_ptr<Grid>& grid, const Point& start,
                            const Point& goal) noexcept {
        if (!Init(grid, start, goal)) {
            return false;
        }
        while (!operator()()) {
        }
        return !path_.empty();
    }

protected:
    std::shared_ptr<Grid> grid_ = nullptr;
    size_t node_expanded_{};
    std::vector<Point> path_{};
    size_t start_id_ = std::numeric_limits<size_t>::max();
    size_t goal_id_ = std::numeric_limits<size_t>::max();
    Point start_{};
    Point goal_{};
    bool stop_after_goal_ = true;
    OpenClosedList<T> open_closed_list_{};
    std::function<double(const Point&, const Point&)> heuristic_ =
        [this](const Point& a, const Point& b) { return grid_ ? grid_->HCost(a, b) : 0.0; };
    std::function<double(double, double)> phi_ = [](const double h, const double g) {
        return g + h;
    };
};

}  // namespace gppc::algorithm

#endif  // GPPC_SEARCH_H_
