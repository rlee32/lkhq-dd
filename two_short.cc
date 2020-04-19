#include "two_short.hh"
#include "randomize/randomize.hh"
#include <unordered_set>

namespace two_short {

namespace {

// gets the non-sequential, cycle-breaking 2-opt when removing edge after first point in new edge.
std::optional<KMove> kmove_remove_next(const Tour &tour, const edge::Edge &new_edge) {
    KMove kmove;
    kmove.starts.push_back(new_edge.first);
    kmove.ends.push_back(new_edge.second);
    kmove.starts.push_back(tour.next(new_edge.first));
    kmove.ends.push_back(tour.prev(new_edge.second));
    kmove.removes.push_back(new_edge.first);
    kmove.removes.push_back(tour.prev(new_edge.second));
    return kmove.valid() ? std::make_optional(kmove) : std::nullopt;
}

// gets the non-sequential, cycle-breaking 2-opt when removing edge before first point in new edge.
std::optional<KMove> kmove_remove_prev(const Tour &tour, const edge::Edge &new_edge) {
    KMove kmove;
    kmove.starts.push_back(new_edge.first);
    kmove.ends.push_back(new_edge.second);
    kmove.starts.push_back(tour.prev(new_edge.first));
    kmove.ends.push_back(tour.next(new_edge.second));
    kmove.removes.push_back(tour.prev(new_edge.first));
    kmove.removes.push_back(new_edge.second);
    return kmove.valid() ? std::make_optional(kmove) : std::nullopt;
}

template <typename EdgeContainer, typename PointContainer>
bool is_compatible(const PointContainer &removes, const EdgeContainer &additions, const KMove &kmove) {
    for (size_t k{0}; k < kmove.current_k(); ++k) {
        if (additions.find(edge::make_edge(kmove.starts[k], kmove.ends[k])) != std::cend(additions)) {
            return false;
        }
        if (removes.find(kmove.removes[k]) != std::cend(removes)) {
            return false;
        }
    }
    return true;
}

template <typename EdgeContainer, typename PointContainer>
void insert(PointContainer &removes, EdgeContainer &additions, const KMove &kmove) {
    for (size_t k{0}; k < kmove.current_k(); ++k) {
        additions.insert(edge::make_edge(kmove.starts[k], kmove.ends[k]));
        removes.insert(kmove.removes[k]);
    }
}

}  // namespace

std::set<edge::Edge> get_short_edges(const PointSet &point_set, const Tour &tour) {
    std::set<edge::Edge> short_edges;
    for (primitives::point_id_t i{0}; i < point_set.size(); ++i) {
        const auto &next_length = tour.length(i);
        const auto &prev_length = tour.prev_length(i);
        const auto &max_length = std::max(prev_length, next_length);
        const auto &search_radius = max_length + 1;
        const auto &points = point_set.get_points(i, search_radius);
        std::vector<primitives::point_id_t> filtered_points;
        for (const auto &point : points) {
            if (point == tour.next(i) or point == tour.prev(i) or point == i) {
                continue;
            }
            if (point_set.length(point , i) >= max_length) {
                continue;
            }
            filtered_points.push_back(point);
        }
        if (filtered_points.empty()) {
            continue;
        }
        for (const auto &point : filtered_points) {
            short_edges.insert(edge::make_edge(i, point));
        }
    }
    return short_edges;
}

KMove make_perturbation(const Tour &tour, std::vector<edge::Edge> &short_edges) {
    static std::random_device device; // will be used to obtain a seed for the random number engine
    static std::mt19937 generator(device()); // standard mersenne_twister_engine seeded with random_device.
    std::shuffle(std::begin(short_edges), std::end(short_edges), generator);
    KMove large_kmove;
    std::unordered_set<primitives::point_id_t> removed;
    std::set<edge::Edge> added;
    for (const auto &edge : short_edges) {
        // max_replacement_ratio is in [0, 1].
        // it is essentially k / n, where k is the k-value for the perturbing k-opt move,
        // and n is the instance size.
        constexpr double MAX_REPLACEMENT_RATIO{0.01};
        if (large_kmove.current_k() >= MAX_REPLACEMENT_RATIO * tour.size()) {
            break;
        }
        if (randomize::random_bool()) {
            // try PREV first, then NEXT if PREV is not viable.
            auto kmove = kmove_remove_prev(tour, edge);
            if (kmove and is_compatible(removed, added, *kmove)) {
                insert(removed, added, *kmove);
                large_kmove += *kmove;
                continue;
            }
            kmove = kmove_remove_next(tour, edge);
            if (kmove and is_compatible(removed, added, *kmove)) {
                insert(removed, added, *kmove);
                large_kmove += *kmove;
                continue;
            }
        } else {
            // try NEXT first, then PREV if NEXT is not viable.
            auto kmove = kmove_remove_next(tour, edge);
            if (kmove and is_compatible(removed, added, *kmove)) {
                insert(removed, added, *kmove);
                large_kmove += *kmove;
                continue;
            }
            kmove = kmove_remove_prev(tour, edge);
            if (kmove and is_compatible(removed, added, *kmove)) {
                insert(removed, added, *kmove);
                large_kmove += *kmove;
                continue;
            }
        }
    }
    return large_kmove;
}

}  // namespace two_short
