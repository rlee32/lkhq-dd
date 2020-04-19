#include "fileio.hh"
#include "merge.hh"

#include "edge_map.hh"
#include "exchange_pair.hh"
#include "combinator.hh"
#include "cycle_util.hh"
#include <kmove.hh>
#include <length_calculator.hh>
#include <cycle_check.hh>

#include <fstream>

namespace merge {

namespace {

template <typename AdjacentsContainer>
std::pair<primitives::point_id_t, primitives::point_id_t> sorted_pair(
    const AdjacentsContainer &adjacents,
    primitives::point_id_t i) {
    const auto &a = adjacents[i];
    return {std::min(a[0], a[1]), std::max(a[0], a[1])};
}

// checks if input set is an incrementing sequence duplicates are accepted.
bool is_sequence(std::vector<primitives::point_id_t> &point_container, primitives::point_id_t end) {
    // first check wrap-around.
    auto prev = *std::cbegin(point_container);
    constexpr primitives::point_id_t LOWEST_VALUE{0};
    if (prev == LOWEST_VALUE) {
        for (auto &point : point_container) {
            const auto diff = point - prev;
            if (diff != 1 and diff != 0) {
                break;
            }
            point += end;
            prev = point;
        }
    }
    // now check for simple incrementing sequence.
    for (const auto &point : point_container) {
        const auto diff = point - prev;
        if (diff != 1 and diff != 0) {
            return false;
        }
        prev = point;
    }
    return true;
}

}  // namespace

std::pair<EdgeSet, EdgeSet> edge_differences(const Tour &tour1, const Tour &tour2) {
    const auto &adjacents1 = tour1.adjacents();
    const auto &adjacents2 = tour2.adjacents();
    EdgeSet diff1, diff2;
    for (primitives::point_id_t i{0}; i < adjacents1.size(); ++i) {
        const auto &sorted1 = sorted_pair(adjacents1, i);
        const auto &sorted2 = sorted_pair(adjacents2, i);
        if (sorted1 == sorted2) {
            continue;
        }
        const auto diff11 = sorted1.first != sorted2.first;
        const auto diff12 = sorted1.first != sorted2.second;
        const auto diff21 = sorted1.second != sorted2.first;
        const auto diff22 = sorted1.second != sorted2.second;
        if (diff11 and diff12) {
            const auto j = sorted1.first;
            diff1.emplace(std::min(i, j), std::max(i, j));
        }
        if (diff21 and diff22) {
            const auto j = sorted1.second;
            diff1.emplace(std::min(i, j), std::max(i, j));
        }
        if (diff11 and diff21) {
            const auto j = sorted2.first;
            diff2.emplace(std::min(i, j), std::max(i, j));
        }
        if (diff12 and diff22) {
            const auto j = sorted2.second;
            diff2.emplace(std::min(i, j), std::max(i, j));
        }
    }
    if (diff2.size() != diff1.size()) {
        throw std::logic_error("number of different edges are not the same between tours.");
    }
    return {std::move(diff1), std::move(diff2)};
}

ExchangePair disjoin(ExchangePair &base) {
    ExchangePair disjoined;
    auto new_points = disjoined.current.insert({base.current.pop_edge()});
    while (new_points) {
        new_points = disjoined.candidate.insert(*base.candidate.pop_edges(*new_points));
        if (not new_points) {
            break;
        }
        new_points = disjoined.current.insert(*base.current.pop_edges(*new_points));
    }
    return disjoined;
}

std::vector<ExchangePair> disjoin(const EdgeSet &current_edges, const EdgeSet &candidate_edges) {
    EdgeMap current(current_edges);
    EdgeMap candidate(candidate_edges);
    ExchangePair base{std::move(current), std::move(candidate)};
    std::vector<ExchangePair> exchange_pairs;
    while (not base.empty()) {
        exchange_pairs.push_back(disjoin(base));
    }
    return exchange_pairs;
}

std::optional<KMove> merge(Tour &current_tour, const Tour &candidate_tour) {
    const auto [old_edges, new_edges] = merge::edge_differences(current_tour, candidate_tour);
    if (old_edges.size() != new_edges.size()) {
        throw std::logic_error("edge diff set does not comprise of the same number of edges from both tours.");
    }
    std::cout << "edge diff count: " << old_edges.size() << std::endl;
    if (old_edges.empty()) {
        return std::nullopt;
    }

    std::ofstream old_edge_file("output/old_edges.txt", std::ofstream::out);
    for (const auto &e : old_edges) {
        old_edge_file << e.first << ' ' << e.second << std::endl;
    }
    std::ofstream new_edge_file("output/new_edges.txt", std::ofstream::out);
    for (const auto &e : new_edges) {
        new_edge_file << e.first << ' ' << e.second << std::endl;
    }

    auto exchanges = disjoin(old_edges, new_edges);
    if (exchanges.empty()) {
        return std::nullopt;
    }

    const auto original_exchange_size = exchanges.size();

    // compute improvements for each exchange.
    std::sort(std::begin(exchanges), std::end(exchanges), [&current_tour](auto &lhs, auto &rhs) {
        return lhs.compute_improvement(current_tour.x(), current_tour.y()) > rhs.compute_improvement(current_tour.x(), current_tour.y());
    });

    if (exchanges.front().compute_improvement(current_tour.x(), current_tour.y()) < 0) {
        return std::nullopt;
    }

    // check to see if exchange pair is useless, meaning zero-cost and non-cycle-breaking.
    // returns true if useless, e.g. should be excluded.
    const auto useless = [&current_tour, &candidate_tour](const ExchangePair &ex) {
        if (*ex.improvement > 0) {
            return false;
        }
        std::vector<primitives::sequence_t> sequence;
        for (const auto &pair : ex.candidate.map()) {
            constexpr primitives::point_id_t START_POINT{0};
            sequence.push_back(current_tour.sequence(pair.first, START_POINT));
        }
        std::sort(std::begin(sequence), std::end(sequence));
        return is_sequence(sequence, current_tour.size()) or !cycle_util::breaks_cycle(current_tour, candidate_tour, ex);
    };
    exchanges.erase(std::remove_if(std::begin(exchanges), std::end(exchanges), useless), std::end(exchanges));
    // max gain, exclude too-low edges.
    const int max_total_improvement = std::accumulate(std::cbegin(exchanges), std::cend(exchanges), int(0), [](int sum, const auto &ex) { return sum + std::max(*ex.improvement, 0); });
    std::cout << "max total improvement: " << max_total_improvement << std::endl;
    exchanges.erase(std::remove_if(std::begin(exchanges), std::end(exchanges), [max_total_improvement](const auto &ex) { return *ex.improvement + max_total_improvement <= 0; }), std::end(exchanges));
    std::cout << "removed " << original_exchange_size - exchanges.size() << " useless exchange(s).\n";
    std::cout << exchanges.size() << " distinct exchange(s)." << std::endl;
    size_t i{0};
    for (auto &ex : exchanges) {
        std::cout << i << ": improvement: " << ex.compute_improvement(current_tour.x(), current_tour.y()) << " ("
            << ex.edge_count() << " edges, breaks cycle: "
            << cycle_util::breaks_cycle(current_tour, candidate_tour, ex) << ")" << std::endl;
        ++i;
    }

    Combinator combinator(exchanges, current_tour, candidate_tour);
    combinator.find();
    std::cout << "move combos checked: " << combinator.checks() << std::endl;
    if (combinator.viable_count() > 0) {
        std::cout << "viable combos: " << combinator.viable_count() << std::endl;
    }
    if (combinator.best_combo()) {
        const auto old_length = current_tour.length();
        const auto &kmove = cycle_util::to_kmove(current_tour, candidate_tour, exchanges, *combinator.best_combo());
        current_tour.swap(kmove);
        const auto new_length = current_tour.length();
        if (static_cast<int>(old_length) - static_cast<int>(new_length) != *combinator.best_improvement()) {
            throw std::logic_error("Tour length after swap is inconsistent with expected improvement.");
        }
        return kmove;
    }
    return std::nullopt;
}

}  // namespace merge
