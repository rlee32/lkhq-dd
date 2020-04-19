#pragma once

#include <algorithm>
#include <random>
#include <primitives.hh>

namespace randomize {

// random integer in [a, b].
inline primitives::point_id_t random_point(primitives::sequence_t a, primitives::sequence_t b) {
    static std::random_device device; // will be used to obtain a seed for the random number engine
    static std::mt19937 generator(device()); // standard mersenne_twister_engine seeded with random_device.
    std::uniform_int_distribution<primitives::point_id_t> distribution(a, b);
    return distribution(generator);
}

inline primitives::sequence_t sequence(primitives::sequence_t max) {
    return random_point(0, max);
}

// selects at random k unique points in the range [start, end).
// TODO: try to do better than O(end - start).
inline std::vector<primitives::sequence_t> random_set(primitives::sequence_t start, primitives::sequence_t end, size_t k) {
    const auto &n = end - start;
    std::vector<primitives::sequence_t> candidates(n);
    for (primitives::sequence_t s{0}; s < n; ++s) {
        candidates[s] = s;
    }
    const auto &begin = std::begin(candidates);
    static std::random_device device; // will be used to obtain a seed for the random number engine
    static std::mt19937 generator(device()); // standard mersenne_twister_engine seeded with random_device.
    std::shuffle(begin, std::end(candidates), generator);
    std::vector<primitives::sequence_t> selection(begin, begin + k);
    std::sort(std::begin(selection), std::end(selection));
    return selection;
}

inline bool random_bool() {
    return (sequence(1) & 1) == 0;
}

}  // namespace randomize


