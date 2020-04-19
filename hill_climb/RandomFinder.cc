#include "RandomFinder.h"

namespace hill_climb {

std::vector<primitives::point_id_t>
RandomFinder::search_neighborhood(primitives::point_id_t p)
{
    m_max_kdepth = std::max(m_max_kdepth, m_kmove.current_k());
    const auto search_radius = m_kmargin.total_margin + 1;
    // TODO: filter out all points not within kmargin.
    auto points = m_root.get_points(p, m_box_maker(p, search_radius));
    if (m_kmove.current_k() < m_kmax)
    {
        return points;
    }
    std::vector<primitives::point_id_t> filtered;
    for (auto point : points)
    {
        if (point == p)
        {
            continue;
        }
        const auto new_length = m_length_calculator(p, point);
        if (m_kmargin.total_margin <= new_length)
        {
            continue;
        }
        filtered.push_back(point);
    }
    constexpr size_t samples = 2;
    if (filtered.size() <= samples)
    {
        return filtered;
    }
    static std::random_device device; // will be used to obtain a seed for the random number engine
    static std::mt19937 generator(device()); // standard mersenne_twister_engine seeded with random_device.
    std::shuffle(std::begin(filtered), std::end(filtered), generator);
    filtered.resize(samples);
    return filtered;
}

bool RandomFinder::final_new_edge() const
{
    return false;
}

}  // namespace hill_climb
