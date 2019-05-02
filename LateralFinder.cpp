#include "LateralFinder.h"

bool LateralFinder::find_best()
{
    reset_search();
    constexpr primitives::point_id_t start {0};
    primitives::point_id_t i {start};
    do
    {
        m_swap_end = m_tour.prev(i);
        start_search(i, m_swap_end);
        m_swap_end = m_tour.next(i);
        start_search(i, i);
        i = m_tour.next(i);
    } while (i != start);
    return false;
}

void LateralFinder::start_search(const primitives::point_id_t swap_start
    , const primitives::point_id_t removed_edge)
{
    const auto remove {m_tour.length(removed_edge)};
    m_starts.push_back(swap_start);
    m_removes.push_back(removed_edge);
    std::vector<primitives::point_id_t> points;
    const auto search_box
    {
        m_tour.search_box(swap_start, remove + 1)
    };
    m_root.get_points(swap_start, search_box, points);
    for (auto p : points)
    {
        if (p == swap_start
            or p == m_tour.prev(swap_start)
            or p == m_tour.next(swap_start))
        {
            continue;
        }
        const auto add {m_tour.length(swap_start, p)};
        if (not gainful(add, remove))
        {
            continue;
        }
        m_ends.push_back(p);
        delete_edge(remove, add);
        m_ends.pop_back();
    }
    m_starts.pop_back();
    m_removes.pop_back();
}

void LateralFinder::delete_edge(const primitives::length_t removed
    , const primitives::length_t added)
{
    // each point p in m_removes represents the edge (p, next(p)).
    // here, m_starts.size() == m_ends.size(), and m_removes.size() == m_ends.size()
    const auto prev {m_tour.prev(m_ends.back())};
    auto new_remove {prev};
    const auto has_prev_edge
    {
        std::find(std::cbegin(m_removes), std::cend(m_removes), new_remove)
            == std::cend(m_removes)
    };
    if (has_prev_edge)
    {
        const auto new_start {prev};
        add_edge(new_start, new_remove, removed, added);
    }
    new_remove = m_ends.back();
    const auto has_next_edge
    {
        std::find(std::cbegin(m_removes), std::cend(m_removes), new_remove)
            == std::cend(m_removes)
    };
    if (has_next_edge)
    {
        const auto next {m_tour.next(m_ends.back())};
        const auto new_start {next};
        add_edge(new_start, new_remove, removed, added);
    }
}

void LateralFinder::add_edge(const primitives::point_id_t new_start
    , const primitives::point_id_t new_remove
    , const primitives::length_t removed
    , const primitives::length_t added)
{
    const auto remove {m_tour.length(new_remove)};
    // first check if can close.
    const auto closing_length {m_tour.length(new_start, m_swap_end)};
    const auto total_closing_add {closing_length + added};
    const auto total_remove {removed + remove};
    const bool lateral {total_remove == total_closing_add};
    if (lateral)
    {
        if (new_start != m_tour.prev(m_swap_end) and new_start != m_tour.next(m_swap_end))
        {
            m_starts.push_back(new_start);
            m_ends.push_back(m_swap_end);
            m_removes.push_back(new_remove);
            //std::cout << "lateral" << std::endl;
            auto test_tour = m_tour;
            test_tour.multicycle_swap(m_starts, m_ends, m_removes);
            if (test_tour.cycles() == 1)
            {
                FeasibleFinder finder(m_root, test_tour);
                if (finder.find_best())
                {
                    std::cout << "improved laterally" << std::endl;
                    test_tour.swap(finder.best_starts(), finder.best_ends(), finder.best_removes());
                    m_tour = test_tour;
                    return;
                }
            }
            else
            {
                //std::cout << "missed multicycle" << std::endl;
            }
            m_starts.pop_back();
            m_ends.pop_back();
            m_removes.pop_back();
        }
    }
    if (m_starts.size() >= m_kmax - 1)
    {
        return;
    }

    std::vector<primitives::point_id_t> points;
    const auto margin {total_remove - added};
    const auto search_box
    {
        m_tour.search_box(new_start, margin + m_tour.length(new_remove) + 1)
    };
    m_root.get_points(new_start, search_box, points);
    for (auto p : points)
    {
        // check easy exclusion cases.
        const bool closing {p == m_swap_end}; // closing should already have been checked.
        const bool neighboring {p == m_tour.next(new_start) or p == m_tour.prev(new_start)};
        const bool self {p == new_start};
        const bool backtrack {p == m_starts.back()};
        if (backtrack or self or closing or neighboring)
        {
            continue;
        }

        // check if worth considering.
        const auto add {m_tour.length(new_start, p)};
        if (not gainful(add, margin))
        {
            continue;
        }

        // check if repeating move.
        const bool has_start
        {
            std::find(std::cbegin(m_starts), std::cend(m_starts), new_start)
                != std::cend(m_starts)
        };
        const bool has_end
        {
            std::find(std::cbegin(m_ends), std::cend(m_ends), p) != std::cend(m_ends)
        };
        if (has_start and has_end)
        {
            continue;
        }

        m_starts.push_back(new_start);
        m_ends.push_back(p);
        m_removes.push_back(new_remove);
        delete_edge(removed + remove, added + add);
        m_starts.pop_back();
        m_ends.pop_back();
        m_removes.pop_back();
    }
}
