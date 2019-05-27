#include "Finder.h"

std::optional<KMove> Finder::find_best()
{
    for (primitives::point_id_t i {0}; i < m_tour.size(); ++i)
    {
        search(i);
        if (m_stop)
        {
            return m_kmove;
        }
    }
    return std::nullopt;
}

void Finder::search(primitives::point_id_t i)
{
    reset_search();
    m_swap_end = m_tour.prev(i);
    delete_prev_edge(i);
    start_search(i, m_swap_end);
    if (m_stop)
    {
        return;
    }

    reset_search();
    m_swap_end = m_tour.next(i);
    delete_next_edge(i);
    start_search(i, i);
    if (m_stop)
    {
        return;
    }
}

void Finder::delete_prev_edge(primitives::point_id_t new_edge_start)
{
    const auto removed_edge_start = m_tour.prev(new_edge_start);
    m_kmove.push_deletion(new_edge_start, removed_edge_start);
    m_kmargin.increase(m_tour.length(removed_edge_start));
}

void Finder::delete_prev_edge()
{
    delete_prev_edge(m_kmove.ends.back());
}

void Finder::delete_next_edge(primitives::point_id_t new_edge_start)
{
    const auto removed_edge_start = new_edge_start;
    m_kmove.push_deletion(new_edge_start, removed_edge_start);
    m_kmargin.increase(m_tour.length(removed_edge_start));
}

void Finder::delete_next_edge()
{
    delete_next_edge(m_kmove.ends.back());
}

void Finder::undo_deletion()
{
    m_kmove.pop_deletion();
    m_kmargin.pop_increase();
}

bool Finder::add_new_edge(primitives::point_id_t new_edge_end)
{
    const auto new_edge_start = m_kmove.starts.back();
    const auto new_length = m_tour.length(new_edge_start, new_edge_end);
    if (not m_kmargin.decrease(new_length))
    {
        return false;
    }
    m_kmove.push_addition(new_edge_end);
    return true;
}

void Finder::delete_new_edge()
{
    m_kmargin.pop_decrease();
    m_kmove.pop_addition();
}

void Finder::start_search(const primitives::point_id_t swap_start
    , const primitives::point_id_t removed_edge)
{
    const auto remove {m_tour.length(removed_edge)};
    const auto points = m_root.get_points(swap_start, m_box_maker(swap_start, remove + 1));
    for (auto p : points)
    {
        if (p == swap_start
            or p == m_tour.prev(swap_start)
            or p == m_tour.next(swap_start))
        {
            continue;
        }
        const auto add {m_tour.length(swap_start, p)};
        if (not add_new_edge(p))
        {
            continue;
        }
        delete_edge(remove, add);
        if (m_stop)
        {
            return;
        }
        delete_new_edge();
    }
}

void Finder::delete_both_edges()
{
    delete_both_edges(m_kmove.ends.back());
}

void Finder::delete_both_edges(primitives::point_id_t p)
{
    if (m_kmove.removable(p))
    {
        delete_next_edge(p);
        try_nearby_points();
        if (m_stop)
        {
            return;
        }
        undo_deletion();
    }

    p = m_tour.prev(p);
    if (m_kmove.removable(p))
    {
        delete_next_edge(p);
        try_nearby_points();
        if (m_stop)
        {
            return;
        }
        undo_deletion();
    }
}

void Finder::check_close()
{
    if (not add_new_edge(m_swap_end))
    {
        return;
    }
    const auto close_edge_start = m_kmove.starts.back();
    const auto remake_prev_edge = close_edge_start == m_tour.prev(m_swap_end);
    const auto remake_next_edge = close_edge_start == m_tour.next(m_swap_end);
    if (not remake_prev_edge and not remake_next_edge)
    {
        // TODO: pass KMove to feasible()
        if (cycle_check::feasible(m_tour, m_kmove.starts, m_kmove.ends, m_kmove.removes))
        {
            m_stop = true;
            return;
        }
    }
    delete_new_edge();
}

void Finder::try_nearby_points()
{
    // TODO: right place for this check?
    if (m_kmove.size() >= m_kmax - 1)
    {
        return;
    }

    const auto start = m_kmove.starts.back();
    const auto search_radius = m_kmargin.total_margin + 1;
    const auto points = m_root.get_points(start, m_box_maker(start, search_radius));
    for (auto p : points)
    {
        // check easy exclusion cases.
        // TODO: instead of prior close check, check here (slightly more efficient).
        const bool closing {p == m_swap_end}; // closing should already have been checked.
        const bool old_edge {p == m_tour.next(start) or p == m_tour.prev(start)};
        const bool self {p == start};
        const bool backtrack {(not m_kmove.ends.empty()) and p == m_kmove.ends.back()};
        if (backtrack or self or closing or old_edge)
        {
            continue;
        }

        // check if worth considering.
        if (not add_new_edge(p))
        {
            continue;
        }
        if (p == m_swap_end)
        {
            // TODO: pass KMove to feasible()
            if (cycle_check::feasible(m_tour, m_kmove.starts, m_kmove.ends, m_kmove.removes))
            {
                m_stop = true;
                return;
            }
        }
        // check if repeating move.
        // TODO: this might be overly exclusive.
        if (not m_kmove.has_start(start) or not m_kmove.has_end(p))
        {
            // TODO
            delete_both_edges();
            if (m_stop)
            {
                return;
            }
        }
        delete_new_edge();
    }
}

void Finder::delete_edge(const primitives::length_t removed
    , const primitives::length_t added)
{
    const auto prev = m_tour.prev(m_kmove.newest_point());
    if (m_kmove.removable(prev))
    {
        add_edge(prev, prev, removed, added);
        if (m_stop)
        {
            return;
        }
    }

    const auto newest_point = m_kmove.newest_point();
    if (m_kmove.removable(newest_point))
    {
        const auto next = m_tour.next(newest_point);
        add_edge(next, newest_point, removed, added);
        if (m_stop)
        {
            return;
        }
    }
}

void Finder::add_edge(const primitives::point_id_t new_start
    , const primitives::point_id_t new_remove
    , const primitives::length_t removed
    , const primitives::length_t added)
{
    const auto remove {m_tour.length(new_remove)};
    // first check if can close.
    const auto closing_length {m_tour.length(new_start, m_swap_end)};
    const auto total_closing_add {closing_length + added};
    const auto total_remove {removed + remove};
    const bool improving {total_remove > total_closing_add};
    if (improving)
    {
        if (new_start != m_tour.prev(m_swap_end) and new_start != m_tour.next(m_swap_end))
        {
            m_kmove.push_all(new_start, m_swap_end, new_remove);
            if (cycle_check::feasible(m_tour, m_kmove.starts, m_kmove.ends, m_kmove.removes))
            {
                m_stop = true;
                return;
            }
            m_kmove.pop_all();
        }
    }

    if (m_kmove.size() >= m_kmax - 1)
    {
        return;
    }

    const auto margin {total_remove - added};
    const auto search_radius = margin + m_tour.length(new_remove) + 1;
    const auto points = m_root.get_points(new_start, m_box_maker(new_start, search_radius));
    for (auto p : points)
    {
        // check easy exclusion cases.
        const bool closing {p == m_swap_end}; // closing should already have been checked.

        const bool neighboring {p == m_tour.next(new_start) or p == m_tour.prev(new_start)};
        const bool self {p == new_start};
        const bool backtrack {p == m_kmove.starts.back()};
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
        if (m_kmove.has_start(new_start) and m_kmove.has_end(p))
        {
            continue;
        }

        m_kmove.push_all(new_start, p, new_remove);
        delete_edge(removed + remove, added + add);
        if (m_stop)
        {
            return;
        }
        m_kmove.pop_all();
    }
}

void Finder::reset_search()
{
    m_kmove.clear();
    m_kmargin.clear();
    m_swap_end = constants::invalid_point;
    m_stop = false;
}

bool Finder::gainful(primitives::length_t new_length, primitives::length_t removed_length) const
{
    return new_length <= removed_length;
}

