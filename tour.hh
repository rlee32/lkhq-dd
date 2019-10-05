#pragma once

#include "box.hh"
#include "box_maker.hh"
#include "kmove.hh"
#include "length_calculator.hh"
#include "constants.h"
#include "point_quadtree/Domain.h"
#include "point_quadtree/node.hh"
#include "primitives.hh"

#include <algorithm> // fill
#include <array>
#include <cstdlib> // abort
#include <iostream>
#include <random> // sample
#include <stdexcept>
#include <vector>

class Tour
{
    using Adjacents = std::array<primitives::point_id_t, 2>;
public:
    Tour() = default;
    Tour(const point_quadtree::Domain* domain
        , const std::vector<primitives::point_id_t>& initial_tour);

    template <typename PointContainer = std::vector<primitives::point_id_t>>
    void swap(const PointContainer& starts, const PointContainer& ends, const PointContainer& removed_edges);
    void swap(const KMove&);
    template <typename SequenceContainer = std::vector<primitives::sequence_t>>
    KMove swap_sequence(SequenceContainer starts, SequenceContainer ends, SequenceContainer edges_to_remove);

    const auto &next() const { return m_next; }
    const auto &order() const { return m_order; }

    auto next(primitives::point_id_t i) const { return m_next[i]; }
    primitives::point_id_t prev(primitives::point_id_t i) const;

    size_t size() const { return m_next.size(); }

    primitives::sequence_t sequence(primitives::point_id_t i, primitives::point_id_t start) const;

    const auto& x() const { return m_domain->x(); }
    const auto& y() const { return m_domain->y(); }
    auto x(primitives::point_id_t i) const { return x()[i]; }
    auto y(primitives::point_id_t i) const { return y()[i]; }

    // total length of tour.
    primitives::length_t length() const;
    primitives::length_t length(primitives::point_id_t i) const;
    primitives::length_t prev_length(primitives::point_id_t i) const;
    primitives::length_t length(primitives::point_id_t i, primitives::point_id_t j) const;

    auto domain() const { return m_domain; }

    Box search_box(primitives::point_id_t i, primitives::length_t radius) const;

    const auto &adjacents() const { return m_adjacents; }

    // throws if invalid tour.
    void validate() const;

    void print_first_cycle() const
    {
        constexpr primitives::point_id_t start {0};
        primitives::point_id_t current {start};
        std::vector<bool> visited(size(), false);
        size_t counter {0};
        do
        {
            //std::cout << current << std::endl;
            current = m_next[current];
            visited[current] = true;
            ++counter;
        } while (current != start);
        std::cout << __func__ << ": first cycle size: " << counter << std::endl;
        //std::cout << "outgroup:" << std::endl;
        for (size_t i {0}; i < size(); ++i)
        {
            if (not visited[i])
            {
                //std::cout << i << std::endl;
            }
        }
    }

    void print() const
    {
        constexpr primitives::point_id_t start {0};
        primitives::point_id_t current {start};
        do
        {
            std::cout << current << std::endl;
            current = m_next[current];
        } while (current != start);
    }

    std::vector<primitives::point_id_t> get_points(
        const point_quadtree::Node& root
        , primitives::point_id_t i
        , primitives::length_t radius) const {
        return root.get_points(i, m_box_maker(i, radius));
    }

protected:
    const point_quadtree::Domain* m_domain {nullptr};
    std::vector<Adjacents> m_adjacents;
    std::vector<primitives::point_id_t> m_next;
    std::vector<primitives::sequence_t> m_sequence;
    std::vector<primitives::point_id_t> m_order;
    BoxMaker m_box_maker;
    LengthCalculator m_length_calculator;

    void reset_adjacencies(const std::vector<primitives::point_id_t>& initial_tour);
    void update_next(const primitives::point_id_t start = 0);

    primitives::point_id_t get_other(primitives::point_id_t point, primitives::point_id_t adjacent) const;
    void create_adjacency(primitives::point_id_t point1, primitives::point_id_t point2);
    void fill_adjacent(primitives::point_id_t point, primitives::point_id_t new_adjacent);
    void break_adjacency(primitives::point_id_t i);
    void break_adjacency(primitives::point_id_t point1, primitives::point_id_t point2);
    void vacate_adjacent_slot(primitives::point_id_t point, primitives::point_id_t adjacent);

};

template <typename PointContainer>
void Tour::swap(const PointContainer& starts, const PointContainer& ends, const PointContainer& removed_edges)
{
    for (auto p : removed_edges)
    {
        break_adjacency(p);
    }
    for (size_t i {0}; i < starts.size(); ++i)
    {
        create_adjacency(starts[i], ends[i]);
    }
    update_next();
}

template <typename SequenceContainer>
KMove Tour::swap_sequence(SequenceContainer starts, SequenceContainer ends, SequenceContainer edges_to_remove) {
    std::transform(std::begin(starts), std::end(starts), std::begin(starts), [this](const auto& sequence) { return m_order[sequence]; });
    std::transform(std::begin(ends), std::end(ends), std::begin(ends), [this](const auto& sequence) { return m_order[sequence]; });
    std::transform(std::begin(edges_to_remove), std::end(edges_to_remove), std::begin(edges_to_remove), [this](const auto& sequence) { return m_order[sequence]; });
    KMove kmove;
    kmove.starts = starts;
    kmove.ends = ends;
    kmove.removes = edges_to_remove;
    swap(kmove);
    return kmove;
}

