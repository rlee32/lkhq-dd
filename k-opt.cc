#include "NanoTimer.h"
#include "check.hh"
#include "config.hh"
#include "fileio.hh"
#include "hill_climb.hh"
#include "hill_climber.hh"
#include "merge/merge.hh"
#include "perturb.hh"
#include "point_quadtree/Domain.h"
#include "point_quadtree/point_quadtree.h"
#include "randomize/double_bridge.h"
#include "tour.hh"
#include "multicycle_tour.hh"
#include "two_short.hh"

#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

int main(int argc, const char** argv)
{
    if (argc == 1) {
        std::cout << "Arguments: config_file_path" << std::endl;
    }
    // Read config file.
    const std::string config_path = (argc == 1) ? "config.txt" : argv[1];
    std::cout << "Reading config file: " << config_path << std::endl;
    Config config(config_path);

    // Read input files.
    const std::optional<std::string> tsp_file_path_string = config.get("tsp_file_path");
    if (not tsp_file_path_string) {
        std::cout << "tsp_file_path not specified.\n";
        return EXIT_FAILURE;
    }
    const std::optional<std::filesystem::path> tsp_file_path(*tsp_file_path_string);
    const auto [x, y] = fileio::read_coordinates(*tsp_file_path_string);
    const auto initial_tour = fileio::initial_tour(x.size(), config.get("tour_file_path"));

    // Initial tour length calculation.
    point_quadtree::Domain domain(x, y);
    std::cout << "domain aspect ratio: " << domain.xdim(0) / domain.ydim(0) << std::endl;
    std::cout << "bounding x, y dim: "
        << domain.xdim(0) << ", " << domain.ydim(0)
        << std::endl;

    Tour tour(&domain, initial_tour);
    const auto initial_tour_length = tour.length();
    std::cout << "Initial tour length: " << initial_tour_length << std::endl;

    // Quad tree.
    NanoTimer timer;
    timer.start();

    std::cout << "\nquadtree stats:\n";
    const auto root {point_quadtree::make_quadtree(x, y, domain)};
    std::cout << "node ratio: "
        << static_cast<double>(point_quadtree::count_nodes(root))
            / point_quadtree::count_points(root)
        << std::endl;
    std::cout << "Finished quadtree in " << timer.stop() / 1e9 << " seconds.\n\n";

    auto best_length = initial_tour_length;

    const auto &save_prefix = tsp_file_path->stem().string();
    const auto &save_dir_string = config.get("save_dir");
    std::optional<std::filesystem::path> save_dir;
    if (save_dir_string) {
        save_dir = std::filesystem::path(*save_dir_string);
        if (not std::filesystem::exists(*save_dir)) {
            std::filesystem::create_directory(*save_dir);
        }
    }
    auto write_if_better = [&](primitives::length_t new_length)
    {
        if (new_length < best_length)
        {
            if (save_dir) {
                const auto &save_path = *save_dir / (save_prefix + '_' + std::to_string(new_length) + ".tour");
                fileio::write_ordered_points(tour.order(), save_path);
                std::cout << "saved tour to " << save_path << std::endl;
            }
            best_length = new_length;
        }
    };

    PointSet point_set(root, x, y);

    // hill climb from initial tour.
    HillClimber hill_climber(point_set);
    const auto &kmax = config.get<size_t>("kmax", 3);
    std::cout << "kmax: " << kmax << std::endl;
    auto new_length = hill_climb::hill_climb(hill_climber, tour, kmax);
    if (new_length < best_length) {
        best_length = new_length;
        std::cout << "new improved length: " << new_length << std::endl;
    }
    write_if_better(new_length);

    constexpr bool RUN_EXPERIMENTAL{false};
    if (RUN_EXPERIMENTAL) {
        // temporary experimental output.
        const auto &short_edge_set = two_short::get_short_edges(point_set, tour);
        std::vector<edge::Edge> short_edges(std::cbegin(short_edge_set), std::cend(short_edge_set));
        fileio::write_pairs(short_edges, "output/short_edges.txt");
        std::cout << "ratio of short edges to instance size: "
            << static_cast<double>(short_edges.size()) / point_set.size()
            << std::endl;
        const auto &perturbation_kmove = two_short::make_perturbation(tour, short_edges);
        std::cout << "replacement ratio: " << static_cast<double>(perturbation_kmove.current_k()) / point_set.size() << std::endl;

        // temporary experimental output.
        MulticycleTour mt(tour);
        mt.multicycle_swap(perturbation_kmove);
        std::cout << "post-swap cycle count: " << mt.cycles() << std::endl;
        std::cout << "min cycle size: " << mt.min_cycle_size() << std::endl;
        const auto &merging_kmove = perturb::random_cycle_merge_move(mt);
        if (merging_kmove) {
            mt.multicycle_swap(*merging_kmove);
            std::cout << "cycles after merging move: " << mt.cycles() << std::endl;
            fileio::write_ordered_points(mt.update_order(), "output/merged_tour.txt");
            hill_climber.changed(*merging_kmove);
            auto new_length = hill_climb::hill_climb(hill_climber, mt, kmax);
            std::cout << "post-merge post-climb length: " << new_length << std::endl;
        }
    }

    // perturbation loop.
    size_t local_optima{1};
    const auto &kmax_kswap = config.get<size_t>("kmax_kswap", 10);
    std::cout << "kmax_kswap: " << kmax_kswap << std::endl;
    do {
        //const auto new_tour = perturb::perturb(hill_climber, tour, kmax);
        //const auto new_tour = perturb::random_restart(point_set, &domain, kmax);
        //const auto new_tour = perturb::random_section(hill_climber, tour, kmax, 0.05);
        const auto new_tour = perturb::kswap(hill_climber, tour, kmax, kmax_kswap);
        check::check_tour(new_tour);
        const auto kmove = merge::merge(tour, new_tour);
        if (kmove) {
            hill_climber.changed(*kmove);
            hill_climb::hill_climb(hill_climber, tour, kmax);
        }
        write_if_better(tour.length());
        std::cout << "best length: " << best_length << std::endl;
        ++local_optima;
        std::cout << "local optima: " << local_optima << std::endl;
    } while (true);

    return EXIT_SUCCESS;
}
