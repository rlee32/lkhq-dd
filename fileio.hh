#pragma once

#include "primitives.hh"

#include <array>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace fileio {

inline void write_ordered_points(const std::vector<primitives::point_id_t>& ordered_points
    , const std::string output_filename)
{
    std::ofstream output_file;
    output_file.open(output_filename);
    output_file << "DIMENSION: " << ordered_points.size() << "\n";
    output_file << "TOUR_SECTION\n";
    for (auto p : ordered_points)
    {
        output_file << p + 1 << "\n";
    }
}

inline std::vector<primitives::point_id_t> read_ordered_points(const std::string &file_path)
{
    std::cout << "\nReading tour file: " << file_path << std::endl;
    std::ifstream file_stream(file_path);
    if (not file_stream.is_open())
    {
        std::cout << __func__ << ": error: could not open file: "
            << file_path << std::endl;
        std::abort();
    }
    size_t point_count{0};
    // header.
    std::string line;
    while (not file_stream.eof())
    {
        std::getline(file_stream, line);
        if (line.find("TOUR_SECTION") != std::string::npos) // header end.
        {
            break;
        }
        if (line.find("DIMENSION") != std::string::npos) // point count.
        {
            std::string point_count_string = line.substr(line.find(':') + 1);
            point_count = std::stoi(point_count_string);
            std::cout << "Number of points according to header: " << point_count << std::endl;
        }
    }
    if (point_count == 0)
    {
        std::cout << __func__ << ": error: no DIMENSION header in the tour file." << std::endl;
        std::abort();
    }
    // point ids.
    std::vector<primitives::point_id_t> point_ids;
    while (not file_stream.eof())
    {
        if (point_ids.size() >= point_count)
        {
            break;
        }
        std::getline(file_stream, line);
        primitives::point_id_t point_id = std::stoi(line);
        point_ids.push_back(point_id - 1); // subtract one to make point id == index.
    }
    std::cout << "Finished reading tour file.\n" << std::endl;
    return point_ids;
}

inline std::vector<primitives::point_id_t> default_tour(primitives::point_id_t point_count)
{
    std::vector<primitives::point_id_t> tour;
    for (primitives::point_id_t i{0}; i < point_count; ++i) {
        tour.push_back(i);
    }
    return tour;
}

inline std::vector<primitives::point_id_t> initial_tour(primitives::point_id_t point_count, const std::optional<std::string> &tour_file_path = std::nullopt)
{
    return tour_file_path ? read_ordered_points(*tour_file_path) : default_tour(point_count);
}

inline std::array<std::vector<primitives::space_t>, 2> read_coordinates(const std::string &file_path)
{
    std::cout << "\nReading point set file: " << file_path << std::endl;
    std::ifstream file_stream(file_path);
    if (not file_stream.is_open())
    {
        std::cout << "Could not open file: " << file_path << std::endl;
        std::exit(EXIT_SUCCESS);
    }
    size_t point_count{0};
    // header.
    std::string line;
    while (not file_stream.eof())
    {
        std::getline(file_stream, line);
        if (line.find("NODE_COORD_SECTION") != std::string::npos) // header end.
        {
            break;
        }
        if (line.find("DIMENSION") != std::string::npos) // point count.
        {
            std::string point_count_string = line.substr(line.find(':') + 1);
            point_count = std::stoi(point_count_string);
            std::cout << "Number of points according to header: " << point_count << std::endl;
        }
    }
    if (point_count == 0)
    {
        std::cout << "Could not read any points from the point set file." << std::endl;
        std::exit(EXIT_SUCCESS);
    }

    // read coordinates.
    std::vector<primitives::space_t> x, y;
    while (not file_stream.eof())
    {
        if (x.size() >= point_count)
        {
            break;
        }
        std::getline(file_stream, line);
        std::stringstream line_stream(line);
        primitives::point_id_t point_id{0};
        line_stream >> point_id;
        if (point_id == x.size() + 1)
        {
            primitives::space_t value{0};
            line_stream >> value;
            x.push_back(value);
            line_stream >> value;
            y.push_back(value);
        }
        else
        {
            std::cout << __func__ << ": error: point id ("
                << point_id
                << ") does not match number of currently read points ("
                << x.size()
                << ")." << std::endl;
            std::exit(EXIT_SUCCESS);
        }
    }
    std::cout << "Finished reading point set file.\n" << std::endl;
    return {x, y};
}

template <typename PairContainer>
void write_pairs(const PairContainer &pairs, const std::string &output_file_path) {
    std::ofstream output_file(output_file_path, std::ofstream::out);
    for (const auto &p : pairs) {
        output_file << p.first << ' ' << p.second << std::endl;
    }
}

} // namespace fileio
